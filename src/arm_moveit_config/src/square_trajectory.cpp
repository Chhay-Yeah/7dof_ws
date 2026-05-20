#include <array>
#include <cmath>
#include <memory>
#include <thread>
#include <atomic>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "demo_trigger.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "tf_trajectory",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto logger = rclcpp::get_logger("tf_trajectory");

    // ── MoveGroup setup ───────────────────────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
    move_group.setEndEffectorLink("ee");
    move_group.setPoseReferenceFrame("base_link");
    move_group.setGoalPositionTolerance(0.005);
    move_group.setGoalOrientationTolerance(0.05);
    move_group.allowReplanning(true);
    move_group.setNumPlanningAttempts(10);
    move_group.setPlanningTime(10.0);
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);

    RCLCPP_INFO(logger, "Planning frame: %s  |  Pose ref: %s",
        move_group.getPlanningFrame().c_str(),
        move_group.getPoseReferenceFrame().c_str());

    // ── Ctrl+C handler: registered via rclcpp shutdown callback (no race) ─────
    std::atomic<bool> interrupted{false};
    rclcpp::on_shutdown([&]() {
        interrupted = true;
        RCLCPP_WARN(logger, "Shutdown — stopping robot...");
        move_group.stop();
    });

    // Helper macro to abort cleanly if Ctrl+C was pressed
    auto check_ok = [&]() -> bool {
        if (interrupted || !rclcpp::ok()) {
            RCLCPP_WARN(logger, "Aborted by user.");
            return false;
        }
        return true;
    };

    // ── Display publisher (transient_local so RViz always catches it) ─────────
    auto qos = rclcpp::QoS(1).transient_local();
    auto display_pub = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "/display_planned_path", qos);

    // ── Marker publishers ───────────────────────────────────────────────────────
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/square_markers", rclcpp::QoS(1).transient_local());
    auto trail_pub = node->create_publisher<visualization_msgs::msg::Marker>(
        "/eef_trail", rclcpp::QoS(1).transient_local());
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ── Start live EEF trail tracker (runs through ALL steps) ─────────────────
    std::atomic<bool> tracking{true};
    visualization_msgs::msg::Marker trail;
    trail.header.frame_id = "base_link";
    trail.ns = "eef_trail"; trail.id = 0;
    trail.type   = visualization_msgs::msg::Marker::LINE_STRIP;
    trail.action = visualization_msgs::msg::Marker::ADD;
    trail.scale.x = 0.004;
    trail.color.r = 1.0f; trail.color.g = 1.0f; trail.color.b = 0.0f; trail.color.a = 1.0f;
    trail.pose.orientation.w = 1.0;
    std::thread tracker([&]() {
        while (tracking && rclcpp::ok())
        {
            auto p = move_group.getCurrentPose("ee").pose.position;
            geometry_msgs::msg::Point pt; pt.x = p.x; pt.y = p.y; pt.z = p.z;
            trail.header.stamp = node->now();
            trail.points.push_back(pt);
            trail_pub->publish(trail);
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
    });

    // Helper: publish trajectory to RViz (5 × 200 ms to survive late subscribers)
    auto publish_display = [&](const moveit_msgs::msg::RobotTrajectory& traj, const std::string& label)
    {
        moveit_msgs::msg::RobotState start_rs;
        auto current_state = move_group.getCurrentState(2.0);
        if (current_state)
            moveit::core::robotStateToRobotStateMsg(*current_state, start_rs);
        else
        {
            start_rs.is_diff = true;
            RCLCPP_WARN(logger, "Could not get current state, using is_diff");
        }
        moveit_msgs::msg::DisplayTrajectory dmsg;
        dmsg.model_id = move_group.getRobotModel()->getName();
        dmsg.trajectory.push_back(traj);
        dmsg.trajectory_start = start_rs;
        for (int i = 0; i < 5; ++i)
        {
            display_pub->publish(dmsg);
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
        RCLCPP_INFO(logger, "[%s] Published %zu trajectory points",
            label.c_str(), traj.joint_trajectory.points.size());
    };

    DemoTrigger trigger(node.get());

    auto run_demo = [&]() -> bool {
    // ── Step 1: go to home ────────────────────────────────────────────────────
    RCLCPP_INFO(logger, "Step 1: planning to home...");
    move_group.setNamedTarget("home");
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = false;
        for (int retry = 0; retry < 5 && !success; ++retry)
        {
            if (static_cast<bool>(move_group.plan(plan)))
                success = true;
            else
            {
                RCLCPP_WARN(logger, "Plan to home attempt %d/5 failed, retrying...", retry + 1);
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }
        }
        if (!success)
        {
            RCLCPP_ERROR(logger, "Failed to plan to home.");
            return false;
        }
        publish_display(plan.trajectory_, "home");
        rclcpp::sleep_for(std::chrono::seconds(2));
        if (!check_ok()) return false;
        move_group.execute(plan);
        if (!check_ok()) return false;
    }
    RCLCPP_INFO(logger, "Reached home.");

    // ── Step 2: move to lowered start pose (from image) ──────────────────────
    RCLCPP_INFO(logger, "Step 2: planning to lowered start pose...");
    {
        std::map<std::string, double> joint_targets = {
            {"joint_1",   0.0 * M_PI / 180.0},
            {"joint_2",  30.0 * M_PI / 180.0},
            {"joint_3",   0.0 * M_PI / 180.0},
            {"joint_4", -60.0 * M_PI / 180.0},
            {"joint_5",   0.0 * M_PI / 180.0},
            {"joint_6",   0.0 * M_PI / 180.0},
            {"joint_7",   0.0 * M_PI / 180.0},
        };
        move_group.setJointValueTarget(joint_targets);
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan_home;
    {
        bool success = false;
        for (int retry = 0; retry < 5 && !success; ++retry)
        {
            if (static_cast<bool>(move_group.plan(plan_home)))
                success = true;
            else
            {
                RCLCPP_WARN(logger, "Plan to start pose attempt %d/5 failed, retrying...", retry + 1);
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }
        }
        if (!success)
        {
            RCLCPP_ERROR(logger, "Failed to plan to start pose after 5 attempts.");
            return false;
        }
    }
    publish_display(plan_home.trajectory_, "start_pose");
    rclcpp::sleep_for(std::chrono::seconds(2));
    if (!check_ok()) return false;
    move_group.execute(plan_home);
    if (!check_ok()) return false;
    RCLCPP_INFO(logger, "Reached start pose.");

    // ── Step 3: move to the square start point ───────────────────────────────
    RCLCPP_INFO(logger, "Step 3: moving to square start point...");
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);

    auto eef_pose = move_group.getCurrentPose("ee").pose;

    // Center the square at current EEF position (ZY plane, X fixed)
    const double cx        = eef_pose.position.x + 0.02;  // keep arm extended forward
    const double cy        = eef_pose.position.y;
    const double cz        = eef_pose.position.z;
    const double half_side = 0.03;  // half the side length (full side = 6 cm)

    // Build square in ZY plane (z = up/down, y = left/right), CCW when viewed from +X
    // Corners: bottom-right → top-right → top-left → bottom-left → close
    const std::array<std::pair<double,double>, 5> corners = {{
        { cy + half_side, cz - half_side },  // 0: bottom-right  (start)
        { cy + half_side, cz + half_side },  // 1: top-right
        { cy - half_side, cz + half_side },  // 2: top-left
        { cy - half_side, cz - half_side },  // 3: bottom-left
        { cy + half_side, cz - half_side },  // 4: bottom-right  (close)
    }};

    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (const auto& [y, z] : corners)
    {
        geometry_msgs::msg::Pose p = eef_pose;
        p.position.x = cx;
        p.position.y = y;
        p.position.z = z;
        waypoints.push_back(p);
    }

    // ── Step 3: move directly to the first corner of the square ─────────────
    RCLCPP_INFO(logger, "Step 3: moving directly to square first corner...");
    {
        move_group.setStartStateToCurrentState();
        std::vector<geometry_msgs::msg::Pose> approach = { eef_pose, waypoints.front() };
        moveit_msgs::msg::RobotTrajectory approach_traj;
        double frac = 0.0;
        for (int i = 0; i < 10 && frac < 0.99; ++i)
            frac = move_group.computeCartesianPath(approach, 0.005, 0.0, approach_traj);
        if (frac < 0.5)
        {
            RCLCPP_WARN(logger, "Cartesian approach only %.1f%%, falling back to joint-space...", frac * 100.0);
            move_group.setPoseTarget(waypoints.front(), "ee");
            moveit::planning_interface::MoveGroupInterface::Plan fallback;
            bool ok = false;
            for (int retry = 0; retry < 5 && !ok; ++retry)
                ok = static_cast<bool>(move_group.plan(fallback));
            if (!ok)
            {
                RCLCPP_ERROR(logger, "Failed to reach square first corner.");
                return false;
            }
            publish_display(fallback.trajectory_, "to_corner1");
            rclcpp::sleep_for(std::chrono::seconds(2));
            if (!check_ok()) return false;
            move_group.execute(fallback);
        }
        else
        {
            moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
            approach_plan.trajectory_ = approach_traj;
            publish_display(approach_traj, "to_corner1");
            rclcpp::sleep_for(std::chrono::seconds(2));
            if (!check_ok()) return false;
            move_group.execute(approach_plan);
        }
        if (!check_ok()) return false;
    }
    RCLCPP_INFO(logger, "Reached square first corner.");

    // ── Step 4: Cartesian square ──────────────────────────────────────────────
    RCLCPP_INFO(logger, "Step 4: computing Cartesian square (half_side=%.2f m, 5 pts)...", half_side);

    // Publish planned square path as LINE_STRIP + center SPHERE in RViz
    {
        visualization_msgs::msg::MarkerArray ma;

        // LINE_STRIP — planned square
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "base_link";
        line.header.stamp    = node->now();
        line.ns = "square"; line.id = 0;
        line.type   = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.005;  // line width
        line.color.r = 0.0f; line.color.g = 0.8f; line.color.b = 1.0f; line.color.a = 1.0f;
        line.pose.orientation.w = 1.0;
        for (const auto& wp : waypoints)
        {
            geometry_msgs::msg::Point pt;
            pt.x = wp.position.x; pt.y = wp.position.y; pt.z = wp.position.z;
            line.points.push_back(pt);
        }
        ma.markers.push_back(line);

        // SPHERE — square center
        visualization_msgs::msg::Marker center;
        center.header = line.header;
        center.ns = "square"; center.id = 1;
        center.type   = visualization_msgs::msg::Marker::SPHERE;
        center.action = visualization_msgs::msg::Marker::ADD;
        center.pose.position.x = cx;
        center.pose.position.y = cy;
        center.pose.position.z = cz;
        center.pose.orientation.w = 1.0;
        center.scale.x = center.scale.y = center.scale.z = 0.015;
        center.color.r = 1.0f; center.color.g = 0.5f; center.color.b = 0.0f; center.color.a = 1.0f;
        ma.markers.push_back(center);

        marker_pub->publish(ma);
        RCLCPP_INFO(logger, "Published planned square markers.");
    }

    move_group.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory cart_traj;
    double fraction = 0.0;
    for (int i = 0; i < 20 && fraction < 0.99; ++i)
    {
        fraction = move_group.computeCartesianPath(
            waypoints, /*eef_step=*/0.005, /*jump_threshold=*/0.0, cart_traj);
        RCLCPP_INFO(logger, "  Cartesian attempt %d: %.1f%%", i + 1, fraction * 100.0);
    }

    if (fraction < 0.5)
    {
        RCLCPP_ERROR(logger, "Cartesian square too short (%.1f%%). Aborting.", fraction * 100.0);
        return false;
    }

    RCLCPP_INFO(logger, "Cartesian square %.1f%% — displaying...", fraction * 100.0);
    publish_display(cart_traj, "cartesian");
    rclcpp::sleep_for(std::chrono::seconds(3));  // watch in RViz
    if (!check_ok()) return false;

    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = cart_traj;
    move_group.clearPathConstraints();
    move_group.execute(cart_plan);
    if (!check_ok()) return false;
    RCLCPP_INFO(logger, "Cartesian square execution complete.");

    // ── Step 5: return to home ────────────────────────────────────────────────
    RCLCPP_INFO(logger, "Step 5: returning to home...");
    move_group.setNamedTarget("home");
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = false;
        for (int retry = 0; retry < 5 && !success; ++retry)
        {
            if (static_cast<bool>(move_group.plan(plan)))
                success = true;
            else
            {
                RCLCPP_WARN(logger, "Plan to home attempt %d/5 failed, retrying...", retry + 1);
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }
        }
        if (!success)
        {
            RCLCPP_ERROR(logger, "Failed to plan back to home.");
            return false;
        }
        publish_display(plan.trajectory_, "return_home");
        rclcpp::sleep_for(std::chrono::seconds(2));
        if (!check_ok()) return false;
        move_group.execute(plan);
    }
    RCLCPP_INFO(logger, "Back at home. Done.");
    return true;
    };  // end run_demo lambda

    run_demo();
    RCLCPP_INFO(logger, "Demo complete. Re-trigger:");
    RCLCPP_INFO(logger, "  ros2 service call /demo/run_service std_srvs/srv/Trigger {}");
    RCLCPP_INFO(logger, "  ros2 topic pub --once /demo/run std_msgs/msg/Empty {}");
    int republish_counter = 0;
    while (rclcpp::ok() && !interrupted) {
        if (trigger.consume()) run_demo();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        if (++republish_counter % 20 == 0) {
            trail.header.stamp = node->now();
            trail_pub->publish(trail);
        }
    }

    tracking = false;
    tracker.join();
    trail_pub->publish(trail);
    executor.cancel();
    spinner.join();
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
}