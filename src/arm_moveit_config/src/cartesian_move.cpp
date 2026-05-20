#include <cmath>
#include <memory>
#include <thread>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "demo_trigger.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "tf_trajectory",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::MultiThreadedExecutor executor;
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

    // ── Static TF broadcaster (path_start / path_end) ───────────────────────
    auto static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    // ── Display publisher (transient_local so RViz always catches it) ─────────
    auto qos = rclcpp::QoS(1).transient_local();
    auto display_pub = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "/display_planned_path", qos);

    // ── Marker publishers ───────────────────────────────────────────────────────
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/square_markers", rclcpp::QoS(1).transient_local());
    auto trail_pub = node->create_publisher<visualization_msgs::msg::Marker>(
        "/eef_trail", rclcpp::QoS(100).transient_local());
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

    // ── Step 1b: Joint-space move to init pose ────────────────────────────────
    RCLCPP_INFO(logger, "Step 1b: Moving to init joint pose...");
    {
        std::map<std::string, double> joint_targets = {
            {"joint_1",  0.0 * M_PI / 180.0},
            {"joint_2", 15.0 * M_PI / 180.0},
            {"joint_3",  0.0 * M_PI / 180.0},
            {"joint_4", -80.0 * M_PI / 180.0},
            {"joint_5",  0.0 * M_PI / 180.0},
            {"joint_6",  0.0 * M_PI / 180.0},
            {"joint_7",  5.0 * M_PI / 180.0},
        };

        move_group.setStartStateToCurrentState();
        move_group.setJointValueTarget(joint_targets);
        moveit::planning_interface::MoveGroupInterface::Plan init_plan;
        bool ok = false;
        for (int retry = 0; retry < 5 && !ok; ++retry)
            ok = static_cast<bool>(move_group.plan(init_plan));
        if (!ok)
        {
            RCLCPP_ERROR(logger, "Failed to plan to init pose.");
            return false;
        }
        publish_display(init_plan.trajectory_, "init_pose");
        rclcpp::sleep_for(std::chrono::seconds(2));
        if (!check_ok()) return false;
        move_group.execute(init_plan);
        if (!check_ok()) return false;
    }
    RCLCPP_INFO(logger, "Reached init pose.");

    // ── Capture EEF pose at init ──────────────────────────────────────────────
    rclcpp::sleep_for(std::chrono::milliseconds(500));  // let state settle
    geometry_msgs::msg::Pose home_eef = move_group.getCurrentPose("ee").pose;
    RCLCPP_INFO(logger, "Home EEF: x=%.3f  y=%.3f  z=%.3f",
        home_eef.position.x, home_eef.position.y, home_eef.position.z);

    // ── Define left-to-right waypoints (±20 cm in Y, X/Z fixed) ─────────────
    const double half_travel = 0.1;   // 20 cm

    geometry_msgs::msg::Pose pose_left  = home_eef;
    geometry_msgs::msg::Pose pose_right = home_eef;
    pose_left.position.x  += 0.03;          // forward 10 cm
    pose_left.position.z  -= 0.04; 
    pose_right.position.x += 0.03;          // forward 10 cm
    pose_right.position.z += 0.05; 
    pose_left.position.y  -= half_travel + 0.02;   // left  (−20 cm)
    pose_right.position.y += half_travel - 0.02;   // right (+20 cm)

    // ── Step 2: Cartesian move home → left start point ───────────────────────
    RCLCPP_INFO(logger, "Step 2: Cartesian move to left start point (y = %.3f)...", pose_left.position.y);
    {
        move_group.setStartStateToCurrentState();
        std::vector<geometry_msgs::msg::Pose> approach = { home_eef, pose_left };
        moveit_msgs::msg::RobotTrajectory approach_traj;
        double frac = 0.0;
        for (int i = 0; i < 10 && frac < 0.99; ++i)
            frac = move_group.computeCartesianPath(approach, 0.005, 1.5, approach_traj);
        RCLCPP_INFO(logger, "Cartesian approach to left: %.1f%%", frac * 100.0);
        if (frac < 0.5)
        {
            RCLCPP_WARN(logger, "Cartesian approach only %.1f%%, falling back to joint-space...", frac * 100.0);
            move_group.setPoseTarget(pose_left, "ee");
            moveit::planning_interface::MoveGroupInterface::Plan fallback;
            bool ok = false;
            for (int retry = 0; retry < 5 && !ok; ++retry)
                ok = static_cast<bool>(move_group.plan(fallback));
            if (!ok)
            {
                RCLCPP_ERROR(logger, "Failed to reach left start point.");
                return false;
            }
            publish_display(fallback.trajectory_, "to_left_start");
            rclcpp::sleep_for(std::chrono::seconds(2));
            if (!check_ok()) return false;
            move_group.execute(fallback);
        }
        else
        {
            moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
            approach_plan.trajectory_ = approach_traj;
            publish_display(approach_traj, "to_left_start");
            rclcpp::sleep_for(std::chrono::seconds(2));
            if (!check_ok()) return false;
            move_group.execute(approach_plan);
        }
        if (!check_ok()) return false;
    }
    RCLCPP_INFO(logger, "Reached left start point.");

    // ── Step 3: Cartesian path left → right (−20 cm → +20 cm in Y) ──────────
    RCLCPP_INFO(logger, "Step 3: computing Cartesian path left → right (%.0f cm total)...",
        half_travel * 2.0 * 100.0);

    std::vector<geometry_msgs::msg::Pose> waypoints = { pose_left, pose_right };

    // Publish planned line marker in RViz
    {
        visualization_msgs::msg::MarkerArray ma;

        // Clear all stale markers from previous runs first
        {
            visualization_msgs::msg::Marker del;
            del.header.frame_id = "base_link";
            del.header.stamp    = node->now();
            del.ns = "";
            del.id = 0;
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            ma.markers.push_back(del);
            marker_pub->publish(ma);
            ma.markers.clear();
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        visualization_msgs::msg::Marker line;
        line.header.frame_id = "base_link";
        line.header.stamp    = node->now();
        line.ns = "ltr_path"; line.id = 0;
        line.type   = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.005;
        line.color.r = 0.0f; line.color.g = 0.8f; line.color.b = 1.0f; line.color.a = 1.0f;
        line.pose.orientation.w = 1.0;
        for (const auto& wp : waypoints)
        {
            geometry_msgs::msg::Point pt;
            pt.x = wp.position.x; pt.y = wp.position.y; pt.z = wp.position.z;
            line.points.push_back(pt);
        }
        ma.markers.push_back(line);

        // SPHERE — path center (home EEF)
        visualization_msgs::msg::Marker center;
        center.header = line.header;
        center.ns = "ltr_path"; center.id = 1;
        center.type   = visualization_msgs::msg::Marker::SPHERE;
        center.action = visualization_msgs::msg::Marker::ADD;
        center.pose.position = home_eef.position;
        center.pose.orientation.w = 1.0;
        center.scale.x = center.scale.y = center.scale.z = 0.015;
        center.color.r = 1.0f; center.color.g = 0.5f; center.color.b = 0.0f; center.color.a = 1.0f;
        ma.markers.push_back(center);

        // TEXT — "Start" at pose_left
        visualization_msgs::msg::Marker text_start;
        text_start.header = line.header;
        text_start.ns = "label_start"; text_start.id = 0;
        text_start.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_start.action = visualization_msgs::msg::Marker::ADD;
        text_start.pose.position    = pose_left.position;
        text_start.pose.position.z += 0.05;   // lift label above point
        text_start.pose.orientation.w = 1.0;
        text_start.scale.z = 0.03;            // text height
        text_start.color.r = 0.0f; text_start.color.g = 1.0f; text_start.color.b = 0.3f; text_start.color.a = 1.0f;
        text_start.text = "Start";
        ma.markers.push_back(text_start);

        // TEXT — "End" at pose_right
        visualization_msgs::msg::Marker text_end;
        text_end.header = line.header;
        text_end.ns = "label_end"; text_end.id = 0;
        text_end.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_end.action = visualization_msgs::msg::Marker::ADD;
        text_end.pose.position    = pose_right.position;
        text_end.pose.position.z += 0.05;
        text_end.pose.orientation.w = 1.0;
        text_end.scale.z = 0.03;
        text_end.color.r = 1.0f; text_end.color.g = 0.3f; text_end.color.b = 0.0f; text_end.color.a = 1.0f;
        text_end.text = "End";
        ma.markers.push_back(text_end);

        marker_pub->publish(ma);
        RCLCPP_INFO(logger, "Published left-to-right path markers.");

        // ── Broadcast static TF frames at path start and end ─────────────────
        std::vector<geometry_msgs::msg::TransformStamped> tf_frames;

        geometry_msgs::msg::TransformStamped tf_start;
        tf_start.header.stamp    = node->now();
        tf_start.header.frame_id = "base_link";
        tf_start.child_frame_id  = "path_start";
        tf_start.transform.translation.x = pose_left.position.x;
        tf_start.transform.translation.y = pose_left.position.y;
        tf_start.transform.translation.z = pose_left.position.z;
        tf_start.transform.rotation      = pose_left.orientation;
        tf_frames.push_back(tf_start);

        geometry_msgs::msg::TransformStamped tf_end;
        tf_end.header.stamp    = node->now();
        tf_end.header.frame_id = "base_link";
        tf_end.child_frame_id  = "path_end";
        tf_end.transform.translation.x = pose_right.position.x;
        tf_end.transform.translation.y = pose_right.position.y;
        tf_end.transform.translation.z = pose_right.position.z;
        tf_end.transform.rotation      = pose_right.orientation;
        tf_frames.push_back(tf_end);

        static_tf_broadcaster->sendTransform(tf_frames);
        RCLCPP_INFO(logger, "Broadcast TF: path_start  (%.3f, %.3f, %.3f)",
            pose_left.position.x, pose_left.position.y, pose_left.position.z);
        RCLCPP_INFO(logger, "Broadcast TF: path_end    (%.3f, %.3f, %.3f)",
            pose_right.position.x, pose_right.position.y, pose_right.position.z);
    }

    move_group.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory cart_traj;
    double fraction = 0.0;
    for (int i = 0; i < 20 && fraction < 0.99; ++i)
    {
        fraction = move_group.computeCartesianPath(
            waypoints, /*eef_step=*/0.005, /*jump_threshold=*/1.5, cart_traj);
        RCLCPP_INFO(logger, "  Cartesian attempt %d: %.1f%%", i + 1, fraction * 100.0);
    }

    if (fraction < 0.5)
    {
        RCLCPP_ERROR(logger, "Cartesian path too short (%.1f%%). Aborting.", fraction * 100.0);
        return false;
    }

    RCLCPP_INFO(logger, "Cartesian path %.1f%% — displaying...", fraction * 100.0);
    publish_display(cart_traj, "ltr_cartesian");
    rclcpp::sleep_for(std::chrono::seconds(3));
    if (!check_ok()) return false;

    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = cart_traj;
    move_group.clearPathConstraints();
    move_group.execute(cart_plan);
    if (!check_ok()) return false;
    RCLCPP_INFO(logger, "Left-to-right Cartesian path execution complete.");

    // ── Step 4: Cartesian return to home ─────────────────────────────────────
    RCLCPP_INFO(logger, "Step 4: Cartesian return to home...");
    {
        move_group.setStartStateToCurrentState();
        auto cur_eef = move_group.getCurrentPose("ee").pose;
        std::vector<geometry_msgs::msg::Pose> return_wps = { cur_eef, home_eef };
        moveit_msgs::msg::RobotTrajectory return_traj;
        double frac = 0.0;
        for (int i = 0; i < 10 && frac < 0.99; ++i)
            frac = move_group.computeCartesianPath(return_wps, 0.005, 1.5, return_traj);
        RCLCPP_INFO(logger, "Cartesian return to home: %.1f%%", frac * 100.0);
        if (frac < 0.5)
        {
            RCLCPP_WARN(logger, "Cartesian return only %.1f%%, falling back to joint-space...", frac * 100.0);
            move_group.setNamedTarget("home");
            moveit::planning_interface::MoveGroupInterface::Plan fallback;
            bool ok = false;
            for (int retry = 0; retry < 5 && !ok; ++retry)
                ok = static_cast<bool>(move_group.plan(fallback));
            if (!ok)
            {
                RCLCPP_ERROR(logger, "Failed to plan back to home.");
                return false;
            }
            publish_display(fallback.trajectory_, "return_home");
            rclcpp::sleep_for(std::chrono::seconds(2));
            if (!check_ok()) return false;
            move_group.execute(fallback);
        }
        else
        {
            moveit::planning_interface::MoveGroupInterface::Plan return_plan;
            return_plan.trajectory_ = return_traj;
            publish_display(return_traj, "return_home");
            rclcpp::sleep_for(std::chrono::seconds(2));
            if (!check_ok()) return false;
            move_group.execute(return_plan);
        }
        if (!check_ok()) return false;
    }
    RCLCPP_INFO(logger, "Back at init pose.");

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
    while (rclcpp::ok() && !interrupted) {
        if (trigger.consume()) run_demo();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    tracking = false;
    tracker.join();
    trail_pub->publish(trail);
    executor.cancel();
    spinner.join();
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
}