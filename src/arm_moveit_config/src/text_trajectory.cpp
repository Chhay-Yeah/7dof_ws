// Draws the text "ECAM" in the air with the end-effector.
// Each letter is composed of one or more continuous strokes; between
// strokes the EE lifts (pen-up) so it doesn't draw connecting lines.
//
// Re-trigger after the first run finishes:
//   - press Enter in the terminal that started the node (works with `ros2 run`)
//   - publish to /demo/run from anywhere (works with `ros2 launch` too):
//       ros2 topic pub --once /demo/run std_msgs/msg/Empty {}

#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <atomic>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using Pose = geometry_msgs::msg::Pose;
using Stroke = std::vector<Pose>;

static Stroke make_stroke(const Pose& proto, double y0, double z0, double x_plane,
                          const std::vector<std::pair<double,double>>& pts)
{
    Stroke s;
    s.reserve(pts.size());
    for (const auto& [dy, dz] : pts) {
        Pose p = proto;
        p.position.x = x_plane;
        p.position.y = y0 - dy;
        p.position.z = z0 + dz;
        s.push_back(p);
    }
    return s;
}

static std::vector<Stroke> letter_E(const Pose& proto, double y0, double z0,
                                    double x_down, double W, double H)
{
    return {
        make_stroke(proto, y0, z0, x_down, {{0,H}, {0,0}, {W,0}}),
        make_stroke(proto, y0, z0, x_down, {{0,H}, {W,H}}),
        make_stroke(proto, y0, z0, x_down, {{0,H/2}, {W*0.8, H/2}}),
    };
}

static std::vector<Stroke> letter_C(const Pose& proto, double y0, double z0,
                                    double x_down, double W, double H)
{
    const double cy = y0 - W/2.0;
    const double cz = z0 + H/2.0;
    const double ry = W/2.0;
    const double rz = H/2.0;
    Stroke s;
    const int N = 14;
    for (int i = 0; i <= N; ++i) {
        double t = static_cast<double>(i) / N;
        double th = (30.0 + t * 300.0) * M_PI / 180.0;
        Pose p = proto;
        p.position.x = x_down;
        p.position.y = cy - ry * std::cos(th);
        p.position.z = cz + rz * std::sin(th);
        s.push_back(p);
    }
    return {s};
}

static std::vector<Stroke> letter_A(const Pose& proto, double y0, double z0,
                                    double x_down, double W, double H)
{
    return {
        make_stroke(proto, y0, z0, x_down, {{0,0}, {W/2.0, H}, {W, 0}}),
        make_stroke(proto, y0, z0, x_down, {{W*0.2, H*0.4}, {W*0.8, H*0.4}}),
    };
}

static std::vector<Stroke> letter_M(const Pose& proto, double y0, double z0,
                                    double x_down, double W, double H)
{
    return {
        make_stroke(proto, y0, z0, x_down, {{0,0}, {0,H}, {W/2.0, H*0.4}, {W, H}, {W, 0}}),
    };
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "text_trajectory",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto logger = rclcpp::get_logger("text_trajectory");

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

    std::atomic<bool> interrupted{false};
    auto check_ok = [&]() -> bool {
        if (interrupted || !rclcpp::ok()) {
            RCLCPP_WARN(logger, "Aborted by user.");
            return false;
        }
        return true;
    };

    auto qos_tl = rclcpp::QoS(1).transient_local();
    auto display_pub = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "/display_planned_path", qos_tl);
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/text_markers", qos_tl);
    auto trail_pub = node->create_publisher<visualization_msgs::msg::Marker>(
        "/eef_trail", qos_tl);

    // ── Re-run trigger: topic + stdin ────────────────────────────────────────
    std::atomic<bool> run_request{false};
    auto trigger_sub = node->create_subscription<std_msgs::msg::Empty>(
        "/demo/run", 10,
        [&](std_msgs::msg::Empty::SharedPtr) {
            RCLCPP_INFO(logger, ">>> /demo/run topic received — re-triggering");
            run_request = true;
        });

    auto trigger_srv = node->create_service<std_srvs::srv::Trigger>(
        "/demo/run_service",
        [&](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            RCLCPP_INFO(logger, ">>> /demo/run_service called — re-triggering");
            run_request = true;
            res->success = true;
            res->message = "Demo queued";
        });

    std::atomic<bool> stdin_alive{true};
    std::thread stdin_thread([&]() {
        std::string line;
        while (stdin_alive && rclcpp::ok()) {
            if (!std::getline(std::cin, line)) {
                // stdin closed (e.g. under ros2 launch) — exit thread silently
                return;
            }
            if (line == "q" || line == "Q") {
                interrupted = true;
                stdin_alive = false;
                return;
            }
            RCLCPP_INFO(logger, ">>> Enter pressed — re-triggering");
            run_request = true;
        }
    });

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ── EEF trail tracker ─────────────────────────────────────────────────────
    std::atomic<bool> tracking{true};
    visualization_msgs::msg::Marker trail;
    trail.header.frame_id = "base_link";
    trail.ns = "eef_trail"; trail.id = 0;
    trail.type   = visualization_msgs::msg::Marker::LINE_STRIP;
    trail.action = visualization_msgs::msg::Marker::ADD;
    trail.scale.x = 0.003;
    trail.color.r = 1.0f; trail.color.g = 1.0f; trail.color.b = 0.0f; trail.color.a = 1.0f;
    trail.pose.orientation.w = 1.0;
    std::mutex trail_mtx;
    std::thread tracker([&]() {
        while (tracking && rclcpp::ok())
        {
            auto p = move_group.getCurrentPose("ee").pose.position;
            geometry_msgs::msg::Point pt; pt.x = p.x; pt.y = p.y; pt.z = p.z;
            {
                std::lock_guard<std::mutex> lk(trail_mtx);
                trail.header.stamp = node->now();
                trail.points.push_back(pt);
                trail_pub->publish(trail);
            }
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
    });

    auto publish_display = [&](const moveit_msgs::msg::RobotTrajectory& traj, const std::string& label) {
        moveit_msgs::msg::RobotState start_rs;
        auto current_state = move_group.getCurrentState(2.0);
        if (current_state) moveit::core::robotStateToRobotStateMsg(*current_state, start_rs);
        else start_rs.is_diff = true;
        moveit_msgs::msg::DisplayTrajectory dmsg;
        dmsg.model_id = move_group.getRobotModel()->getName();
        dmsg.trajectory.push_back(traj);
        dmsg.trajectory_start = start_rs;
        for (int i = 0; i < 3; ++i) {
            display_pub->publish(dmsg);
            rclcpp::sleep_for(std::chrono::milliseconds(150));
        }
        RCLCPP_INFO(logger, "[%s] published %zu points", label.c_str(),
                    traj.joint_trajectory.points.size());
    };

    // ── One full demo pass ───────────────────────────────────────────────────
    auto run_demo = [&]() -> bool {
        RCLCPP_INFO(logger, "=== Starting ECAM demo pass ===");

        // Clear previous trail
        {
            std::lock_guard<std::mutex> lk(trail_mtx);
            trail.points.clear();
            trail.header.stamp = node->now();
            trail_pub->publish(trail);
        }

        // Step 1: home
        RCLCPP_INFO(logger, "Step 1: planning to home...");
        move_group.setNamedTarget("home");
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (!static_cast<bool>(move_group.plan(plan))) {
                RCLCPP_ERROR(logger, "Failed to plan to home.");
                return false;
            }
            publish_display(plan.trajectory_, "home");
            if (!check_ok()) return false;
            move_group.execute(plan);
        }

        // Step 2: writing pose
        RCLCPP_INFO(logger, "Step 2: moving to writing pose...");
        {
            std::map<std::string, double> jt = {
                {"joint_1",   0.0 * M_PI / 180.0},
                {"joint_2",  30.0 * M_PI / 180.0},
                {"joint_3",   0.0 * M_PI / 180.0},
                {"joint_4", -60.0 * M_PI / 180.0},
                {"joint_5",   0.0 * M_PI / 180.0},
                {"joint_6",   0.0 * M_PI / 180.0},
                {"joint_7",   0.0 * M_PI / 180.0},
            };
            move_group.setStartStateToCurrentState();
            move_group.setJointValueTarget(jt);
            moveit::planning_interface::MoveGroupInterface::Plan p;
            if (!static_cast<bool>(move_group.plan(p))) {
                RCLCPP_ERROR(logger, "Failed to plan to writing pose.");
                return false;
            }
            publish_display(p.trajectory_, "writing_pose");
            if (!check_ok()) return false;
            move_group.execute(p);
        }

        rclcpp::sleep_for(std::chrono::milliseconds(500));
        Pose home_eef = move_group.getCurrentPose("ee").pose;
        RCLCPP_INFO(logger, "Writing-pose EEF: x=%.3f y=%.3f z=%.3f",
                    home_eef.position.x, home_eef.position.y, home_eef.position.z);

        const double W   = 0.025;
        const double H   = 0.05;
        const double GAP = 0.012;
        const double x_down = home_eef.position.x + 0.02;
        const double x_up   = x_down - 0.020;

        const double total_w = 4.0 * W + 3.0 * GAP;
        const double y_left  = home_eef.position.y + total_w / 2.0;
        const double z_base  = home_eef.position.z - H / 2.0;

        std::vector<std::vector<Stroke>> letters = {
            letter_E(home_eef, y_left - 0 * (W + GAP), z_base, x_down, W, H),
            letter_C(home_eef, y_left - 1 * (W + GAP), z_base, x_down, W, H),
            letter_A(home_eef, y_left - 2 * (W + GAP), z_base, x_down, W, H),
            letter_M(home_eef, y_left - 3 * (W + GAP), z_base, x_down, W, H),
        };
        const std::vector<std::string> letter_names = {"E", "C", "A", "M"};

        // Markers: planned outline
        {
            visualization_msgs::msg::MarkerArray ma;
            visualization_msgs::msg::Marker del;
            del.header.frame_id = "base_link";
            del.header.stamp    = node->now();
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            ma.markers.push_back(del);
            marker_pub->publish(ma);
            ma.markers.clear();
            rclcpp::sleep_for(std::chrono::milliseconds(100));

            int id = 0;
            for (const auto& L : letters) {
                for (const auto& stroke : L) {
                    visualization_msgs::msg::Marker m;
                    m.header.frame_id = "base_link";
                    m.header.stamp = node->now();
                    m.ns = "ecam"; m.id = id++;
                    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
                    m.action = visualization_msgs::msg::Marker::ADD;
                    m.scale.x = 0.004;
                    m.color.r = 0.0f; m.color.g = 0.8f; m.color.b = 1.0f; m.color.a = 1.0f;
                    m.pose.orientation.w = 1.0;
                    for (const auto& p : stroke) {
                        geometry_msgs::msg::Point pt;
                        pt.x = p.position.x; pt.y = p.position.y; pt.z = p.position.z;
                        m.points.push_back(pt);
                    }
                    ma.markers.push_back(m);
                }
            }
            marker_pub->publish(ma);
            RCLCPP_INFO(logger, "Published planned ECAM outline (%zu strokes).", ma.markers.size());
        }

        auto pen_up_to = [&](const Pose& target_down) -> bool {
            Pose cur = move_group.getCurrentPose("ee").pose;
            Pose lift = cur;        lift.position.x = x_up;
            Pose hover = target_down; hover.position.x = x_up;
            std::vector<Pose> wps = { cur, lift, hover, target_down };
            moveit_msgs::msg::RobotTrajectory traj;
            double frac = 0.0;
            for (int i = 0; i < 10 && frac < 0.99; ++i)
                frac = move_group.computeCartesianPath(wps, 0.005, 0.0, traj);
            if (frac < 0.5) {
                RCLCPP_WARN(logger, "Pen-up move only %.0f%%, fallback to joint-space", frac*100);
                move_group.setStartStateToCurrentState();
                move_group.setPoseTarget(target_down, "ee");
                moveit::planning_interface::MoveGroupInterface::Plan p;
                if (!static_cast<bool>(move_group.plan(p))) return false;
                return static_cast<bool>(move_group.execute(p));
            }
            moveit::planning_interface::MoveGroupInterface::Plan p;
            p.trajectory_ = traj;
            return static_cast<bool>(move_group.execute(p));
        };

        auto draw_stroke = [&](const Stroke& stroke) -> bool {
            if (stroke.empty()) return true;
            Pose cur = move_group.getCurrentPose("ee").pose;
            std::vector<Pose> wps;
            wps.push_back(cur);
            for (const auto& p : stroke) wps.push_back(p);
            moveit_msgs::msg::RobotTrajectory traj;
            double frac = 0.0;
            for (int i = 0; i < 15 && frac < 0.99; ++i)
                frac = move_group.computeCartesianPath(wps, 0.005, 0.0, traj);
            if (frac < 0.7) {
                RCLCPP_WARN(logger, "Stroke Cartesian only %.0f%%, skipping", frac*100);
                return false;
            }
            moveit::planning_interface::MoveGroupInterface::Plan p;
            p.trajectory_ = traj;
            return static_cast<bool>(move_group.execute(p));
        };

        for (size_t li = 0; li < letters.size(); ++li) {
            RCLCPP_INFO(logger, "Drawing letter '%s' (%zu strokes)...",
                        letter_names[li].c_str(), letters[li].size());
            for (size_t si = 0; si < letters[li].size(); ++si) {
                if (!check_ok()) return false;
                const auto& stroke = letters[li][si];
                if (stroke.empty()) continue;
                if (!pen_up_to(stroke.front())) {
                    RCLCPP_WARN(logger, "  pen-up to stroke %zu failed, continuing", si);
                    continue;
                }
                if (!draw_stroke(stroke))
                    RCLCPP_WARN(logger, "  stroke %zu execute failed", si);
            }
        }

        // Lift, then return home
        {
            Pose cur = move_group.getCurrentPose("ee").pose;
            Pose lift = cur; lift.position.x = x_up;
            std::vector<Pose> wps = { cur, lift };
            moveit_msgs::msg::RobotTrajectory traj;
            double frac = 0.0;
            for (int i = 0; i < 5 && frac < 0.99; ++i)
                frac = move_group.computeCartesianPath(wps, 0.005, 0.0, traj);
            if (frac > 0.5) {
                moveit::planning_interface::MoveGroupInterface::Plan p;
                p.trajectory_ = traj;
                move_group.execute(p);
            }
        }

        RCLCPP_INFO(logger, "Returning to home...");
        move_group.setNamedTarget("home");
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (static_cast<bool>(move_group.plan(plan))) {
                publish_display(plan.trajectory_, "return_home");
                move_group.execute(plan);
            }
        }
        RCLCPP_INFO(logger, "=== Done writing ECAM ===");
        return true;
    };

    // Initial run
    run_demo();

    RCLCPP_INFO(logger, "Demo complete. To re-run:");
    RCLCPP_INFO(logger, "  - Service (button via rqt):  ros2 service call /demo/run_service std_srvs/srv/Trigger {}");
    RCLCPP_INFO(logger, "  - Topic:  ros2 topic pub --once /demo/run std_msgs/msg/Empty {}");
    RCLCPP_INFO(logger, "  - Press Enter in this terminal (only if launched via ros2 run)");
    RCLCPP_INFO(logger, "Ctrl+C (or 'q' + Enter) to exit.");

    while (rclcpp::ok() && !interrupted) {
        if (run_request.exchange(false)) {
            run_demo();
            RCLCPP_INFO(logger, "Ready for next trigger (/demo/run or Enter).");
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    tracking = false;
    tracker.join();
    stdin_alive = false;
    if (stdin_thread.joinable()) stdin_thread.detach();  // can't unblock getline cleanly
    executor.cancel();
    spinner.join();
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
}
