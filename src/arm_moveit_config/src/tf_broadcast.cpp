#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("end_link_tf_reader");

    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);

    rclcpp::Rate rate(10.0);

    while (rclcpp::ok())
    {
        try
        {
            geometry_msgs::msg::TransformStamped tf =
                tf_buffer->lookupTransform("base_link", "ee", tf2::TimePointZero);

            RCLCPP_INFO(node->get_logger(),
                "end_link -> x: %.4f  y: %.4f  z: %.4f  qx: %.4f  qy: %.4f  qz: %.4f  qw: %.4f",
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(node->get_logger(), "Could not get transform: %s", ex.what());
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}