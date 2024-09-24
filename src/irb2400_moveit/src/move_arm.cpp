#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "move_arm",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("move_arm");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "arm");

    // Set the maximum velocity scaling factor (e.g., 1.0 for 100% of max velocity)
    move_group_interface.setMaxVelocityScalingFactor(1.0);  // Increase velocity to maximum

    // Define 4 different target poses
    std::vector<geometry_msgs::msg::Pose> target_poses(4);

    //first goal 
    target_poses[0].orientation.w = 1.0;
    target_poses[0].position.x = 0.58;
    target_poses[0].position.y = 0.7;
    target_poses[0].position.z = 1.38;
 

    // Second goal
    target_poses[1].orientation.w = 1.0;
    target_poses[1].position.x = 0.68;
    target_poses[1].position.y = -0.72;
    target_poses[1].position.z = 0.5;

    // Third goal
    target_poses[2].orientation.w = 1.0;
    target_poses[2].position.x = 0.78;
    target_poses[2].position.y = 0.72;
    target_poses[2].position.z = 1.18;

    // Back to home position 
    target_poses[3].orientation.w = 1.0;
    target_poses[3].position.x = 0.910;
    target_poses[3].position.y = 0.0;
    target_poses[3].position.z = 1.436;

    // Iterate through the 4 goals
    for (const auto &target_pose : target_poses) {
        move_group_interface.setPoseTarget(target_pose);
        move_group_interface.setPlanningTime(10.0); 

        // Create a plan to the current target pose
        auto const [success, plan] = [&move_group_interface]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();

        // Execute the plan
        if (success) {
            move_group_interface.execute(plan);
            RCLCPP_INFO(logger, "Successfully moved to goal.");
        } else {
            RCLCPP_ERROR(logger, "Planning failed for one of the goals!");
            break;  // Stop if planning fails for any goal
        }
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
