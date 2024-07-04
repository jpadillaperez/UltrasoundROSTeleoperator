#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>
#include <string>
#include <vector>
#include <cmath>

struct Joint
{
    float x;
    float y;
    float z;
};

class JointAngleNode : public rclcpp::Node
{
public:
    JointAngleNode()
    : Node("joint_angle_node")
    {
        joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "body_tracking_data", 10, std::bind(&JointAngleNode::joint_callback, this, std::placeholders::_1));
    }

private:
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::unordered_map<std::string, Joint> joint_data;

        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            Joint joint;
            joint.x = msg->position[i*3];
            joint.y = msg->position[i*3 + 1];
            joint.z = msg->position[i*3 + 2];
            joint_data[msg->name[i]] = joint;
        }

        // Calculate angles for left arm
        if (joint_data.find("Shoulder Left") != joint_data.end() &&
            joint_data.find("Elbow Left") != joint_data.end() &&
            joint_data.find("Wrist Left") != joint_data.end())
        {
            float elbow_angle_left = calculate_angle(joint_data["Shoulder Left"], joint_data["Elbow Left"], joint_data["Wrist Left"]);
            float shoulder_angle_left = calculate_angle(joint_data["Elbow Left"], joint_data["Shoulder Left"], joint_data["Spine Shoulder"]);
            float wrist_angle_left = calculate_angle(joint_data["Elbow Left"], joint_data["Wrist Left"], joint_data["Hand Left"]);
            RCLCPP_INFO(this->get_logger(), "Left Elbow Angle: %.2f degrees", elbow_angle_left);
            RCLCPP_INFO(this->get_logger(), "Left Shoulder Angle: %.2f degrees", shoulder_angle_left);
            RCLCPP_INFO(this->get_logger(), "Left Wrist Angle: %.2f degrees", wrist_angle_left);
        }

        // Calculate angles for right arm
        if (joint_data.find("Shoulder Right") != joint_data.end() &&
            joint_data.find("Elbow Right") != joint_data.end() &&
            joint_data.find("Wrist Right") != joint_data.end())
        {
            float elbow_angle_right = calculate_angle(joint_data["Shoulder Right"], joint_data["Elbow Right"], joint_data["Wrist Right"]);
            float shoulder_angle_right = calculate_angle(joint_data["Elbow Right"], joint_data["Shoulder Right"], joint_data["Spine Shoulder"]);
            float wrist_angle_right = calculate_angle(joint_data["Elbow Right"], joint_data["Wrist Right"], joint_data["Hand Right"]);
            RCLCPP_INFO(this->get_logger(), "Right Elbow Angle: %.2f degrees", elbow_angle_right);
            RCLCPP_INFO(this->get_logger(), "Right Shoulder Angle: %.2f degrees", shoulder_angle_right);
            RCLCPP_INFO(this->get_logger(), "Right Wrist Angle: %.2f degrees", wrist_angle_right);
        }
    }

    float calculate_angle(const Joint& a, const Joint& b, const Joint& c)
    {
        auto vec1 = std::vector<float>{b.x - a.x, b.y - a.y, b.z - a.z};
        auto vec2 = std::vector<float>{c.x - b.x, c.y - b.y, c.z - b.z};

        float dot_product = vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2];
        float magnitude1 = std::sqrt(vec1[0]*vec1[0] + vec1[1]*vec1[1] + vec1[2]*vec1[2]);
        float magnitude2 = std::sqrt(vec2[0]*vec2[0] + vec2[1]*vec2[1] + vec2[2]*vec2[2]);

        float angle_rad = std::acos(dot_product / (magnitude1 * magnitude2));
        float angle_deg = angle_rad * (180.0 / M_PI);

        return angle_deg;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointAngleNode>());
    rclcpp::shutdown();
    return 0;
}
