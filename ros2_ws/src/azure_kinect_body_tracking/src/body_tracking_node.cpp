#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <k4a/k4a.h>
#include <k4abt.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <unordered_map>
#include <string>
#include <vector>
#include <thread>
#include <unordered_set>

using namespace std::chrono_literals;

struct Joint
{
    float x;
    float y;
    float z;
};

class BodyTrackingNode : public rclcpp::Node
{
public:
    BodyTrackingNode()
    : Node("body_tracking_node")
    {
        joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("body_tracking_data", 10);
        joint_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("body_joints", 10);
        skeleton_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("body_skeleton", 10);
        rgb_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("rgb_image", 10);
        depth_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("depth_image", 10);

        timer_ = this->create_wall_timer(
            10ms, std::bind(&BodyTrackingNode::timer_callback, this));
        
        // Initialize Kinect
        k4a_device_open(K4A_DEVICE_DEFAULT, &device_);
        
        k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        device_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        device_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        device_config.camera_fps = K4A_FRAMES_PER_SECOND_15;

        k4a_device_start_cameras(device_, &device_config);
        k4a_device_start_imu(device_);

        k4a_calibration_t sensor_calibration;
        k4a_device_get_calibration(device_, device_config.depth_mode, device_config.color_resolution, &sensor_calibration);
        
        tracker_ = k4abt::tracker::create(sensor_calibration);

        // Initialize joint names
        initialize_joint_names();
    }

    ~BodyTrackingNode()
    {
        k4a_device_stop_cameras(device_);
        k4a_device_close(device_);
    }

private:
    void initialize_joint_names()
    {
        joint_names_ = {
            {0, "Pelvis"},
            {1, "Spine Navel"},
            {2, "Spine Chest"},
            {3, "Neck"},
            {4, "Clavicle Left"},
            {5, "Shoulder Left"},
            {6, "Elbow Left"},
            {7, "Wrist Left"},
            {8, "Hand Left"},
            {9, "Hand Tip Left"},
            {10, "Thumb Left"},
            {11, "Clavicle Right"},
            {12, "Shoulder Right"},
            {13, "Elbow Right"},
            {14, "Wrist Right"},
            {15, "Hand Right"},
            {16, "Hand Tip Right"},
            {17, "Thumb Right"},
            {18, "Hip Left"},
            {19, "Knee Left"},
            {20, "Ankle Left"},
            {21, "Foot Left"},
            {22, "Hip Right"},
            {23, "Knee Right"},
            {24, "Ankle Right"},
            {25, "Foot Right"},
            {26, "Head"},
            {27, "Nose"},
            {28, "Eye Left"},
            {29, "Ear Left"},
            {30, "Eye Right"},
            {31, "Ear Right"}
        };

        valid_joint_names_ = {
            "Pelvis", "Spine Navel", "Spine Chest", "Neck", "Clavicle Left", "Shoulder Left", 
            "Elbow Left", "Wrist Left", "Hand Left", "Clavicle Right", "Shoulder Right", 
            "Elbow Right", "Wrist Right", "Hand Right", "Hip Left", "Knee Left", "Ankle Left", 
            "Foot Left", "Hip Right", "Knee Right", "Ankle Right", "Foot Right", "Head"
        };
    }

    void timer_callback()
    {
        k4a_capture_t sensor_capture;
        if (k4a_device_get_capture(device_, &sensor_capture, K4A_WAIT_INFINITE) == K4A_WAIT_RESULT_SUCCEEDED)
        {
            std::thread processing_thread(&BodyTrackingNode::process_capture, this, sensor_capture);
            processing_thread.detach();
        }
    }

    void process_capture(k4a_capture_t sensor_capture)
    {
        k4a::capture capture(sensor_capture);
        publish_images(capture);  // Publish RGB and Depth images

        if (tracker_.enqueue_capture(capture))
        {
            k4abt::frame body_frame = tracker_.pop_result();
            uint32_t num_bodies = body_frame.get_num_bodies();

            for (uint32_t i = 0; i < num_bodies; ++i)
            {
                k4abt_body_t body = body_frame.get_body(i);
                
                std::unordered_map<std::string, Joint> joint_data;
                
                for (int j = 0; j < K4ABT_JOINT_COUNT; ++j)
                {
                    std::string joint_name = joint_names_[j];
                    if (valid_joint_names_.find(joint_name) != valid_joint_names_.end())
                    {
                        Joint joint;
                        joint.x = body.skeleton.joints[j].position.v[0] * 0.01;
                        joint.y = body.skeleton.joints[j].position.v[1] * 0.01;
                        joint.z = body.skeleton.joints[j].position.v[2] * 0.01;

                        joint_data[joint_name] = joint;
                    }
                }

                // Print all valid joint coordinates
                for (const auto& pair : joint_data)
                {
                    std::cout << pair.first << " coordinates: " 
                              << pair.second.x << ", " 
                              << pair.second.y << ", " 
                              << pair.second.z << std::endl;
                }

                // Publish the joint state data
                sensor_msgs::msg::JointState joint_state_msg;
                joint_state_msg.header.stamp = this->now();
                for (const auto& pair : joint_data)
                {
                    joint_state_msg.name.push_back(pair.first);
                    joint_state_msg.position.push_back(pair.second.x);
                    joint_state_msg.position.push_back(pair.second.y);
                    joint_state_msg.position.push_back(pair.second.z);
                }
                joint_publisher_->publish(joint_state_msg);

                // Publish the markers
                auto marker_joints = init_markers_spheres();
                auto marker_skeleton = init_markers_lines();

                create_markers(joint_data, marker_joints, marker_skeleton);
                joint_marker_publisher_->publish(marker_joints);
                skeleton_marker_publisher_->publish(marker_skeleton);
            }
        }
    }

    void create_markers(const std::unordered_map<std::string, Joint>& joint_data, 
                        visualization_msgs::msg::Marker& marker_joints, 
                        visualization_msgs::msg::Marker& marker_skeleton)
    {
        // Add points to joint marker
        for (const auto& pair : joint_data)
        {
            marker_joints.points.push_back(add_point_to_marker(pair.second));
        }

        // Add lines to skeleton marker
        std::vector<std::pair<std::string, std::string>> connections = {
            {"Pelvis", "Spine Navel"}, {"Spine Navel", "Spine Chest"}, {"Spine Chest", "Neck"},
            {"Neck", "Head"}, {"Neck", "Clavicle Left"}, {"Clavicle Left", "Shoulder Left"},
            {"Shoulder Left", "Elbow Left"}, {"Elbow Left", "Wrist Left"}, {"Wrist Left", "Hand Left"},
            {"Neck", "Clavicle Right"}, {"Clavicle Right", "Shoulder Right"},
            {"Shoulder Right", "Elbow Right"}, {"Elbow Right", "Wrist Right"},
            {"Wrist Right", "Hand Right"}, {"Pelvis", "Hip Left"}, {"Hip Left", "Knee Left"},
            {"Knee Left", "Ankle Left"}, {"Ankle Left", "Foot Left"}, {"Pelvis", "Hip Right"},
            {"Hip Right", "Knee Right"}, {"Knee Right", "Ankle Right"}, {"Ankle Right", "Foot Right"}
        };

        for (const auto& connection : connections)
        {
            if (joint_data.find(connection.first) != joint_data.end() && joint_data.find(connection.second) != joint_data.end())
            {
                marker_skeleton.points.push_back(add_point_to_marker(joint_data.at(connection.first)));
                marker_skeleton.points.push_back(add_point_to_marker(joint_data.at(connection.second)));
            }
        }
    }

    visualization_msgs::msg::Marker init_markers_spheres()
    {
        visualization_msgs::msg::Marker marker_joints;
        marker_joints.header.frame_id = "map";
        marker_joints.ns = "joints";
        marker_joints.id = 1;
        marker_joints.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker_joints.action = visualization_msgs::msg::Marker::ADD;
        marker_joints.scale.x = 0.7;
        marker_joints.scale.y = 0.7;
        marker_joints.scale.z = 0.7;
        marker_joints.color.a = 1.0;
        marker_joints.color.r = 1.0;
        marker_joints.color.g = 0.0;
        marker_joints.color.b = 0.0;
        marker_joints.lifetime = rclcpp::Duration(3s);
        return marker_joints;
    }

    visualization_msgs::msg::Marker init_markers_lines()
    {
        visualization_msgs::msg::Marker marker_line;
        marker_line.header.frame_id = "map";
        marker_line.ns = "joint_line";
        marker_line.id = 1;
        marker_line.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker_line.action = visualization_msgs::msg::Marker::ADD;
        marker_line.scale.x = 0.1;
        marker_line.color.a = 1.0;
        marker_line.color.r = 0.0;
        marker_line.color.g = 1.0;
        marker_line.color.b = 0.0;
        marker_line.lifetime = rclcpp::Duration(3s);
        return marker_line;
    }

    geometry_msgs::msg::Point add_point_to_marker(const Joint& joint)
    {
        geometry_msgs::msg::Point p;
        p.x = joint.x;
        p.y = joint.y;
        p.z = joint.z;
        return p;
    }

    void publish_images(const k4a::capture& capture)
    {
        auto color_image = capture.get_color_image();
        auto depth_image = capture.get_depth_image();

        if (color_image != nullptr)
        {
            auto color_data = color_image.get_buffer();
            cv::Mat color_mat(cv::Size(color_image.get_width_pixels(), color_image.get_height_pixels()), CV_8UC4, (void*)color_data, cv::Mat::AUTO_STEP);
            auto color_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgra8", color_mat).toImageMsg();
            color_msg->header.stamp = this->now();
            color_msg->header.frame_id = "camera_color_frame";
            rgb_image_publisher_->publish(*color_msg);
        }

        if (depth_image != nullptr)
        {
            auto depth_data = depth_image.get_buffer();
            cv::Mat depth_mat(cv::Size(depth_image.get_width_pixels(), depth_image.get_height_pixels()), CV_16U, (void*)depth_data, cv::Mat::AUTO_STEP);
            auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth_mat).toImageMsg();
            depth_msg->header.stamp = this->now();
            depth_msg->header.frame_id = "camera_depth_frame";
            depth_image_publisher_->publish(*depth_msg);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr joint_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr skeleton_marker_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    k4a_device_t device_;
    k4abt::tracker tracker_;

    std::unordered_map<int, std::string> joint_names_;
    std::unordered_set<std::string> valid_joint_names_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BodyTrackingNode>());
    rclcpp::shutdown();
    return 0;
}
