#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <vector>
#include <mutex>
#include "tilt_compensator.hpp"

class LidarProcessorNode : public rclcpp::Node {
public:
    LidarProcessorNode() : Node("lidar_processor_node"), tilt_compensator_(0.98) {
        // V11 Extrinsic Calibration (map/lidar -> base_link)
        double tx = -0.5500, ty = -0.2000, tz = 1.2712;
        double roll = 3.0316, pitch = 0.0349, yaw = 0.0532;

        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
        
        Eigen::Matrix3f R = (yawAngle * pitchAngle * rollAngle).matrix();
        Eigen::Vector3f T(tx, ty, tz);

        R_inv_ = R.transpose();
        T_inv_ = -R_inv_ * T;

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&LidarProcessorNode::imu_callback, this, std::placeholders::_1));

        sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud", 10, std::bind(&LidarProcessorNode::pc_callback, this, std::placeholders::_1));
            
        pub_pc_base_link_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/points", 10);
        pub_pc_odom_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/points_odom", 10);

        RCLCPP_INFO(this->get_logger(), "Lidar Processor Node Started. Direct IMU Subscription.");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        std::array<double, 3> accel_raw = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
        std::array<double, 3> gyro_raw = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        
        static double external_yaw = 0.0;
        swing_estimator_.process_imu(timestamp, accel_raw, gyro_raw, external_yaw);

        Eigen::Vector3f a_raw(accel_raw[0], accel_raw[1], accel_raw[2]);
        Eigen::Vector3f g_raw(gyro_raw[0], gyro_raw[1], gyro_raw[2]);
        Eigen::Vector3f a_base = R_inv_ * a_raw;
        Eigen::Vector3f g_base = R_inv_ * g_raw;
        std::array<double, 3> accel_base = {a_base.x(), a_base.y(), a_base.z()};
        std::array<double, 3> gyro_base = {g_base.x(), g_base.y(), g_base.z()};

        tilt_compensator_.update(timestamp, accel_base, gyro_base);
        std::array<double, 4> quat = tilt_compensator_.get_quaternion(external_yaw);

        std::lock_guard<std::mutex> lock(imu_mutex_);
        Eigen::Quaternionf q(quat[3], quat[0], quat[1], quat[2]); // w, x, y, z
        current_R_odom_ = q.toRotationMatrix();
        imu_ready_ = true;
    }
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (msg->width * msg->height == 0) return;

        // Prepare output message
        sensor_msgs::msg::PointCloud2 out_msg;
        out_msg.header.stamp = msg->header.stamp;
        out_msg.header.frame_id = "base_link";
        out_msg.height = 1;
        out_msg.is_bigendian = false;
        out_msg.is_dense = true;

        sensor_msgs::PointCloud2Modifier modifier(out_msg);
        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::msg::PointField::UINT32);

        // Pre-allocate memory, worst case same size as input
        modifier.resize(msg->width * msg->height);

        sensor_msgs::PointCloud2Iterator<float> iter_x_out(out_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y_out(out_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z_out(out_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgb_out(out_msg, "rgb");

        sensor_msgs::PointCloud2ConstIterator<float> iter_x_in(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y_in(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z_in(*msg, "z");

        size_t valid_points = 0;

        for (; iter_x_in != iter_x_in.end(); ++iter_x_in, ++iter_y_in, ++iter_z_in) {
            Eigen::Vector3f p_map(*iter_x_in, *iter_y_in, *iter_z_in);
            Eigen::Vector3f p_base = R_inv_ * p_map + T_inv_;

            // Filter condition: x, y in (-3, 3) and z > -0.1
            if (p_base.x() > -3.0f && p_base.x() < 3.0f &&
                p_base.y() > -3.0f && p_base.y() < 3.0f &&
                p_base.z() > -0.1f) {
                
                // Z-axis Color Mapping (Jet Colormap: Blue -> Green -> Red)
                float z_min = -0.4f;
                float z_max = 0.7f;
                float v = std::clamp((p_base.z() - z_min) / (z_max - z_min), 0.0f, 1.0f);
                
                uint8_t r = 0, g = 0, b = 0;
                if (v < 0.25f) {
                    r = 0;
                    g = static_cast<uint8_t>(255.0f * (4.0f * v));
                    b = 255;
                } else if (v < 0.5f) {
                    r = 0;
                    g = 255;
                    b = static_cast<uint8_t>(255.0f * (1.0f - 4.0f * (v - 0.25f)));
                } else if (v < 0.75f) {
                    r = static_cast<uint8_t>(255.0f * (4.0f * (v - 0.5f)));
                    g = 255;
                    b = 0;
                } else {
                    r = 255;
                    g = static_cast<uint8_t>(255.0f * (1.0f - 4.0f * (v - 0.75f)));
                    b = 0;
                }
                
                uint32_t rgb = (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);

                *iter_x_out = p_base.x();
                *iter_y_out = p_base.y();
                *iter_z_out = p_base.z();
                *iter_rgb_out = rgb;

                ++iter_x_out;
                ++iter_y_out;
                ++iter_z_out;
                ++iter_rgb_out;
                valid_points++;
            }
        }

        modifier.resize(valid_points);
        if (valid_points > 0) {
            pub_pc_base_link_->publish(out_msg);

            // ----------------------------------------------------
            // Transform to odom frame
            // ----------------------------------------------------
            sensor_msgs::msg::PointCloud2 out_msg_odom = out_msg;
            out_msg_odom.header.frame_id = "odom";

            Eigen::Matrix3f R_odom;
            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                if (!imu_ready_) return; // Wait until IMU has initialized
                R_odom = current_R_odom_;
            }

            sensor_msgs::PointCloud2Iterator<float> iter_x_odom(out_msg_odom, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y_odom(out_msg_odom, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z_odom(out_msg_odom, "z");

            for (; iter_x_odom != iter_x_odom.end(); ++iter_x_odom, ++iter_y_odom, ++iter_z_odom) {
                // 按照您的建议：不再依赖 ROS TF！
                // 我们直接在点云节点里使用从 IMU 获取的最新的旋转矩阵进行正向乘法
                // odom = R_imu * base_link
                Eigen::Vector3f p_base(*iter_x_odom, *iter_y_odom, *iter_z_odom);
                Eigen::Vector3f p_odom = R_odom * p_base;
                *iter_x_odom = p_odom.x();
                *iter_y_odom = p_odom.y();
                *iter_z_odom = p_odom.z();
            }

            pub_pc_odom_->publish(out_msg_odom);
        }
    }

    Eigen::Matrix3f R_inv_;
    Eigen::Vector3f T_inv_;
    
    TiltCompensator tilt_compensator_;
    SwingEstimator swing_estimator_;
    std::mutex imu_mutex_;
    Eigen::Matrix3f current_R_odom_;
    bool imu_ready_ = false;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_base_link_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_odom_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
