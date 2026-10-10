#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <vector>
#include <cmath>
#include <array>
#include <mutex>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "tilt_compensator.hpp"

class ImuSensorNode : public rclcpp::Node {
public:
    ImuSensorNode() : Node("imu_sensor_node"), tilt_compensator_(0.98) {
        // Initialize Extrinsic Rotation (Lidar -> base_link)
        double roll = 3.0316, pitch = 0.0349, yaw = 0.0532;
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f R = (yawAngle * pitchAngle * rollAngle).matrix();
        R_inv_ = R.transpose();

        // We subscribe to the official LiDAR IMU topic
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&ImuSensorNode::imu_callback, this, std::placeholders::_1));

        inclinometer_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/excavator/inclinometer_relative_deg", 10,
            std::bind(&ImuSensorNode::inclinometer_callback, this, std::placeholders::_1));
            
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/excavator/joint_states", 10);
        joint_angle_deg_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/excavator/joint_angles_deg", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "IMU Sensor Node with C++ TiltCompensator Started.");
    }

private:
    void inclinometer_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 3) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Received relative inclinometer data with insufficient length: %zu", msg->data.size());
            return;
        }

        std::lock_guard<std::mutex> lock(sensor_mutex_);
        diff_ba_deg_ = msg->data[0];
        diff_ab_deg_ = msg->data[1];
        diff_bs_deg_ = msg->data[2];
        sensor_ready_ = true;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
        std::array<double, 3> accel_raw = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
        std::array<double, 3> gyro_raw = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        
        // --- 核心修复：坐标系对齐 ---
        // 1. SwingEstimator 需要最原始的雷达 IMU 数据（它会自动校准出雷达坐标系下的 up_vector）
        static double swing_yaw_v4 = 0.0;
        swing_estimator_.process_imu(timestamp, accel_raw, gyro_raw, swing_yaw_v4);
        const double ros_yaw = -swing_yaw_v4;

        // 2. TiltCompensator 需要 base_link 坐标系下的数据！
        // 因为雷达是倾斜安装的 (roll 约 173 度，倒扣)，如果不转换，TiltCompensator 算出的重力是反的 (Z轴颠倒)。
        Eigen::Vector3f a_raw(accel_raw[0], accel_raw[1], accel_raw[2]);
        Eigen::Vector3f g_raw(gyro_raw[0], gyro_raw[1], gyro_raw[2]);
        Eigen::Vector3f a_base = R_inv_ * a_raw;
        Eigen::Vector3f g_base = R_inv_ * g_raw;
        std::array<double, 3> accel_base = {a_base.x(), a_base.y(), a_base.z()};
        std::array<double, 3> gyro_base = {g_base.x(), g_base.y(), g_base.z()};

        // Update Tilt Compensator with base_link aligned IMU data
        tilt_compensator_.update(timestamp, accel_base, gyro_base);
        
        std::array<double, 4> quat = tilt_compensator_.get_quaternion(ros_yaw);
        
        // 3. Publish Odom -> base_link TF
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = msg->header.stamp;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        
        t.transform.rotation.x = quat[0];
        t.transform.rotation.y = quat[1];
        t.transform.rotation.z = quat[2];
        t.transform.rotation.w = quat[3];
        
        tf_broadcaster_->sendTransform(t);
        
        // 4. Publish Joint States
        double diff_ba = 0.0;
        double diff_ab = 0.0;
        double diff_bs = 0.0;
        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            if (sensor_ready_) {
                diff_ba = diff_ba_deg_;
                diff_ab = diff_ab_deg_;
                diff_bs = diff_bs_deg_;
            }
        }

        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = msg->header.stamp;
        joint_msg.name = {"boom_joint", "arm_joint", "bucket_joint", "swing_joint"};
        joint_msg.position = {
            diff_bs * M_PI / 180.0,
            diff_ab * M_PI / 180.0,
            diff_ba * M_PI / 180.0,
            swing_yaw_v4
        };
        joint_state_pub_->publish(joint_msg);

        // Publish degree-based joint angles for the angle controller, using the
        // same joint order as /excavator/joint_states.
        auto joint_angle_deg_msg = std_msgs::msg::Float64MultiArray();
        joint_angle_deg_msg.data = {
            diff_bs,
            diff_ab,
            diff_ba,
            swing_yaw_v4 * 180.0 / M_PI
        };
        joint_angle_deg_pub_->publish(joint_angle_deg_msg);
    }

    TiltCompensator tilt_compensator_;
    SwingEstimator swing_estimator_;
    Eigen::Matrix3f R_inv_;
    std::mutex sensor_mutex_;
    double diff_ba_deg_ = 0.0;
    double diff_ab_deg_ = 0.0;
    double diff_bs_deg_ = 0.0;
    bool sensor_ready_ = false;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr inclinometer_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_angle_deg_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuSensorNode>());
    rclcpp::shutdown();
    return 0;
}
