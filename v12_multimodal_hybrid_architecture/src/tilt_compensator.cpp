#include "tilt_compensator.hpp"

TiltCompensator::TiltCompensator(double alpha) 
    : alpha_(alpha), last_time_(-1.0), roll_(0.0), pitch_(0.0),
      is_initialized_(false), init_count_(0), roll_sum_(0.0), pitch_sum_(0.0),
      roll0_(0.0), pitch0_(0.0) {}

void TiltCompensator::update(double timestamp, const std::array<double, 3>& accel, const std::array<double, 3>& gyro) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    double ax = accel[0];
    double ay = accel[1];
    double az = accel[2];
    double gx = gyro[0];
    double gy = gyro[1];

    double accel_roll = std::atan2(ay, az);
    double accel_pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));

    if (last_time_ < 0) {
        last_time_ = timestamp;
        roll_ = accel_roll;
        pitch_ = accel_pitch;
        return;
    }

    double dt = timestamp - last_time_;
    if (dt <= 0.0 || dt > 0.5) {
        last_time_ = timestamp;
        return;
    }

    // 1. Gyro integration
    roll_ += gx * dt;
    pitch_ += gy * dt;

    // 2. Accelerometer correction
    double total_accel = std::sqrt(ax*ax + ay*ay + az*az);
    if (total_accel > 8.0 && total_accel < 11.5) {
        double current_alpha = alpha_;
        if (std::abs(total_accel - 9.8) > 2.0) {
            current_alpha = 0.999; 
        }

        roll_ = current_alpha * roll_ + (1.0 - current_alpha) * accel_roll;
        pitch_ = current_alpha * pitch_ + (1.0 - current_alpha) * accel_pitch;

        if (!is_initialized_) {
            roll_sum_ += accel_roll;
            pitch_sum_ += accel_pitch;
            init_count_++;
            if (init_count_ >= 50) {
                roll0_ = roll_sum_ / 50.0;
                pitch0_ = pitch_sum_ / 50.0;
                roll_ = roll0_;
                pitch_ = pitch0_;
                is_initialized_ = true;
                RCLCPP_INFO(rclcpp::get_logger("TiltCompensator"), "Tilt Calibration Done! roll0: %.3f, pitch0: %.3f", roll0_, pitch0_);
            }
        }
    }

    last_time_ = timestamp;
}

std::array<double, 4> TiltCompensator::get_quaternion(double external_yaw_rad) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 和 V11 的 Python 模板保持一致：
    // 1. 标定完成前，仅输出外部 yaw，避免把未校准的安装误差直接带进 odom 点云。
    // 2. 标定完成后，输出相对初始水平面的 roll/pitch，不额外取反。
    double rel_roll = 0.0;
    double rel_pitch = 0.0;
    if (is_initialized_) {
        rel_roll = roll_ - roll0_;
        rel_pitch = pitch_ - pitch0_;
    }
    
    double cr = std::cos(rel_roll * 0.5);
    double sr = std::sin(rel_roll * 0.5);
    double cp = std::cos(rel_pitch * 0.5);
    double sp = std::sin(rel_pitch * 0.5);
    double cy = std::cos(external_yaw_rad * 0.5);
    double sy = std::sin(external_yaw_rad * 0.5);

    // Standard formulation for Z-Y-X (yaw-pitch-roll)
    std::array<double, 4> q;
    q[0] = sr * cp * cy - cr * sp * sy; // x
    q[1] = cr * sp * cy + sr * cp * sy; // y
    q[2] = cr * cp * sy - sr * sp * cy; // z
    q[3] = cr * cp * cy + sr * sp * sy; // w
    
    return q;
}
