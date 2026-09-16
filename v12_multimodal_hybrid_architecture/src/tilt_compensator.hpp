#ifndef TILT_COMPENSATOR_HPP
#define TILT_COMPENSATOR_HPP

#include <vector>
#include <array>
#include <cmath>
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

class TiltCompensator {
public:
    TiltCompensator(double alpha = 0.98);
    
    void update(double timestamp, const std::array<double, 3>& accel, const std::array<double, 3>& gyro);
    
    // Returns quaternion [x, y, z, w]
    std::array<double, 4> get_quaternion(double external_yaw_rad = 0.0);

private:
    double alpha_;
    double last_time_;
    double roll_;
    double pitch_;
    
    bool is_initialized_;
    int init_count_;
    double roll_sum_;
    double pitch_sum_;
    double roll0_;
    double pitch0_;

    std::mutex mutex_;
};

class SwingEstimator {
public:
    SwingEstimator() : is_calibrating_(true), calib_duration_(3.0), last_time_(-1.0), current_swing_rad_(0.0), last_w_yaw_(0.0) {}

    bool process_imu(double timestamp, const std::array<double, 3>& accel, const std::array<double, 3>& gyro, double& out_swing_rad) {
        (void)timestamp;

        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        const double current_time = std::chrono::duration<double>(now).count();

        if (last_time_ < 0) {
            last_time_ = current_time;
            calib_start_time_ = current_time;
            return false;
        }

        double dt = current_time - last_time_;
        last_time_ = current_time;

        if (dt <= 0 || dt > 0.5) return false;

        if (is_calibrating_) {
            calib_gyro_samples_.push_back(gyro);
            calib_accel_samples_.push_back(accel);

            if (current_time - calib_start_time_ >= calib_duration_) {
                double bx = 0, by = 0, bz = 0;
                for (const auto& g : calib_gyro_samples_) {
                    bx += g[0]; by += g[1]; bz += g[2];
                }
                bx /= calib_gyro_samples_.size();
                by /= calib_gyro_samples_.size();
                bz /= calib_gyro_samples_.size();
                gyro_bias_ = {bx, by, bz};

                double ax = 0, ay = 0, az = 0;
                for (const auto& a : calib_accel_samples_) {
                    ax += a[0]; ay += a[1]; az += a[2];
                }
                ax /= calib_accel_samples_.size();
                ay /= calib_accel_samples_.size();
                az /= calib_accel_samples_.size();

                double norm = std::sqrt(ax*ax + ay*ay + az*az);
                if (norm > 0) {
                    up_vector_ = {ax/norm, ay/norm, az/norm};
                } else {
                    up_vector_ = {0.0, 0.0, 1.0};
                }

                is_calibrating_ = false;
                RCLCPP_INFO(rclcpp::get_logger("SwingEstimator"), "Calibration Done! Up Vector: (%.3f, %.3f, %.3f)", up_vector_[0], up_vector_[1], up_vector_[2]);
            }
            return false;
        }

        double wx = gyro[0] - gyro_bias_[0];
        double wy = gyro[1] - gyro_bias_[1];
        double wz = gyro[2] - gyro_bias_[2];

        // 空间投影: 将本地 3D 角速度投影到真实的绝对垂直轴上
        double w_yaw = wx * up_vector_[0] + wy * up_vector_[1] + wz * up_vector_[2];

        // 与 V11 的 DirectSwingAngleEstimator 保持一致：
        // 这里输出的是底盘/控制约定下的回转角，正右负左。
        w_yaw = -w_yaw;

        if (std::abs(w_yaw) < 0.002) {
            w_yaw = 0.0;
        }

        // 梯形积分
        current_swing_rad_ += (w_yaw + last_w_yaw_) / 2.0 * dt;
        last_w_yaw_ = w_yaw;

        while (current_swing_rad_ > M_PI) {
            current_swing_rad_ -= 2.0 * M_PI;
        }
        while (current_swing_rad_ < -M_PI) {
            current_swing_rad_ += 2.0 * M_PI;
        }

        out_swing_rad = current_swing_rad_;
        return true;
    }

private:
    bool is_calibrating_;
    double calib_duration_;
    double calib_start_time_;
    double last_time_;
    double current_swing_rad_;
    double last_w_yaw_;

    std::vector<std::array<double, 3>> calib_gyro_samples_;
    std::vector<std::array<double, 3>> calib_accel_samples_;

    std::array<double, 3> gyro_bias_ = {0,0,0};
    std::array<double, 3> up_vector_ = {0,0,1};
};

#endif // TILT_COMPENSATOR_HPP
