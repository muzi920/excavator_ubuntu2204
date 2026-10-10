#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <memory>
#include <string>

class RTSPCameraNode : public rclcpp::Node {
public:
    RTSPCameraNode() : Node("rtsp_camera_node") {
        // Declare parameters
        this->declare_parameter<std::string>("camera_name", "cam_hik");
        this->declare_parameter<std::string>("rtsp_url", "rtsp://admin:GWWzPzb2Tci@192.168.158.101:554/Streaming/Channels/101");
        this->declare_parameter<int>("fps", 15);
        this->declare_parameter<int>("target_width", 1920);
        this->declare_parameter<int>("target_height", 1080);

        std::string camera_name = this->get_parameter("camera_name").as_string();
        rtsp_url_ = this->get_parameter("rtsp_url").as_string();
        fps_ = this->get_parameter("fps").as_int();
        target_width_ = this->get_parameter("target_width").as_int();
        target_height_ = this->get_parameter("target_height").as_int();

        std::string topic_name = "/" + camera_name + "/image_raw";
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);

        RCLCPP_INFO(this->get_logger(), "Starting RTSP Camera Node for [%s]", camera_name.c_str());
        RCLCPP_INFO(this->get_logger(), "URL: %s", rtsp_url_.c_str());
        RCLCPP_INFO(this->get_logger(), "Target resolution: %dx%d", target_width_, target_height_);

        // Start capture thread
        capture_thread_ = std::thread(&RTSPCameraNode::capture_loop, this, camera_name);
    }

    ~RTSPCameraNode() {
        running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
    }

private:
    void capture_loop(const std::string& frame_id) {
        // Build GStreamer pipeline for low latency hardware decoding (FFMPEG fallback if no gstreamer)
        // using standard OpenCV VideoCapture
        
        cv::VideoCapture cap;
        
        while (rclcpp::ok() && running_) {
            if (!cap.isOpened()) {
                RCLCPP_INFO(this->get_logger(), "Connecting to RTSP stream...");
                // Setting environment variable for lower latency in FFMPEG
                // Using FFMPEG backend directly with low delay flags
                // If you have GStreamer installed, a proper gstreamer pipeline string can be used here.
                cap.open(rtsp_url_, cv::CAP_FFMPEG);
                
                if (!cap.isOpened()) {
                    RCLCPP_WARN(this->get_logger(), "Failed to open stream. Retrying in 2 seconds...");
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    continue;
                }
                RCLCPP_INFO(this->get_logger(), "Stream connected successfully!");
            }

            cv::Mat frame;
            if (cap.read(frame)) {
                if (!frame.empty()) {
                    if (target_width_ > 0 && target_height_ > 0 &&
                        (frame.cols != target_width_ || frame.rows != target_height_)) {
                        cv::resize(frame, frame, cv::Size(target_width_, target_height_), 0.0, 0.0, cv::INTER_LINEAR);
                    }

                    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                    msg->header.stamp = this->now();
                    msg->header.frame_id = frame_id;
                    publisher_->publish(*msg);
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Stream dropped. Reconnecting...");
                cap.release();
            }

            // Simple rate limiting
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / fps_));
        }
        
        if (cap.isOpened()) {
            cap.release();
        }
    }

    std::string rtsp_url_;
    int fps_;
    int target_width_;
    int target_height_;
    bool running_ = true;
    std::thread capture_thread_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RTSPCameraNode>());
    rclcpp::shutdown();
    return 0;
}
