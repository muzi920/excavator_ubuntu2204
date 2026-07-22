import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import tf2_ros
from geometry_msgs.msg import Vector3Stamped
import tf2_geometry_msgs
import math

class SwingAngleEstimator(Node):
    def __init__(self):
        super().__init__('swing_angle_estimator')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 1. 提高队列长度至 200，防止高频 IMU 数据丢包导致积分漏算
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            200 
        )
        
        self.swing_pub = self.create_publisher(Float32, '/imu/swing_angle', 10)
        
        self.current_swing_rad = 0.0
        self.last_time = None
        self.last_w_z = 0.0
        
        # --- 缓存 TF，避免高频查表导致的 CPU 瓶颈 ---
        self.cached_transform = None
        
        # --- 零偏动态校准相关变量 ---
        self.is_calibrating = True
        self.calib_samples = []
        self.calib_start_time = None
        self.CALIB_DURATION = 3.0  # 开机静止校准时间(秒)
        self.gyro_bias_z = 0.0
        
        self.get_logger().info("Swing Angle Estimator started.")
        self.get_logger().info("【重要】正在进行陀螺仪零偏校准 (3秒)，请保持挖掘机绝对静止！...")

    def imu_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_time is None:
            self.last_time = current_time
            self.calib_start_time = current_time
            return
            
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 异常时间戳过滤
        if dt <= 0 or dt > 0.5:
            return
            
        try:
            frame_id = msg.header.frame_id if msg.header.frame_id else 'map'
            
            # 高频优化：只在启动时查询一次静态 TF，之后复用缓存，极大地提高解算频率
            if self.cached_transform is None:
                self.cached_transform = self.tf_buffer.lookup_transform('base_link', frame_id, rclpy.time.Time())
                self.get_logger().info("【TF缓存】已成功获取并缓存 map -> base_link 坐标变换，开启高频解算！")
            
            # TF 向量旋转
            angular_velocity_in = Vector3Stamped()
            angular_velocity_in.header = msg.header
            angular_velocity_in.header.frame_id = frame_id
            angular_velocity_in.vector = msg.angular_velocity
            
            angular_velocity_out = tf2_geometry_msgs.do_transform_vector3(angular_velocity_in, self.cached_transform)
            raw_w_z = angular_velocity_out.vector.z
            
            # 2. 动态零偏校准阶段
            if self.is_calibrating:
                self.calib_samples.append(raw_w_z)
                if current_time - self.calib_start_time >= self.CALIB_DURATION:
                    self.gyro_bias_z = sum(self.calib_samples) / len(self.calib_samples)
                    self.is_calibrating = False
                    self.get_logger().info(f"【校准完成】Z轴零偏为: {self.gyro_bias_z:.6f} rad/s，可以开始运动了。")
                return
            
            # 3. 扣除零偏
            w_z_corrected = raw_w_z - self.gyro_bias_z
            
            # 4. 极小死区滤波 (0.002 rad/s 约等于 0.1 deg/s)
            # 大幅减小死区，防止丢掉起步和刹车时的缓慢转动数据
            if abs(w_z_corrected) < 0.002:
                w_z_corrected = 0.0
                
            # 5. 梯形积分算法 (比之前的矩形欧拉积分精确得多)
            self.current_swing_rad += (w_z_corrected + self.last_w_z) / 2.0 * dt
            self.last_w_z = w_z_corrected
            
            swing_deg = math.degrees(self.current_swing_rad)
            
            # 约束在 [-180, 180] (如果希望记录多圈累计角度，可以注释掉这部分)
            while swing_deg > 180.0:
                swing_deg -= 360.0
            while swing_deg < -180.0:
                swing_deg += 360.0
                
            swing_msg = Float32()
            swing_msg.data = swing_deg
            self.swing_pub.publish(swing_msg)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SwingAngleEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
