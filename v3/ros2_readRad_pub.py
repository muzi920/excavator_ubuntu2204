import os
import sys
import time
import glob
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import math

# 将官方 SDK 路径加入环境变量以便引用
sys.path.append(os.path.join(os.path.dirname(__file__), "WitStandardModbus_WT901C485-main", "Python", "Python-SDK-WT901C485_new"))
import device_model

id_to_name = {
    0x50: "铲斗",
    0x51: "小臂",
    0x52: "大臂",
    0x53: "回转"
}

# 用于记录上一次发布的时间，限制过高的异常发布频率（避免解析错乱时的数据粘连/时间错误）
last_published_time = {
    0x50: 0,
    0x51: 0,
    0x52: 0,
    0x53: 0
}

# 用于记录初始的绝对 AngX 角度
init_abs_ang_x = {
    0x50: None,
    0x51: None,
    0x52: None,
    0x53: None
}

# 用于记录最新的绝对 AngX 角度，以计算相对角度
latest_abs_ang_x = {
    0x50: 0.0,
    0x51: 0.0,
    0x52: 0.0,
    0x53: 0.0
}

class ImuPublisherNode(Node):
    def __init__(self):
        super().__init__('wit_imu_publisher')
        
        # 声明参数，方便后期通过 launch 或命令行修改
        self.declare_parameter('baud', 230400)
        
        # 针对 4 个不同部位创建 4 个发布者，分别发布角度(ang)和加速度(acc)
        self.pubs = {
            "大臂": {
                "ang_x": self.create_publisher(Float32MultiArray, 'imu/boom_ang_x', 10),
                "ang_y": self.create_publisher(Float32MultiArray, 'imu/boom_ang_y', 10),
                "acc_x": self.create_publisher(Float32MultiArray, 'imu/boom_acc_x', 10),
                "acc_y": self.create_publisher(Float32MultiArray, 'imu/boom_acc_y', 10)
            },
            "小臂": {
                "ang_x": self.create_publisher(Float32MultiArray, 'imu/arm_ang_x', 10),
                "ang_y": self.create_publisher(Float32MultiArray, 'imu/arm_ang_y', 10),
                "acc_x": self.create_publisher(Float32MultiArray, 'imu/arm_acc_x', 10),
                "acc_y": self.create_publisher(Float32MultiArray, 'imu/arm_acc_y', 10)
            },
            "铲斗": {
                "ang_x": self.create_publisher(Float32MultiArray, 'imu/bucket_ang_x', 10),
                "ang_y": self.create_publisher(Float32MultiArray, 'imu/bucket_ang_y', 10),
                "acc_x": self.create_publisher(Float32MultiArray, 'imu/bucket_acc_x', 10),
                "acc_y": self.create_publisher(Float32MultiArray, 'imu/bucket_acc_y', 10)
            },
            "回转": {
                "ang_x": self.create_publisher(Float32MultiArray, 'imu/swing_ang_x', 10),
                "ang_y": self.create_publisher(Float32MultiArray, 'imu/swing_ang_y', 10),
                "acc_x": self.create_publisher(Float32MultiArray, 'imu/swing_acc_x', 10),
                "acc_y": self.create_publisher(Float32MultiArray, 'imu/swing_acc_y', 10)
            }
        }
        
        # 新增一个 Topic，专门用于发布相对位置的大臂、小臂和铲斗的 Ang_x 数据
        # 数组内容约定为: [boom_rel_ang_x, arm_rel_ang_x, bucket_rel_ang_x]
        self.pub_rel_ang_x = self.create_publisher(Float32MultiArray, 'imu/relative_ang_x', 10)

        baud = self.get_parameter('baud').value
        # 需要轮询的传感器ID列表
        addrLis = [0x50, 0x51, 0x52, 0x53]
        
        # 移除 glob.glob 动态扫描，改用基于物理接口绑定的固定 udev 规则名称
        # 这样可以防止脚本误将轮询指令发送给挖掘机控制器，导致控制器崩溃
        ports = [
            "/dev/ttyUSB_Sensor1",
            "/dev/ttyUSB_Sensor2",
            "/dev/ttyUSB_Sensor3",
            "/dev/ttyUSB_Sensor4",
        ]
        
        self.devices = []
        
        for port in ports:
            try:
                # 传入的回调函数携带 port，以便在内部通过 id 识别
                device = device_model.DeviceModel(port, port, baud, addrLis, self.create_callback(port))
                device.openDevice()
                device.startLoopRead()
                self.devices.append(device)
                self.get_logger().info(f"[{port}] 初始化成功，正在探测传感器...")
            except Exception as e:
                self.get_logger().error(f"[{port}] 初始化失败: {e}")

    def create_callback(self, port_name):
        def updateData(DeviceModel):
            current_time = time.time()
            for addr, name in id_to_name.items():
                data = DeviceModel.deviceData.get(addr, {})
                # 确保需要的数据都已读到
                if data and "AccX" in data and "AngX" in data:
                    # 获取角度和加速度数据
                    ang_x = data.get("AngX", 0.0)
                    ang_y = data.get("AngY", 0.0)
                    acc_x = data.get("AccX", 0.0)
                    acc_y = data.get("AccY", 0.0)
                    
                    # 记录初始角度
                    if init_abs_ang_x[addr] is None:
                        init_abs_ang_x[addr] = ang_x
                    
                    # 更新最新绝对角度
                    latest_abs_ang_x[addr] = ang_x
                    
                    # 只有当所有传感器都获取到初始角度后，才开始计算和发布
                    if None not in init_abs_ang_x.values():
                        # 根据运动学关系，直接计算当前各部位之间的相对角度
                        # 0x50: 铲斗, 0x51: 小臂, 0x52: 大臂, 0x53: 回转
                        # 这样计算出来的初始值就是真实的相对夹角（例如：铲斗初始角度 - 小臂初始角度），而不是 0
                        rel_bucket_x = latest_abs_ang_x[0x50] - latest_abs_ang_x[0x51]
                        rel_arm_x = latest_abs_ang_x[0x51] - latest_abs_ang_x[0x52]
                        rel_boom_x = latest_abs_ang_x[0x52] - latest_abs_ang_x[0x53]
                            
                        # 避免时间错误/粘连数据：限制最高发布频率
                        interval = current_time - last_published_time[addr]
                        if interval > 0.01:
                            # 1. 保持之前的四类数据发布不变（发布原始的绝对角度和加速度）
                            msg_ang_x = Float32MultiArray()
                            msg_ang_x.data = [float(ang_x)]
                            self.pubs[name]["ang_x"].publish(msg_ang_x)
                            
                            msg_ang_y = Float32MultiArray()
                            msg_ang_y.data = [float(ang_y)]
                            self.pubs[name]["ang_y"].publish(msg_ang_y)
                            
                            # 发布加速度数据
                            msg_acc_x = Float32MultiArray()
                            msg_acc_x.data = [float(acc_x)]
                            self.pubs[name]["acc_x"].publish(msg_acc_x)
                            
                            msg_acc_y = Float32MultiArray()
                            msg_acc_y.data = [float(acc_y)]
                            self.pubs[name]["acc_y"].publish(msg_acc_y)
                            
                            # 2. 新增发布综合相对角度数据
                            # 为了避免4个传感器回调各发一次导致数据冗余，我们可以选择只在其中一个传感器（比如铲斗 0x50）的回调中触发这综合数据的发布
                            # 因为 latest_abs_ang_x 保存了所有的最新状态
                            if addr == 0x50:
                                msg_rel_ang = Float32MultiArray()
                                # 约定数据格式为 [大臂相对角, 小臂相对角, 铲斗相对角]
                                msg_rel_ang.data = [float(rel_boom_x), float(rel_arm_x), float(rel_bucket_x)]
                                self.pub_rel_ang_x.publish(msg_rel_ang)
                            
                            # 更新状态
                            last_published_time[addr] = current_time
                    
                    # 清除已读数据，确保下一次回调拿到的一定是最新读取进来的数据，而不是历史缓存
                    DeviceModel.deviceData[addr].clear()
        return updateData
        
    def close_devices(self):
        self.get_logger().info("正在停止传感器读取...")
        for device in self.devices:
            device.stopLoopRead()
            
        time.sleep(0.5)
        
        for device in self.devices:
            device.isOpen = False
            device.closeDevice()

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_devices()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()