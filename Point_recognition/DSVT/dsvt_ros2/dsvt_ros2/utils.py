"""
ROS2 / numpy 点云格式转换工具。

核心功能:
- sensor_msgs/PointCloud2 ←→ numpy 数组
- 检测结果 → visualization_msgs/MarkerArray (RViz2 直接显示)

不依赖 vision_msgs (可跳过 apt 安装)。
"""

import numpy as np

# ---- 默认颜色方案 ----
DEFAULT_COLORS = {
    0: (0.0, 0.0, 1.0),   # blue
    1: (0.0, 1.0, 0.0),   # green
    2: (0.0, 1.0, 1.0),   # cyan
    3: (1.0, 1.0, 0.0),   # yellow
    4: (1.0, 0.0, 1.0),   # magenta
    5: (1.0, 0.5, 0.0),   # orange
    6: (1.0, 0.0, 0.0),   # red
}


# ============================================================================
# PointCloud2 ←→ numpy
# ============================================================================

def pointcloud2_to_numpy(ros_msg):
    """将 ROS2 PointCloud2 消息转为 numpy 数组。

    Parameters
    ----------
    ros_msg : sensor_msgs.msg.PointCloud2

    Returns
    -------
    points : np.ndarray
        shape (N, 4+)  [x, y, z, intensity, ...]
    """
    field_names = [f.name for f in ros_msg.fields]
    field_offsets = {f.name: f.offset for f in ros_msg.fields}
    point_step = ros_msg.point_step
    raw = np.frombuffer(ros_msg.data, dtype=np.uint8)
    n_pts = len(raw) // point_step

    def _read_f32(name):
        """向量化读取 float32: 逐字节跨步提取, 再 view 为 float32"""
        off = field_offsets[name]
        result = np.zeros(n_pts, dtype=np.float32)
        vu8 = result.view(np.uint8)
        for b in range(4):
            vu8[b::4] = raw[off + b::point_step]
        return result

    x, y, z = _read_f32('x'), _read_f32('y'), _read_f32('z')
    fields = [x, y, z]

    if 'intensity' in field_names:
        fields.append(_read_f32('intensity'))
    elif 'i' in field_names:
        fields.append(_read_f32('i'))

    return np.column_stack(fields)


def numpy_to_pointcloud2(points, frame_id='lidar', stamp=None):
    """numpy 数组 → ROS2 PointCloud2。

    Parameters
    ----------
    points : np.ndarray
        shape (N, 3+), 至少 [x, y, z]
    frame_id : str
    stamp : builtin_interfaces.msg.Time, optional

    Returns
    -------
    sensor_msgs.msg.PointCloud2
    """
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header

    N = points.shape[0]
    C = min(points.shape[1], 4)

    field_names = ['x', 'y', 'z', 'intensity']
    fields = []
    for i in range(C):
        fields.append(PointField(
            name=field_names[i],
            offset=i * 4,
            datatype=PointField.FLOAT32,
            count=1,
        ))

    pts = np.zeros((N, 4), dtype=np.float32)
    pts[:, :C] = points[:, :C].astype(np.float32)

    header = Header(frame_id=frame_id)
    if stamp is not None:
        header.stamp = stamp

    return PointCloud2(
        header=header,
        height=1,
        width=N,
        fields=fields,
        is_bigendian=False,
        point_step=16,
        row_step=16 * N,
        data=pts.tobytes(),
        is_dense=True,
    )


# ============================================================================
# 检测结果 → RViz2 MarkerArray
# ============================================================================

def boxes_to_marker_array(
    boxes,
    scores,
    labels,
    class_names=None,
    header=None,
    color_map=None,
    alpha=0.5,
    ns='detection',
    score_thresh=0.0,
):
    """将 3D 检测结果构造为 MarkerArray (RViz2 直接显示)。

    Parameters
    ----------
    boxes : np.ndarray
        shape (M, 7), [x, y, z, dx, dy, dz, heading]
    scores : np.ndarray
        shape (M,)
    labels : np.ndarray
        shape (M,), 整数类别 ID
    class_names : list[str], optional
    header : std_msgs.msg.Header, optional
        消息头 (包含 frame_id 和 stamp)
    color_map : dict, optional
        {label_id: (r, g, b)}, 0-1 范围
    alpha : float
        包围盒透明度
    ns : str
        Marker 命名空间
    score_thresh : float
        额外的置信度阈值

    Returns
    -------
    visualization_msgs.msg.MarkerArray
    """
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray

    if color_map is None:
        color_map = DEFAULT_COLORS

    if header is None:
        from std_msgs.msg import Header
        header = Header(frame_id='lidar')

    markers = MarkerArray()

    # 首先清除上一次的所有 Marker (DELETEALL)
    clear = Marker()
    clear.header = header
    clear.action = Marker.DELETEALL
    clear.ns = ns
    clear.id = 0
    markers.markers.append(clear)

    for i in range(len(boxes)):
        if scores[i] < score_thresh:
            continue

        x, y, z, dx, dy, dz, heading = boxes[i]
        label_id = int(labels[i])
        r, g, b = color_map.get(label_id, DEFAULT_COLORS.get(label_id, (1.0, 1.0, 1.0)))

        marker_id = i + 1  # 0 已被 DELETEALL 使用

        # ---- 3D 包围盒 ----
        box_marker = Marker()
        box_marker.header = header
        box_marker.ns = ns
        box_marker.id = marker_id
        box_marker.type = Marker.CUBE
        box_marker.action = Marker.ADD

        box_marker.pose.position.x = float(x)
        box_marker.pose.position.y = float(y)
        box_marker.pose.position.z = float(z)
        box_marker.pose.orientation.z = float(np.sin(heading / 2.0))
        box_marker.pose.orientation.w = float(np.cos(heading / 2.0))

        box_marker.scale.x = float(max(dx, 0.1))
        box_marker.scale.y = float(max(dy, 0.1))
        box_marker.scale.z = float(max(dz, 0.1))

        box_marker.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(alpha))

        # 生命周期: 500ms
        box_marker.lifetime.sec = 0
        box_marker.lifetime.nanosec = 500_000_000

        markers.markers.append(box_marker)

        # ---- 文字标签 (类别 + 置信度) ----
        text = Marker()
        text.header = header
        text.ns = ns + '_labels'
        text.id = marker_id + 50000  # 避免 ID 冲突
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD

        text.pose.position.x = float(x)
        text.pose.position.y = float(y)
        text.pose.position.z = float(z + dz / 2.0 + 0.3)

        cls_name = class_names[label_id] if class_names and label_id < len(class_names) else f'cls_{label_id}'
        text.text = f'{cls_name} {scores[i]:.2f}'
        text.scale.z = 0.5  # 字体高度

        text.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)
        text.lifetime.sec = 0
        text.lifetime.nanosec = 200_000_000

        markers.markers.append(text)

    return markers
