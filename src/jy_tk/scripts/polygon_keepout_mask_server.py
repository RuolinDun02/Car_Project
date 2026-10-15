#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo


class PolygonKeepoutMaskServer(Node):

    def __init__(self):
        super().__init__("polygon_keepout_mask_server")

        # 1. 声明地图、禁区 Mask、多边形坐标及栅格数值等参数
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("mask_topic", "/keepout_filter_mask")
        self.declare_parameter(
            "filter_info_topic", "/costmap_filter_info"
        )
        self.declare_parameter("polygon_points", "[]")
        self.declare_parameter("inside_value", 100)
        self.declare_parameter("outside_value", 0)

        # 2. 读取地图、Mask 和 FilterInfo 的 Topic 名称
        self.map_topic = (
            self.get_parameter("map_topic").get_parameter_value().string_value
        )
        self.mask_topic = (
            self.get_parameter("mask_topic").get_parameter_value().string_value
        )
        self.filter_info_topic = (
            self.get_parameter("filter_info_topic")
            .get_parameter_value()
            .string_value
        )

        # 3. 将 JSON 格式的多边形顶点解析为坐标列表
        polygon_text = (
            self.get_parameter("polygon_points")
            .get_parameter_value()
            .string_value
        )

        try:
            raw_polygon = json.loads(polygon_text)
            self.polygon = [
                (float(point[0]), float(point[1]))
                for point in raw_polygon
            ]
        except (json.JSONDecodeError, TypeError, ValueError, IndexError) as exc:
            raise RuntimeError(
                f"polygon_points 格式错误: {polygon_text}"
            ) from exc

        if len(self.polygon) < 3:
            raise RuntimeError("polygon_points 至少需要三个顶点")

        # 4. 读取禁区内部和外部的栅格数值
        self.inside_value = int(
            self.get_parameter("inside_value").value
        )
        self.outside_value = int(
            self.get_parameter("outside_value").value
        )

        # 5. 配置可靠传输及历史消息保留的 QoS
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 6. 创建禁区 Mask 发布器
        self.mask_pub = self.create_publisher(
            OccupancyGrid,
            self.mask_topic,
            latched_qos,
        )

        # 7. 创建 Nav2 Costmap Filter 元信息发布器
        self.info_pub = self.create_publisher(
            CostmapFilterInfo,
            self.filter_info_topic,
            latched_qos,
        )

        # 8. 订阅静态地图，并在收到地图时生成禁区 Mask
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            latched_qos,
        )

        self.get_logger().info(
            f"等待地图 {self.map_topic}，禁区顶点: {self.polygon}"
        )

    @staticmethod
    def point_in_polygon(x, y, polygon):
        """使用射线法判断指定坐标是否位于多边形内部。"""
        inside = False
        j = len(polygon) - 1

        for i in range(len(polygon)):
            xi, yi = polygon[i]
            xj, yj = polygon[j]

            intersects = (
                (yi > y) != (yj > y)
                and x
                < (xj - xi) * (y - yi) / (yj - yi) + xi
            )

            if intersects:
                inside = not inside

            j = i

        return inside

    def map_callback(self, source_map):

        # 9. 获取地图尺寸、分辨率和原点坐标
        width = source_map.info.width
        height = source_map.info.height
        resolution = source_map.info.resolution
        origin_x = source_map.info.origin.position.x
        origin_y = source_map.info.origin.position.y

        # 10. 初始化 Mask 栅格数据及禁区栅格计数
        data = [self.outside_value] * (width * height)
        marked_cells = 0

        # 11. 遍历地图栅格，将多边形内部区域标记为禁区
        for grid_y in range(height):
            world_y = origin_y + (grid_y + 0.5) * resolution

            for grid_x in range(width):
                world_x = origin_x + (grid_x + 0.5) * resolution

                if self.point_in_polygon(
                    world_x,
                    world_y,
                    self.polygon,
                ):
                    index = grid_y * width + grid_x
                    data[index] = self.inside_value
                    marked_cells += 1

        # 12. 构造与原始地图坐标系和分辨率一致的禁区 Mask
        mask = OccupancyGrid()
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.header.frame_id = source_map.header.frame_id or "map"
        mask.info = source_map.info
        mask.data = data

        # 13. 配置 KeepoutFilter 的类型、Mask Topic 和代价转换参数
        filter_info = CostmapFilterInfo()
        filter_info.header.stamp = self.get_clock().now().to_msg()
        filter_info.header.frame_id = mask.header.frame_id
        filter_info.type = 0
        filter_info.filter_mask_topic = self.mask_topic
        filter_info.base = 0.0
        filter_info.multiplier = 1.0

        # 14. 发布禁区 Mask 和 FilterInfo 供 Nav2 KeepoutFilter 使用
        self.mask_pub.publish(mask)
        self.info_pub.publish(filter_info)

        self.get_logger().info(
            f"禁区掩码已发布: {width}x{height}, "
            f"禁区栅格数={marked_cells}"
        )


def main(args=None):

    # 15. 初始化 ROS 2 节点并持续监听地图消息
    rclpy.init(args=args)
    node = PolygonKeepoutMaskServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 16. 退出时销毁节点并关闭 ROS 2
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
