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

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("mask_topic", "/keepout_filter_mask")
        self.declare_parameter("filter_info_topic", "/costmap_filter_info")
        self.declare_parameter("inside_value", 100)
        self.declare_parameter("outside_value", 0)

        # ★ 多禁行区：polygons 列表 + polygon_<name> 参数
        self.declare_parameter("polygons", [""])
        # 兼容旧格式：单区
        self.declare_parameter("polygon_points", "[]")

        self.map_topic = (
            self.get_parameter("map_topic").get_parameter_value().string_value
        )
        self.mask_topic = (
            self.get_parameter("mask_topic").get_parameter_value().string_value
        )
        self.filter_info_topic = (
            self.get_parameter("filter_info_topic")
            .get_parameter_value().string_value
        )

        # ★ 解析所有多边形
        self.polygons = self._load_polygons()
        if not self.polygons:
            raise RuntimeError(
                "未找到有效禁行区，请配置 polygons + polygon_<name> "
                "或 polygon_points"
            )

        self.inside_value = int(
            self.get_parameter("inside_value").value
        )
        self.outside_value = int(
            self.get_parameter("outside_value").value
        )

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.mask_pub = self.create_publisher(
            OccupancyGrid, self.mask_topic, latched_qos)
        self.info_pub = self.create_publisher(
            CostmapFilterInfo, self.filter_info_topic, latched_qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, latched_qos)

        names = [name for name, _ in self.polygons]
        self.get_logger().info(
            f"等待地图 {self.map_topic}；"
            f"已加载 {len(self.polygons)} 个禁行区: {names}"
        )
        for name, poly in self.polygons:
            self.get_logger().info(f"  [{name}] {len(poly)} 个顶点")

    # ================================================================
    def _load_polygons(self):
        """优先读 polygons + polygon_<name>，回退到 polygon_points。"""
        polygons = self._load_polygons_from_names()
        if not polygons:
            polygons = self._load_polygon_legacy()
        return polygons

    def _load_polygons_from_names(self):
        result = []
        try:
            names = list(self.get_parameter("polygons").value)
        except Exception:
            return result

        names = [str(n).strip() for n in names if str(n).strip()]

        for name in names:
            param_name = f"polygon_{name}"
            if not self.has_parameter(param_name):
                try:
                    self.declare_parameter(param_name, "[]")
                except Exception:
                    pass
                if not self.has_parameter(param_name):
                    self.get_logger().warn(
                        f"未找到参数 {param_name}，跳过 [{name}]")
                    continue

            polygon_text = (
                self.get_parameter(param_name)
                .get_parameter_value().string_value
            )

            try:
                raw = json.loads(polygon_text)
                poly = [
                    (float(p[0]), float(p[1])) for p in raw
                ]
            except (json.JSONDecodeError, TypeError,
                    ValueError, IndexError) as exc:
                self.get_logger().warn(
                    f"[{name}] 格式错误: {polygon_text}；"
                    f"错误: {exc}；跳过")
                continue

            if len(poly) < 3:
                self.get_logger().warn(
                    f"[{name}] 少于 3 个顶点，跳过")
                continue

            result.append((name, poly))

        return result

    def _load_polygon_legacy(self):
        try:
            polygon_text = (
                self.get_parameter("polygon_points")
                .get_parameter_value().string_value
            )
        except Exception:
            return []

        try:
            raw = json.loads(polygon_text)
            poly = [(float(p[0]), float(p[1])) for p in raw]
        except (json.JSONDecodeError, TypeError,
                ValueError, IndexError):
            return []

        if len(poly) < 3:
            return []

        return [("default", poly)]

    # ================================================================
    @staticmethod
    def point_in_polygon(x, y, polygon):
        inside = False
        j = len(polygon) - 1
        for i in range(len(polygon)):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            intersects = (
                (yi > y) != (yj > y)
                and x < (xj - xi) * (y - yi) / (yj - yi) + xi
            )
            if intersects:
                inside = not inside
            j = i
        return inside

    def map_callback(self, source_map):
        width = source_map.info.width
        height = source_map.info.height
        resolution = source_map.info.resolution
        origin_x = source_map.info.origin.position.x
        origin_y = source_map.info.origin.position.y

        data = [self.outside_value] * (width * height)
        marked_cells = 0

        for grid_y in range(height):
            world_y = origin_y + (grid_y + 0.5) * resolution
            for grid_x in range(width):
                world_x = origin_x + (grid_x + 0.5) * resolution
                for _, poly in self.polygons:
                    if self.point_in_polygon(world_x, world_y, poly):
                        index = grid_y * width + grid_x
                        data[index] = self.inside_value
                        marked_cells += 1
                        break

        mask = OccupancyGrid()
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.header.frame_id = source_map.header.frame_id or "map"
        mask.info = source_map.info
        mask.data = data

        filter_info = CostmapFilterInfo()
        filter_info.header.stamp = self.get_clock().now().to_msg()
        filter_info.header.frame_id = mask.header.frame_id
        filter_info.type = 0
        filter_info.filter_mask_topic = self.mask_topic
        filter_info.base = 0.0
        filter_info.multiplier = 1.0

        self.mask_pub.publish(mask)
        self.info_pub.publish(filter_info)

        self.get_logger().info(
            f"禁区掩码已发布: {width}x{height}, "
            f"禁区栅格数={marked_cells}, "
            f"多边形数={len(self.polygons)}")


def main(args=None):
    rclpy.init(args=args)
    node = PolygonKeepoutMaskServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
