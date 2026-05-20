# rviz_publisher_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Pose2D, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
import json
import math

# ── Constants ─────────────────────────────────────────────────────────────────
GRID_COLS   = 8
GRID_ROWS   = 8
TILE_SIZE   = 1.0
TILE_HEIGHT = 0.05   # flat tiles
ROBOT_H     = 0.3
FRAME_ID    = "map"


def rgba(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def point(x, y, z=0.0):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def vec3(x, y, z):
    v = Vector3()
    v.x, v.y, v.z = float(x), float(y), float(z)
    return v


def base_marker(ns, uid, mtype):
    m = Marker()
    m.header.frame_id    = FRAME_ID
    m.ns                 = ns
    m.id                 = uid
    m.type               = mtype
    m.action             = Marker.ADD
    m.pose.orientation.w = 1.0
    return m


def wetness_to_rgba(w):
    """Same color mapping as pygame visualizer."""
    t = max(0.0, min(100.0, w)) / 100.0
    if t < 0.3:
        s = t / 0.3
        r = (180 + s * 20) / 255
        g = (120 + s * 60) / 255
        b = (60  + s * 20) / 255
    else:
        s = (t - 0.3) / 0.7
        r = (200 - s * 170) / 255
        g = (180 - s *  60) / 255
        b = (80  - s *  40) / 255
    return rgba(r, g, b, 1.0)


class RvizPublisherNode(Node):
    def __init__(self):
        super().__init__('rviz_publisher_node')

        self.wetness       = None
        self.dry_threshold = 30.0
        self.robot_pose    = None
        self.robot_state   = {}
        self.watering_pos  = None
        self.water_anim_t  = 0.0

        self.create_subscription(String, '/world/grid',      self.grid_cb,    10)
        self.create_subscription(Pose2D, '/robot/pose',      self.robot_cb,   10)
        self.create_subscription(String, '/robot/state',     self.state_cb,   10)
        self.create_subscription(Pose2D, '/watering/event',  self.water_cb,   10)

        self.pub_tiles    = self.create_publisher(MarkerArray, '/viz/tiles',    1)
        self.pub_robot    = self.create_publisher(MarkerArray, '/viz/robot',   10)
        self.pub_watering = self.create_publisher(MarkerArray, '/viz/watering', 10)

        self.create_timer(0.05, self.publish_all)
        self.get_logger().info("RvizPublisherNode ready.")

    def grid_cb(self, msg):
        data = json.loads(msg.data)
        self.wetness       = data['wetness']
        self.dry_threshold = data['dry_threshold']

    def robot_cb(self, msg):
        self.robot_pose = msg

    def state_cb(self, msg):
        self.robot_state = json.loads(msg.data)

    def water_cb(self, msg):
        self.watering_pos = (msg.x, msg.y)
        self.water_anim_t = 1.0

    def publish_all(self):
        now = self.get_clock().now().to_msg()
        self._publish_tiles(now)
        self._publish_robot(now)
        self._publish_watering(now)

    # ── Tiles ─────────────────────────────────────────────────────────────────
    def _publish_tiles(self, now):
        if self.wetness is None:
            return

        markers = []
        for row in range(GRID_ROWS):
            for col in range(GRID_COLS):
                uid = row * GRID_COLS + col
                w   = self.wetness[row][col]

                # Tile cube — height varies slightly with wetness for 3D effect
                tile_h = TILE_HEIGHT + (w / 100.0) * 0.08
                m = base_marker("tiles", uid, Marker.CUBE)
                m.header.stamp  = now
                m.pose.position = point(
                    col * TILE_SIZE + TILE_SIZE / 2,
                    row * TILE_SIZE + TILE_SIZE / 2,
                    tile_h / 2
                )
                m.scale         = vec3(TILE_SIZE - 0.04,
                                       TILE_SIZE - 0.04,
                                       tile_h)
                m.color         = wetness_to_rgba(w)
                markers.append(m)

                # Dry indicator — always publish for every tile using same uid
                # If wet: scale to 0 (invisible). If dry: visible orange sphere.
                # This way the marker always exists and updates in place — no
                # leftover spheres when a tile gets watered.
                s = base_marker("dry_indicators", uid, Marker.SPHERE)
                s.header.stamp  = now
                s.pose.position = point(
                    col * TILE_SIZE + TILE_SIZE / 2,
                    row * TILE_SIZE + TILE_SIZE / 2,
                    0.25
                )
                if w < self.dry_threshold:
                    s.scale = vec3(0.2, 0.2, 0.2)
                    s.color = rgba(1.0, 0.63, 0.12, 1.0)
                else:
                    # Scale to zero = invisible, but marker still exists
                    s.scale = vec3(0.001, 0.001, 0.001)
                    s.color = rgba(0.0, 0.0, 0.0, 0.0)
                markers.append(s)

        arr = MarkerArray()
        arr.markers = markers
        self.pub_tiles.publish(arr)

    # ── Robot ─────────────────────────────────────────────────────────────────
    def _publish_robot(self, now):
        if self.robot_pose is None:
            return

        markers = []
        rx, ry  = self.robot_pose.x, self.robot_pose.y
        d       = self.robot_pose.theta
        state   = self.robot_state.get('state', '')

        # Body
        body = base_marker("robot_body", 0, Marker.CYLINDER)
        body.header.stamp  = now
        body.pose.position = point(rx, ry, ROBOT_H / 2)
        body.scale         = vec3(0.35, 0.35, ROBOT_H)
        body.color         = rgba(0.15, 0.55, 1.0, 1.0)
        markers.append(body)

        # Dome
        dome = base_marker("robot_dome", 1, Marker.SPHERE)
        dome.header.stamp  = now
        dome.pose.position = point(rx, ry, ROBOT_H + 0.1)
        dome.scale         = vec3(0.28, 0.28, 0.2)
        dome.color         = rgba(0.4, 0.78, 1.0, 1.0)
        markers.append(dome)

        # Direction arrow
        arrow = base_marker("robot_arrow", 2, Marker.ARROW)
        arrow.header.stamp = now
        arrow.points = [
            point(rx, ry, ROBOT_H + 0.05),
            point(rx + d * 0.4, ry, ROBOT_H + 0.05),
        ]
        arrow.scale = vec3(0.06, 0.12, 0.1)
        arrow.color = rgba(1.0, 1.0, 0.2, 1.0)
        markers.append(arrow)

        # Watering glow ring when in watering state
        if state == 'watering':
            ring = base_marker("robot_ring", 3, Marker.CYLINDER)
            ring.header.stamp  = now
            ring.pose.position = point(rx, ry, 0.02)
            ring.scale         = vec3(0.9, 0.9, 0.04)
            ring.color         = rgba(0.3, 0.7, 1.0, 0.7)
            markers.append(ring)

        arr = MarkerArray()
        arr.markers = markers
        self.pub_robot.publish(arr)

    # ── Watering effect ───────────────────────────────────────────────────────
    def _publish_watering(self, now):
        markers = []

        if self.watering_pos and self.water_anim_t > 0:
            wx, wy = self.watering_pos
            alpha  = self.water_anim_t
            radius = (1.0 - self.water_anim_t) * TILE_SIZE * 1.2 + 0.1

            ring = base_marker("water_ring", 0, Marker.CYLINDER)
            ring.header.stamp  = now
            ring.pose.position = point(wx, wy, 0.03)
            ring.scale         = vec3(radius * 2, radius * 2, 0.02)
            ring.color         = rgba(0.2, 0.6, 1.0, alpha * 0.7)
            markers.append(ring)

            self.water_anim_t = max(0.0, self.water_anim_t - 0.05)

        # Always publish (even empty) to clear old markers
        arr = MarkerArray()
        arr.markers = markers if markers else []
        self.pub_watering.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = RvizPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()