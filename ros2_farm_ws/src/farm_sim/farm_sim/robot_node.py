# robot_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import json
import math

# ── Constants ─────────────────────────────────────────────────────────────────
GRID_COLS      = 8
GRID_ROWS      = 8
TILE_SIZE      = 1.0
ROBOT_SPEED    = 0.15
ARRIVE_DIST    = 0.05
WATER_DURATION = 1.5


def tile_centre(col, row):
    return (col * TILE_SIZE + TILE_SIZE / 2,
            row * TILE_SIZE + TILE_SIZE / 2)


def snake_order():
    """Returns all (col, row) in snake/lawnmower order."""
    tiles = []
    for row in range(GRID_ROWS):
        cols = range(GRID_COLS) if row % 2 == 0 else range(GRID_COLS - 1, -1, -1)
        for col in cols:
            tiles.append((col, row))
    return tiles


SNAKE = snake_order()   # pre-computed visit order


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        self.config        = None
        self.wetness       = None
        self.dry_threshold = 30.0

        # Start at first tile
        self.snake_idx = 0
        cx, cy = tile_centre(*SNAKE[0])
        self.x = cx
        self.y = cy

        # FSM
        self.state       = 'moving'
        self.water_timer = 0.0

        # Subscriptions
        self.create_subscription(String, '/world/config', self.config_cb,  1)
        self.create_subscription(String, '/world/grid',   self.grid_cb,   10)

        # Publishers
        self.pose_pub     = self.create_publisher(Pose2D, '/robot/pose',     10)
        self.watering_pub = self.create_publisher(Pose2D, '/watering/event', 10)
        self.state_pub    = self.create_publisher(String, '/robot/state',    10)

        self.create_timer(0.05, self.update)
        self.get_logger().info("RobotNode ready.")

    def config_cb(self, msg):
        self.config        = json.loads(msg.data)
        self.dry_threshold = self.config['dry_threshold']

    def grid_cb(self, msg):
        data               = json.loads(msg.data)
        self.wetness       = data['wetness']
        self.dry_threshold = data['dry_threshold']

    def current_tile(self):
        return SNAKE[self.snake_idx]

    def advance_tile(self):
        self.snake_idx = (self.snake_idx + 1) % len(SNAKE)

    def wetness_at(self, col, row):
        if self.wetness is None:
            return 100.0
        return self.wetness[row][col]

    def update(self):
        if self.wetness is None:
            return

        dt  = 0.05
        col, row = self.current_tile()
        tx, ty   = tile_centre(col, row)

        # ── MOVING ────────────────────────────────────────────────────────────
        if self.state == 'moving':
            dx   = tx - self.x
            dy   = ty - self.y
            dist = math.hypot(dx, dy)

            if dist < ARRIVE_DIST:
                # Snap to tile centre
                self.x = tx
                self.y = ty

                # Check if this tile needs watering
                if self.wetness_at(col, row) < self.dry_threshold:
                    self.water_timer = WATER_DURATION
                    self.state       = 'watering'
                    self.get_logger().info(
                        f"Tile ({col},{row}) dry, watering.")
                else:
                    # Move on to next tile immediately
                    self.advance_tile()

            else:
                step  = min(ROBOT_SPEED, dist)
                self.x += step * dx / dist
                self.y += step * dy / dist
                self.x  = clamp(self.x, 0.0, GRID_COLS * TILE_SIZE)
                self.y  = clamp(self.y, 0.0, GRID_ROWS * TILE_SIZE)

        # ── WATERING ──────────────────────────────────────────────────────────
        elif self.state == 'watering':
            self.water_timer -= dt
            if self.water_timer <= 0.0:
                # Notify environment
                msg = Pose2D()
                msg.x, msg.y, msg.theta = self.x, self.y, 0.0
                self.watering_pub.publish(msg)
                self.get_logger().info(
                    f"Watered ({col},{row}), continuing.")
                self.advance_tile()
                self.state = 'moving'

        # ── Publish ───────────────────────────────────────────────────────────
        pose       = Pose2D()
        pose.x     = self.x
        pose.y     = self.y
        pose.theta = 0.0
        self.pose_pub.publish(pose)

        state_msg      = String()
        state_msg.data = json.dumps({
            "state":       self.state,
            "water_timer": round(self.water_timer, 2),
            "patrol_row":  row,
            "patrol_col":  col,
            "tile_index":  self.snake_idx,
        })
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()