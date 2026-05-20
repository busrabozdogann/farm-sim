# environment_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import json
import random

# ── Grid config ───────────────────────────────────────────────────────────────
GRID_COLS  = 8
GRID_ROWS  = 8
TILE_SIZE  = 1.0

# Wetness
WETNESS_MAX       = 100.0
WETNESS_START_MIN = 60.0
WETNESS_START_MAX = 100.0
WETNESS_DRY_THRESHOLD = 30.0   # below this → tile needs watering
WATER_REFILL      = 100.0      # wetness set to this after watering

# Each tile depletes at a random rate (units per second)
DEPLETE_RATE_MIN = 0.1
DEPLETE_RATE_MAX = 0.3

UPDATE_RATE = 0.1   # seconds between wetness updates (10 Hz)


def tile_centre(col, row):
    return (col * TILE_SIZE + TILE_SIZE / 2,
            row * TILE_SIZE + TILE_SIZE / 2)


class EnvironmentNode(Node):
    def __init__(self):
        super().__init__('environment_node')

        # Grid state: wetness[row][col]
        self.wetness = [
            [random.uniform(WETNESS_START_MIN, WETNESS_START_MAX)
             for _ in range(GRID_COLS)]
            for _ in range(GRID_ROWS)
        ]

        # Depletion rates per tile
        self.deplete_rates = [
            [random.uniform(DEPLETE_RATE_MIN, DEPLETE_RATE_MAX)
             for _ in range(GRID_COLS)]
            for _ in range(GRID_ROWS)
        ]

        # Publishers
        self.grid_pub   = self.create_publisher(String, '/world/grid',   10)
        self.config_pub = self.create_publisher(String, '/world/config',  1)

        # Subscribers
        self.create_subscription(Pose2D, '/watering/event', self.watering_cb, 10)

        # Timers
        self.create_timer(UPDATE_RATE, self.update_wetness)
        self.create_timer(0.1,         self.publish_grid)
        self.create_timer(1.0,         self.publish_config)   # resend config periodically

        self.get_logger().info("EnvironmentNode ready.")
        self.get_logger().info(f"Grid: {GRID_COLS}x{GRID_ROWS}, tile size: {TILE_SIZE}")

    # ── Wetness update ────────────────────────────────────────────────────────
    def update_wetness(self):
        for row in range(GRID_ROWS):
            for col in range(GRID_COLS):
                self.wetness[row][col] = max(
                    0.0,
                    self.wetness[row][col] - self.deplete_rates[row][col] * UPDATE_RATE
                )

    # ── Watering callback ─────────────────────────────────────────────────────
    def watering_cb(self, msg):
        col = int(msg.x / TILE_SIZE)
        row = int(msg.y / TILE_SIZE)
        col = max(0, min(GRID_COLS - 1, col))
        row = max(0, min(GRID_ROWS - 1, row))
        self.wetness[row][col] = WATER_REFILL
        self.get_logger().info(f"Tile ({col},{row}) watered → {WATER_REFILL}")

    # ── Publish ───────────────────────────────────────────────────────────────
    def publish_grid(self):
        msg = String()
        msg.data = json.dumps({
            "wetness":    self.wetness,
            "dry_threshold": WETNESS_DRY_THRESHOLD,
        })
        self.grid_pub.publish(msg)

    def publish_config(self):
        msg = String()
        msg.data = json.dumps({
            "cols":      GRID_COLS,
            "rows":      GRID_ROWS,
            "tile_size": TILE_SIZE,
            "dry_threshold": WETNESS_DRY_THRESHOLD,
            "wetness_max":   WETNESS_MAX,
        })
        self.config_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()