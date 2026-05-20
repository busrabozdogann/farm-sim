# visualizer_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import json
import math
import sys

try:
    import pygame
except ImportError:
    print("Install pygame: pip install pygame")
    sys.exit(1)

# ── Display ───────────────────────────────────────────────────────────────────
WIN_W, WIN_H = 760, 820
PAD          = 40
GRID_COLS    = 8
GRID_ROWS    = 8
TILE_PX      = (WIN_W - 2 * PAD) // GRID_COLS   # pixels per tile

PANEL_H      = WIN_H - WIN_W + PAD   # bottom panel for status text

# ── Colours ───────────────────────────────────────────────────────────────────
BG        = ( 20,  20,  20)
GRID_LINE = ( 50,  50,  50)
ROBOT_C   = ( 50, 140, 255)
ROBOT_B   = (150, 210, 255)
TEXT_C    = (220, 220, 220)
DIM_C     = (130, 130, 130)


def wetness_color(w, threshold=30.0):
    """
    Maps wetness 0-100 to a color:
      0   → dusty brown  (180, 120,  60)
      30  → yellow-brown (200, 180,  80)  ← dry threshold
      100 → dark green   ( 30, 120,  40)
    """
    t = max(0.0, min(100.0, w)) / 100.0
    if t < 0.3:
        # dry: brown → yellow-brown
        s = t / 0.3
        r = int(180 + s * 20)
        g = int(120 + s * 60)
        b = int(60  + s * 20)
    else:
        # wet: yellow-brown → dark green
        s = (t - 0.3) / 0.7
        r = int(200 - s * 170)
        g = int(180 - s *  60)
        b = int(80  - s *  40)
    return (r, g, b)


def tile_rect(col, row):
    x = PAD + col * TILE_PX
    y = PAD + (GRID_ROWS - 1 - row) * TILE_PX   # y-flip
    return pygame.Rect(x, y, TILE_PX, TILE_PX)


def world_to_screen(wx, wy):
    sx = int(PAD + wx * TILE_PX)
    sy = int(PAD + (GRID_ROWS - wy) * TILE_PX)
    return sx, sy


class VisualizerNode(Node):
    def __init__(self, screen, font, sfont):
        super().__init__('visualizer_node')
        self.screen = screen
        self.font   = font
        self.sfont  = sfont

        self.wetness       = None
        self.dry_threshold = 30.0
        self.robot_pose    = None
        self.robot_state   = {}

        # Watering animation
        self.water_anim    = []   # list of (x, y, radius, alpha)

        self.create_subscription(String, '/world/grid',   self.grid_cb,   10)
        self.create_subscription(Pose2D, '/robot/pose',   self.robot_cb,  10)
        self.create_subscription(String, '/robot/state',  self.state_cb,  10)
        self.create_subscription(Pose2D, '/watering/event', self.water_event_cb, 10)

        self.create_timer(0.033, self.draw)
        self.get_logger().info("VisualizerNode ready.")

    def grid_cb(self, msg):
        data = json.loads(msg.data)
        self.wetness       = data['wetness']
        self.dry_threshold = data['dry_threshold']

    def robot_cb(self, msg):
        self.robot_pose = msg

    def state_cb(self, msg):
        self.robot_state = json.loads(msg.data)

    def water_event_cb(self, msg):
        sx, sy = world_to_screen(msg.x, msg.y)
        self.water_anim.append([sx, sy, 5, 200])

    # ── Draw ──────────────────────────────────────────────────────────────────
    def draw(self):
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit(); rclpy.shutdown(); sys.exit(0)
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                pygame.quit(); rclpy.shutdown(); sys.exit(0)

        self.screen.fill(BG)

        # ── Tiles ─────────────────────────────────────────────────────────────
        if self.wetness:
            for row in range(GRID_ROWS):
                for col in range(GRID_COLS):
                    w   = self.wetness[row][col]
                    col_val = wetness_color(w, self.dry_threshold)
                    rect = tile_rect(col, row)
                    pygame.draw.rect(self.screen, col_val, rect)

                    # Dry indicator — small orange dot
                    if w < self.dry_threshold:
                        cx = rect.centerx
                        cy = rect.centery
                        pygame.draw.circle(self.screen, (255, 160, 30),
                                           (cx, cy), 5)

                    # Grid lines
                    pygame.draw.rect(self.screen, GRID_LINE, rect, 1)

                    # Wetness number (small)
                    txt = self.sfont.render(f"{int(w)}", True,
                                            (0, 0, 0) if w > 40 else (220, 220, 180))
                    self.screen.blit(txt, (rect.x + 3, rect.y + 3))

        # ── Watering animations ───────────────────────────────────────────────
        for anim in self.water_anim[:]:
            sx, sy, r, alpha = anim
            if r < TILE_PX * 1.5 and alpha > 0:
                surf = pygame.Surface((r*2+2, r*2+2), pygame.SRCALPHA)
                pygame.draw.circle(surf, (80, 180, 255, int(alpha)),
                                   (r+1, r+1), r, 3)
                self.screen.blit(surf, (sx - r - 1, sy - r - 1))
                anim[2] += 3
                anim[3] -= 12
            else:
                self.water_anim.remove(anim)

        # ── Robot ─────────────────────────────────────────────────────────────
        if self.robot_pose:
            rx, ry = world_to_screen(self.robot_pose.x, self.robot_pose.y)
            radius = TILE_PX // 3

            # Watering glow
            state = self.robot_state.get('state', '')
            if state == 'watering':
                t = self.robot_state.get('water_timer', 0)
                pulse = int(40 + 20 * math.sin(pygame.time.get_ticks() * 0.01))
                pygame.draw.circle(self.screen, (80, 180, 255),
                                   (rx, ry), radius + pulse // 3, 3)

            pygame.draw.circle(self.screen, ROBOT_C, (rx, ry), radius)
            pygame.draw.circle(self.screen, ROBOT_B, (rx, ry), radius, 2)

            # Direction arrow
            d = self.robot_pose.theta
            ax = int(rx + d * radius * 0.8)
            pygame.draw.line(self.screen, (255, 230, 50),
                             (rx, ry), (ax, ry), 3)

        # ── Status panel ──────────────────────────────────────────────────────
        panel_y = PAD + GRID_ROWS * TILE_PX + 8
        state   = self.robot_state.get('state', '—')
        row     = self.robot_state.get('patrol_row', 0)
        timer   = self.robot_state.get('water_timer', 0.0)

        state_color = {
            'patrol':   (100, 200, 100),
            'navigate': (255, 200,  50),
            'watering': ( 80, 180, 255),
            'return':   (200, 120, 255),
        }.get(state, TEXT_C)

        title = self.font.render("Farm Irrigation Robot", True, (255,255,255))
        self.screen.blit(title, (PAD, panel_y))

        info = f"State: {state.upper()}   Row: {row}   " + \
               (f"Watering: {timer:.1f}s" if state == 'watering' else "")
        surf = self.sfont.render(info, True, state_color)
        self.screen.blit(surf, (PAD, panel_y + 22))

        # Legend
        legend = [
            ((180, 120, 60),  "Dry"),
            ((200, 180, 80),  "Low"),
            (( 30, 120, 40),  "Wet"),
            ((255, 160, 30),  "Needs water"),
            (ROBOT_C,         "Robot"),
        ]
        lx = WIN_W - 160
        for col_v, label in legend:
            pygame.draw.circle(self.screen, col_v, (lx, panel_y + 8), 6)
            s = self.sfont.render(label, True, DIM_C)
            self.screen.blit(s, (lx + 12, panel_y))
            panel_y += 18

        pygame.display.flip()


def main(args=None):
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Farm Irrigation Robot")
    font  = pygame.font.SysFont("monospace", 15, bold=True)
    sfont = pygame.font.SysFont("monospace", 12)

    rclpy.init(args=args)
    node = VisualizerNode(screen, font, sfont)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()


if __name__ == '__main__':
    main()