"""
acc_3d.py - 3D Adaptive Cruise Control simulator in pygame.

A small perspective renderer (chase camera, projected road, 3D box-model cars)
written on top of pygame - no OpenGL, no CARLA, no extra libraries.

Controls (keyboard or on-screen buttons, hold to apply)
  UP    / ACCEL  button   increase speed     -> cruise control OFF
  DOWN  / DECEL  button   decrease speed     -> cruise control OFF
  SPACE / BRAKE  button   brake              -> cruise control OFF
  C     / CRUISE button   cruise ON at current speed (>= 30 km/h) / OFF
  R                       reset
  ESC                     quit

Run:  python acc_3d.py
"""
from __future__ import annotations

import math
import random
import sys

import pygame

from acc_model import (CAR_LEN_M, EGO_LANE, KMH, LANE_COUNT, LANE_WIDTH,
                       SENSOR_RANGE, V_MAX, Inputs, World, blink_on)

# --------------------------------------------------------------------------- #
# Screen / camera
# --------------------------------------------------------------------------- #
WIDTH, HEIGHT = 1200, 760
VIEW_H = 560                 # 3D view on top, dashboard below
FPS = 60
FOCAL = 820.0                # px, focal length of the virtual camera
CAM_BACK = 10.0              # m behind the ego car centre
CAM_HEIGHT = 3.2             # m above the road
PITCH = 0.07                 # rad, camera looks slightly down
CX, CY = WIDTH / 2, 300
HORIZON = CY - FOCAL * math.tan(PITCH)
DRAW_DIST = 230.0            # m
SEG = 4.0                    # m, road segment length
ROAD_HALF = LANE_COUNT * LANE_WIDTH / 2

# Colours
SKY_TOP, SKY_BOTTOM = (70, 130, 210), (190, 220, 245)
GRASS_A, GRASS_B = (60, 140, 60), (52, 125, 52)
ROAD_A, ROAD_B = (70, 72, 78), (64, 66, 72)
RUMBLE_A, RUMBLE_B = (210, 40, 40), (235, 235, 235)
LINE = (235, 235, 235)
PANEL_BG = (18, 20, 26)
TEXT = (232, 234, 240)
MUTED = (130, 136, 150)
GREEN = (50, 210, 100)
AMBER = (255, 176, 0)
RED = (235, 55, 55)
GLASS = (30, 40, 60)


def shade(color, f):
    return tuple(int(clamp255(c * f)) for c in color)


def clamp255(v):
    return max(0, min(255, v))


def lane_x(lane):
    return (lane - EGO_LANE) * LANE_WIDTH


# --------------------------------------------------------------------------- #
# Camera / projection
# --------------------------------------------------------------------------- #
class Camera:
    def __init__(self):
        self.x, self.y, self.z = 0.0, CAM_HEIGHT, 0.0
        self.sp, self.cp = math.sin(PITCH), math.cos(PITCH)

    def follow(self, ego):
        self.z = ego.pos - CAM_BACK
        self.x = lane_x(ego.lane)

    def depth(self, y, z):
        return (z - self.z) * self.cp - (y - self.y) * self.sp

    def project(self, x, y, z):
        dx, dy, dz = x - self.x, y - self.y, z - self.z
        zc = dz * self.cp - dy * self.sp
        if zc < 0.3:
            return None
        yc = dy * self.cp + dz * self.sp
        return (CX + FOCAL * dx / zc, CY - FOCAL * yc / zc)


# --------------------------------------------------------------------------- #
# Renderer
# --------------------------------------------------------------------------- #
class Renderer:
    def __init__(self, screen):
        self.s = screen
        self.cam = Camera()
        self.fx = pygame.Surface((WIDTH, VIEW_H), pygame.SRCALPHA)
        self.sky = self._make_sky()
        self.mountains = self._make_mountains()
        self.f_huge = pygame.font.SysFont("arial", 64, bold=True)
        self.f_big = pygame.font.SysFont("arial", 34, bold=True)
        self.f_med = pygame.font.SysFont("arial", 20, bold=True)
        self.f_small = pygame.font.SysFont("arial", 16)
        self.f_tiny = pygame.font.SysFont("arial", 12)
        self.buttons = {
            "ACCEL": pygame.Rect(840, 580, 165, 72),
            "CRUISE": pygame.Rect(1015, 580, 165, 72),
            "DECEL": pygame.Rect(840, 664, 165, 72),
            "BRAKE": pygame.Rect(1015, 664, 165, 72),
        }

    # ---- static backgrounds --------------------------------------------- #
    def _make_sky(self):
        surf = pygame.Surface((WIDTH, int(HORIZON) + 1))
        h = surf.get_height()
        for y in range(h):
            t = y / max(1, h - 1)
            col = tuple(int(a + (b - a) * t) for a, b in zip(SKY_TOP, SKY_BOTTOM))
            pygame.draw.line(surf, col, (0, y), (WIDTH, y))
        return surf

    def _make_mountains(self):
        rng = random.Random(3)
        surf = pygame.Surface((WIDTH, 120), pygame.SRCALPHA)
        for layer, (col, amp, step) in enumerate((((120, 140, 170), 95, 90), ((90, 115, 120), 60, 60))):
            pts = [(0, 120)]
            x = 0
            while x <= WIDTH + step:
                pts.append((x, 120 - rng.uniform(0.35, 1.0) * amp))
                x += step
            pts.append((WIDTH, 120))
            pygame.draw.polygon(surf, col, pts)
        return surf

    # ---- primitives ------------------------------------------------------ #
    def poly(self, surf, color, pts3d):
        pts = [self.cam.project(*p) for p in pts3d]
        if any(p is None for p in pts):
            return False
        pygame.draw.polygon(surf, color, pts)
        return True

    def ground_quad(self, color, x0, x1, z0, z1, y=0.0, surf=None):
        self.poly(surf or self.s, color, [(x0, y, z0), (x1, y, z0), (x1, y, z1), (x0, y, z1)])

    def box(self, x0, x1, y0, y1, z0, z1, color):
        """Draw the camera-visible faces of an axis-aligned box (z0 = rear)."""
        cam = self.cam
        if cam.x < x0:
            self.poly(self.s, shade(color, 0.62), [(x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0)])
        elif cam.x > x1:
            self.poly(self.s, shade(color, 0.62), [(x1, y0, z0), (x1, y0, z1), (x1, y1, z1), (x1, y1, z0)])
        if cam.y > y1:
            self.poly(self.s, shade(color, 1.12), [(x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)])
        if cam.z < z0:
            self.poly(self.s, shade(color, 0.85), [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)])

    def rear_quad(self, color, x0, x1, y0, y1, z):
        self.poly(self.s, color, [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)])

    def glow(self, x, y, z, radius_m, color):
        p = self.cam.project(x, y, z)
        if p is None:
            return
        zc = max(0.5, self.cam.depth(y, z))
        r = max(2, int(FOCAL * radius_m / zc))
        g = pygame.Surface((r * 2, r * 2), pygame.SRCALPHA)
        pygame.draw.circle(g, (*color, 90), (r, r), r)
        self.s.blit(g, (p[0] - r, p[1] - r))

    # ---- 3D car model ---------------------------------------------------- #
    def draw_car(self, car, t, braking):
        x = lane_x(car.lane)
        zr = car.pos - CAR_LEN_M / 2          # rear bumper
        zf = car.pos + CAR_LEN_M / 2
        hw = 0.9
        col = car.color
        # shadow
        self.ground_quad((30, 32, 36), x - hw - 0.15, x + hw + 0.15, zr - 0.2, zf + 0.1, 0.01)
        # wheels
        for wx in (x - hw + 0.02, x + hw - 0.3):
            for wz in (zr + 0.55, zf - 1.25):
                self.box(wx, wx + 0.28, 0.0, 0.62, wz, wz + 0.7, (20, 20, 22))
        # lower body, cabin
        self.box(x - hw, x + hw, 0.28, 1.0, zr, zf, col)
        cz0, cz1 = zr + 1.15, zr + 3.3
        self.box(x - 0.74, x + 0.74, 1.0, 1.52, cz0, cz1, shade(col, 0.95))
        # rear window, bumper, plate
        self.rear_quad(GLASS, x - 0.62, x + 0.62, 1.06, 1.46, cz0 - 0.01)
        self.rear_quad((40, 40, 44), x - hw, x + hw, 0.28, 0.42, zr - 0.01)
        self.rear_quad((240, 240, 240), x - 0.26, x + 0.26, 0.5, 0.66, zr - 0.02)
        # tail lights
        tail = (255, 40, 40) if braking else (130, 20, 22)
        for sx in (-1, 1):
            lx0, lx1 = (x + sx * 0.52, x + sx * 0.86)
            self.rear_quad(tail, min(lx0, lx1), max(lx0, lx1), 0.72, 0.9, zr - 0.02)
            if braking:
                self.glow(x + sx * 0.69, 0.81, zr - 0.05, 0.2, (255, 30, 30))
        # indicators (turn signal) - blink on the side of the lane change
        if car.indicator and blink_on(t):
            sx = car.indicator
            ix0, ix1 = x + sx * 0.66, x + sx * 0.9
            self.rear_quad(AMBER, min(ix0, ix1), max(ix0, ix1), 0.56, 0.7, zr - 0.03)
            self.glow(x + sx * 0.8, 0.63, zr - 0.05, 0.3, AMBER)
            # side repeater on the front fender
            sxp = x + sx * (hw + 0.01)
            self.poly(self.s, AMBER, [(sxp, 0.75, zf - 0.9), (sxp, 0.75, zf - 0.6),
                                      (sxp, 0.88, zf - 0.6), (sxp, 0.88, zf - 0.9)])

    # ---- tree ------------------------------------------------------------ #
    def draw_tree(self, x, z, size):
        pts = [self.cam.project(x, h * size, z) for h in (0, 6.5, 2.2, 1.8, 2.79)]
        if any(p is None for p in pts):
            return
        base, top, trunk_top, low1, low2 = pts
        zc = max(0.5, self.cam.depth(0, z))
        k = FOCAL / zc
        tw = max(1, 0.45 * k * size)
        pygame.draw.rect(self.s, (90, 60, 35), (base[0] - tw / 2, trunk_top[1], tw, base[1] - trunk_top[1]))
        w = 2.6 * k * size
        for i, (f, low) in enumerate(((0.0, low1), (0.33, low2))):
            y_low = low[1]
            pygame.draw.polygon(self.s, (30 + i * 12, 105 + i * 15, 45),
                                [(base[0] - w / 2 * (1 - f * 0.6), y_low),
                                 (base[0] + w / 2 * (1 - f * 0.6), y_low), (top[0], top[1])])

    # ---- world ----------------------------------------------------------- #
    def draw_world(self, world):
        cam = self.cam
        cam.follow(world.ego)
        s = self.s
        s.blit(self.sky, (0, 0))
        s.blit(self.mountains, (0, HORIZON - 119))
        pygame.draw.rect(s, GRASS_A, (0, HORIZON, WIDTH, VIEW_H - HORIZON))

        first = math.floor((cam.z + 1.0) / SEG)
        n = int(DRAW_DIST / SEG)
        for i in range(first + n, first - 1, -1):          # far -> near
            z0, z1 = i * SEG, (i + 1) * SEG
            z0 = max(z0, cam.z + 1.0)
            if z1 <= z0:
                continue
            even = (i // 2) % 2 == 0
            self.ground_quad(GRASS_A if even else GRASS_B, -300, 300, z0, z1)
            self.ground_quad(ROAD_A if even else ROAD_B, -ROAD_HALF, ROAD_HALF, z0, z1)
            rc = RUMBLE_A if i % 2 == 0 else RUMBLE_B
            self.ground_quad(rc, -ROAD_HALF - 0.6, -ROAD_HALF, z0, z1)
            self.ground_quad(rc, ROAD_HALF, ROAD_HALF + 0.6, z0, z1)
            self.ground_quad(LINE, -ROAD_HALF + 0.15, -ROAD_HALF + 0.3, z0, z1)
            self.ground_quad(LINE, ROAD_HALF - 0.3, ROAD_HALF - 0.15, z0, z1)
            if i % 3 == 0:                                   # dashed lane lines
                for k in range(1, LANE_COUNT):
                    bx = -ROAD_HALF + k * LANE_WIDTH
                    self.ground_quad(LINE, bx - 0.08, bx + 0.08, z0, z1)

        self.draw_radar(world)

        # objects sorted far -> near (painter's algorithm)
        objects = []
        spacing = 18.0
        for j in range(math.floor(cam.z / spacing) + 1, math.floor((cam.z + DRAW_DIST) / spacing) + 1):
            h = (j * 2654435761) & 0xFFFF
            for side in (-1, 1):
                off = 5 + (h % 9) + (3 if side > 0 else 0)
                z = j * spacing + (h % 7)
                size = 0.8 + (h % 5) * 0.1
                objects.append((z, "tree", (side * (ROAD_HALF + off), z, size)))
        for c in world.traffic:
            objects.append((c.pos - CAR_LEN_M / 2, "car", c))
        objects.append((world.ego.pos - CAR_LEN_M / 2, "ego", world.ego))
        objects.sort(key=lambda o: -o[0])

        for z, kind, obj in objects:
            if z - cam.z < (1.5 if kind == "tree" else 4.0):
                continue
            if kind == "tree":
                self.draw_tree(*obj)
            elif kind == "car":
                self.draw_car(obj, world.time, braking=obj.accel < -0.5)
            else:
                self.draw_car(obj, world.time, braking=world.a_cmd < -0.5 and obj.speed > 0.1)

        self.draw_target_label(world)

    def draw_radar(self, world):
        ego, t = world.ego, world.target
        rng = world.gap_to(t) if t is not None else SENSOR_RANGE
        z0 = ego.pos + CAR_LEN_M / 2
        z1 = z0 + rng
        x = lane_x(EGO_LANE)
        if t is None:
            col = (60, 220, 120, 55)
        elif world.fcw:
            col = (240, 50, 50, 95)
        else:
            col = (255, 176, 0, 75)
        self.fx.fill((0, 0, 0, 0))
        w1 = 0.4 + 1.2 * rng / SENSOR_RANGE
        self.poly(self.fx, col, [(x - 0.4, 0.02, z0), (x + 0.4, 0.02, z0),
                                 (x + w1, 0.02, z1), (x - w1, 0.02, z1)])
        self.s.blit(self.fx, (0, 0))

    def draw_target_label(self, world):
        t = world.target
        if t is None:
            return
        p = self.cam.project(lane_x(t.lane), 2.3, t.pos)
        if p is None:
            return
        label = self.f_med.render(f"{world.gap_to(t):.0f} m  |  {t.speed / KMH:.0f} km/h", True, TEXT)
        r = label.get_rect(midbottom=p).inflate(14, 6)
        bg = pygame.Surface(r.size, pygame.SRCALPHA)
        bg.fill((0, 0, 0, 150))
        self.s.blit(bg, r)
        pygame.draw.rect(self.s, RED if world.fcw else AMBER, r, 2, border_radius=4)
        self.s.blit(label, label.get_rect(center=r.center))

    # ---- HUD ------------------------------------------------------------- #
    def text(self, s, font, color, pos, anchor="topleft"):
        img = font.render(s, True, color)
        rect = img.get_rect(**{anchor: pos})
        self.s.blit(img, rect)
        return rect

    def draw_top_hud(self, world):
        on = world.cc.enabled
        box = pygame.Rect(16, 14, 330, 54)
        bg = pygame.Surface(box.size, pygame.SRCALPHA)
        bg.fill((0, 0, 0, 140))
        self.s.blit(bg, box)
        self.text(f"{world.ego.speed / KMH:5.0f} km/h", self.f_big, TEXT, (box.x + 12, box.y + 8))
        pill = pygame.Rect(box.x + 190, box.y + 12, 128, 30)
        pygame.draw.rect(self.s, GREEN if on else (70, 74, 84), pill, border_radius=15)
        self.text("CRUISE ON" if on else "CRUISE OFF", self.f_med, (10, 30, 15) if on else TEXT,
                  pill.center, "center")
        if world.cc.following and blink_on(world.time):
            self.text("VEHICLE AHEAD - SLOWING", self.f_med, AMBER, (box.x + 4, box.bottom + 8))
        if world.fcw and blink_on(world.time):
            self.text("BRAKE!", self.f_huge, RED, (CX, 120), "center")

    def speedometer(self, cx, cy, r, v, set_speed, enabled):
        pygame.draw.circle(self.s, (30, 33, 42), (cx, cy), r)
        pygame.draw.circle(self.s, (70, 75, 90), (cx, cy), r, 3)
        a0, a1 = math.radians(225), math.radians(-45)

        def ang(val):
            return a0 + (a1 - a0) * (val / V_MAX)

        for kmh in range(0, 181, 10):
            a = ang(kmh * KMH)
            major = kmh % 20 == 0
            r0 = r - (14 if major else 8)
            pygame.draw.line(self.s, TEXT if major else MUTED,
                             (cx + r0 * math.cos(a), cy - r0 * math.sin(a)),
                             (cx + (r - 4) * math.cos(a), cy - (r - 4) * math.sin(a)), 2 if major else 1)
            if major:
                self.text(str(kmh), self.f_tiny, MUTED,
                          (cx + (r - 26) * math.cos(a), cy - (r - 26) * math.sin(a)), "center")
        if set_speed is not None:
            a = ang(set_speed)
            pygame.draw.circle(self.s, GREEN if enabled else MUTED,
                               (int(cx + (r - 6) * math.cos(a)), int(cy - (r - 6) * math.sin(a))), 5)
        a = ang(v)
        pygame.draw.line(self.s, RED, (cx, cy), (cx + (r - 18) * math.cos(a), cy - (r - 18) * math.sin(a)), 4)
        pygame.draw.circle(self.s, (200, 200, 210), (cx, cy), 7)
        self.text(f"{v / KMH:.0f}", self.f_big, TEXT, (cx, cy + 38), "center")
        self.text("km/h", self.f_tiny, MUTED, (cx, cy + 62), "center")

    def lamp(self, x, y, label, on, color):
        pygame.draw.circle(self.s, color if on else (50, 54, 64), (x, y), 15)
        if on:
            g = pygame.Surface((64, 64), pygame.SRCALPHA)
            pygame.draw.circle(g, (*color, 80), (32, 32), 30)
            self.s.blit(g, (x - 32, y - 32))
        self.text(label, self.f_small, TEXT if on else MUTED, (x + 24, y), "midleft")

    def draw_dashboard(self, world, pressed):
        s, ego, cc = self.s, world.ego, world.cc
        pygame.draw.rect(s, PANEL_BG, (0, VIEW_H, WIDTH, HEIGHT - VIEW_H))
        pygame.draw.line(s, (60, 64, 76), (0, VIEW_H), (WIDTH, VIEW_H), 2)

        self.speedometer(110, VIEW_H + 100, 88, ego.speed, cc.set_speed, cc.enabled)

        # cruise state block
        badge = pygame.Rect(225, VIEW_H + 20, 260, 64)
        on = cc.enabled
        pygame.draw.rect(s, GREEN if on else (55, 59, 70), badge, border_radius=12)
        self.text("CRUISE CONTROL", self.f_small, (10, 40, 20) if on else MUTED, (badge.x + 14, badge.y + 7))
        self.text("ON" if on else "OFF", self.f_big, (10, 30, 15) if on else TEXT, (badge.x + 14, badge.y + 24))
        set_txt = f"SET {cc.set_speed / KMH:.0f} km/h" if cc.set_speed is not None else "SET ---"
        self.text(set_txt, self.f_med, (10, 30, 15) if on else MUTED, (badge.right - 12, badge.y + 34), "topright")

        mode_txt = {"OFF": "Normal driving (you control speed)",
                    "CRUISE": "Holding set speed",
                    "FOLLOWING": "Car ahead - keeping 1.8 s gap"}[world.mode]
        mode_col = {"OFF": MUTED, "CRUISE": GREEN, "FOLLOWING": AMBER}[world.mode]
        self.text(mode_txt, self.f_med, mode_col, (225, VIEW_H + 96))
        self.text(cc.last_event, self.f_small, TEXT, (225, VIEW_H + 126))
        t = world.target
        if t is not None:
            gap = world.gap_to(t)
            tg = f"{gap / ego.speed:.1f} s" if ego.speed > 0.5 else "--"
            info = f"Radar: car ahead {gap:.0f} m ({tg}), {t.speed / KMH:.0f} km/h"
        else:
            info = "Radar: lane ahead is free"
        self.text(info, self.f_small, MUTED, (225, VIEW_H + 152))

        blink = blink_on(world.time)
        self.lamp(530, VIEW_H + 40, "CRUISE", cc.enabled, GREEN)
        self.lamp(530, VIEW_H + 80, "FOLLOW (car ahead)", cc.following and blink, AMBER)
        self.lamp(530, VIEW_H + 120, "BRAKE WARNING", world.fcw and blink, RED)
        self.text("R = reset   ESC = quit", self.f_tiny, MUTED, (530, VIEW_H + 165))

        labels = {"ACCEL": ("+ SPEED", "Up arrow"), "CRUISE": ("CRUISE", "C key"),
                  "DECEL": ("- SPEED", "Down arrow"), "BRAKE": ("BRAKE", "Space")}
        colors = {"ACCEL": (35, 135, 75), "CRUISE": (35, 95, 195),
                  "DECEL": (165, 115, 25), "BRAKE": (175, 40, 40)}
        for name, rect in self.buttons.items():
            active = pressed.get(name, False) or (name == "CRUISE" and cc.enabled)
            col = shade(colors[name], 1.35) if active else colors[name]
            pygame.draw.rect(s, col, rect, border_radius=12)
            if active:
                pygame.draw.rect(s, TEXT, rect, 3, border_radius=12)
            big, small = labels[name]
            self.text(big, self.f_med, TEXT, (rect.centerx, rect.centery - 9), "center")
            self.text(small, self.f_tiny, (225, 228, 235), (rect.centerx, rect.centery + 16), "center")

    def draw(self, world, pressed):
        self.draw_world(world)
        if world.collision_timer > 0:
            flash = pygame.Surface((WIDTH, VIEW_H), pygame.SRCALPHA)
            flash.fill((220, 0, 0, 80))
            self.s.blit(flash, (0, 0))
            self.text("COLLISION!", self.f_huge, TEXT, (CX, 200), "center")
        self.draw_top_hud(world)
        self.draw_dashboard(world, pressed)


# --------------------------------------------------------------------------- #
# Main loop
# --------------------------------------------------------------------------- #
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("3D Adaptive Cruise Control - pygame")
    clock = pygame.time.Clock()
    world = World()
    renderer = Renderer(screen)
    mouse_button = None

    while True:
        cruise_pressed = False
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                pygame.quit()
                return
            if ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_c:
                    cruise_pressed = True
                elif ev.key == pygame.K_r:
                    world.reset()
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                for name, rect in renderer.buttons.items():
                    if rect.collidepoint(ev.pos):
                        mouse_button = name
                        if name == "CRUISE":
                            cruise_pressed = True
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                mouse_button = None

        keys = pygame.key.get_pressed()
        pressed = {
            "ACCEL": keys[pygame.K_UP] or mouse_button == "ACCEL",
            "DECEL": keys[pygame.K_DOWN] or mouse_button == "DECEL",
            "BRAKE": keys[pygame.K_SPACE] or mouse_button == "BRAKE",
            "CRUISE": mouse_button == "CRUISE",
        }
        inp = Inputs(pressed["ACCEL"], pressed["DECEL"], pressed["BRAKE"], cruise_pressed)

        dt = min(clock.tick(FPS) / 1000.0, 0.05)
        world.step(dt, inp)
        renderer.draw(world, pressed)
        pygame.display.flip()


if __name__ == "__main__":
    main()
    sys.exit(0)
