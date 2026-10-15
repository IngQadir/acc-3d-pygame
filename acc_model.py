"""
acc_model.py - vehicle, traffic and Adaptive Cruise Control logic.

No pygame in here, so the logic can be unit tested headless.
Units: metres, seconds, m/s, m/s^2. Lanes are numbered 0 (left) .. 2 (right).
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

# --------------------------------------------------------------------------- #
# Parameters
# --------------------------------------------------------------------------- #
KMH = 1 / 3.6                # km/h -> m/s
LANE_COUNT = 3
EGO_LANE = 1                 # ego drives in the middle lane
LANE_WIDTH = 3.5             # m
CAR_LEN_M = 4.5              # m

V_MAX = 180 * KMH
CC_MIN_SPEED = 30 * KMH

# Normal driving (cruise OFF) - driver inputs
A_THROTTLE = 2.5
A_DECEL = -2.0
A_BRAKE = -7.0
A_COAST = -0.3               # rolling resistance + drag when no pedal pressed
ACTUATOR_TAU = 0.3           # s, first-order powertrain / brake lag

# Cruise control: speed PI + time-gap controller
KP, KI = 0.6, 0.08
INTEGRAL_LIMIT = 20.0
ACC_A_MAX, ACC_A_MIN = 1.8, -3.5
TIME_GAP = 1.8               # s
STANDSTILL_GAP = 6.0         # m
K_GAP, K_REL = 0.15, 0.7
SENSOR_RANGE = 150.0         # m (radar)

# Traffic
SPAWN_AHEAD = 200.0          # m
SIGNAL_TIME = 2.5            # s indicator on before the lead car moves over
LANE_CHANGE_TIME = 2.5       # s
BLINK_PERIOD = 0.66          # s (~1.5 Hz like a real indicator)

TRAFFIC_COLORS = [(200, 50, 50), (235, 200, 60), (190, 190, 200),
                  (120, 70, 170), (240, 130, 40), (60, 160, 150), (240, 240, 240)]
EGO_COLOR = (30, 110, 220)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def blink_on(t):
    return (t % BLINK_PERIOD) < BLINK_PERIOD / 2


@dataclass
class Inputs:
    accel: bool = False
    decel: bool = False
    brake: bool = False
    cruise_pressed: bool = False   # rising edge of the CRUISE button


class Car:
    def __init__(self, pos, lane, speed, color, is_lead=False):
        self.pos = pos               # m, longitudinal position of the car centre
        self.lane = float(lane)      # float so lane changes are smooth
        self.speed = speed
        self.desired_speed = speed
        self.accel = 0.0
        self.color = color
        self.is_lead = is_lead       # lead cars change lane after a while
        self.indicator = 0           # -1 left, +1 right, 0 off
        self.mode = "drive"          # drive -> signal -> change -> done
        self.timer = 0.0
        self.lc_from = lane
        self.lc_to = lane
        self.follow_time = 0.0
        self.change_after = 4.0

    def start_lane_change(self, to_lane):
        self.lc_from = round(self.lane)
        self.lc_to = to_lane
        self.indicator = -1 if to_lane < self.lc_from else 1
        self.mode = "signal"
        self.timer = 0.0

    def update(self, dt, leader):
        target = self.desired_speed
        if leader is not None:
            gap = leader.pos - self.pos - CAR_LEN_M
            if gap < 15 + 1.5 * self.speed:
                target = min(target, leader.speed)
            if gap < 8:
                target = min(target, leader.speed * 0.9)
        old = self.speed
        self.speed += clamp(target - self.speed, -4 * dt, 1.5 * dt)
        self.accel = (self.speed - old) / dt if dt > 0 else 0.0
        self.pos += self.speed * dt

        if self.mode == "signal":
            self.timer += dt
            if self.timer >= SIGNAL_TIME:
                self.mode, self.timer = "change", 0.0
        elif self.mode == "change":
            self.timer += dt
            t = min(1.0, self.timer / LANE_CHANGE_TIME)
            s = t * t * (3 - 2 * t)              # smoothstep
            self.lane = self.lc_from + (self.lc_to - self.lc_from) * s
            if t >= 1.0:
                self.lane = float(self.lc_to)
                self.mode = "done"
                self.indicator = 0


class CruiseControl:
    """Speed PI controller + time-gap controller with min() arbitration."""

    def __init__(self):
        self.enabled = False
        self.set_speed = None
        self.integral = 0.0
        self.following = False
        self.last_event = "Normal driving - press CRUISE (C) to engage (>= 30 km/h)"

    def engage(self, v):
        if v < CC_MIN_SPEED:
            self.last_event = "Cruise needs at least 30 km/h"
            return
        self.enabled = True
        self.set_speed = v
        self.integral = 0.0
        self.last_event = f"Cruise ON - set to {v / KMH:.0f} km/h"

    def cancel(self, reason):
        if self.enabled:
            self.enabled = False
            self.following = False
            self.last_event = f"Cruise OFF ({reason}) - normal driving"

    def step(self, v, gap, v_lead, dt):
        err = self.set_speed - v
        a = KP * err + KI * self.integral
        self.following = False
        if gap is not None:
            desired = STANDSTILL_GAP + TIME_GAP * v
            a_gap = K_GAP * (gap - desired) + K_REL * (v_lead - v)
            if a_gap < a:
                a = a_gap
                self.following = True
        a_sat = clamp(a, ACC_A_MIN, ACC_A_MAX)
        # anti-windup: integrate only in pure speed mode and when not saturated
        if not self.following and a == a_sat:
            self.integral = clamp(self.integral + err * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
        return a_sat


class World:
    def __init__(self, seed=None, traffic=True):
        self.rng = random.Random(seed)
        self.traffic_enabled = traffic
        self.reset()

    def reset(self):
        self.ego = Car(0.0, EGO_LANE, 80 * KMH, EGO_COLOR)
        self.cc = CruiseControl()
        self.traffic: list[Car] = []
        self.time = 0.0
        self.a_cmd = 0.0
        self.target = None
        self.fcw = False
        self.collisions = 0
        self.collision_timer = 0.0
        self.lead_timer = 4.0
        self.next_lead_delay = 8.0
        self.ambient_timer = 1.5

    # ---- helpers --------------------------------------------------------- #
    def gap_to(self, car):
        return car.pos - self.ego.pos - CAR_LEN_M

    def find_target(self):
        """Radar: nearest car in the ego lane, ahead, within sensor range."""
        best = None
        for c in self.traffic:
            if abs(c.lane - EGO_LANE) < 0.5 and c.pos > self.ego.pos:
                if self.gap_to(c) < SENSOR_RANGE and (best is None or c.pos < best.pos):
                    best = c
        return best

    def leader_of(self, car):
        lane = round(car.lane)
        best = None
        for c in self.traffic:
            if c is car or round(c.lane) != lane or c.pos <= car.pos:
                continue
            if best is None or c.pos < best.pos:
                best = c
        return best

    def lane_free(self, lane, pos, exclude=None, margin=40.0):
        for c in self.traffic:
            if c is exclude:
                continue
            lanes = {round(c.lane), c.lc_to if c.mode in ("signal", "change") else round(c.lane)}
            if lane in lanes and abs(c.pos - pos) < margin:
                return False
        return True

    @property
    def mode(self):
        if not self.cc.enabled:
            return "OFF"
        return "FOLLOWING" if self.cc.following else "CRUISE"

    # ---- simulation step ------------------------------------------------- #
    def step(self, dt, inp: Inputs):
        self.time += dt
        ego, cc = self.ego, self.cc

        if inp.cruise_pressed:
            if cc.enabled:
                cc.cancel("CRUISE button")
            else:
                cc.engage(ego.speed)

        if cc.enabled:                       # driver override -> cruise off
            if inp.brake:
                cc.cancel("BRAKE")
            elif inp.accel:
                cc.cancel("ACCELERATOR")
            elif inp.decel:
                cc.cancel("DECELERATE")

        self.target = self.find_target()
        gap = v_lead = None
        if self.target is not None:
            gap, v_lead = self.gap_to(self.target), self.target.speed

        if cc.enabled:
            a_cmd = cc.step(ego.speed, gap, v_lead, dt)
        else:                                # normal driving
            cc.following = False
            if inp.brake:
                a_cmd = A_BRAKE
            elif inp.accel:
                a_cmd = A_THROTTLE
            elif inp.decel:
                a_cmd = A_DECEL
            else:
                a_cmd = A_COAST
        self.a_cmd = a_cmd

        ego.accel += (a_cmd - ego.accel) * min(1.0, dt / ACTUATOR_TAU)
        ego.speed = clamp(ego.speed + ego.accel * dt, 0.0, V_MAX)
        if ego.speed == 0.0 and ego.accel < 0:
            ego.accel = 0.0
        ego.pos += ego.speed * dt

        for car in list(self.traffic):
            car.update(dt, self.leader_of(car))
        self._update_leads(dt)
        if self.traffic_enabled:
            self._spawn(dt)
        self._check_collision()
        self._update_warning()
        self.traffic = [c for c in self.traffic if -120 < c.pos - ego.pos < 280]
        self.collision_timer = max(0.0, self.collision_timer - dt)

    def _update_leads(self, dt):
        for car in self.traffic:
            if not (car.is_lead and car.mode == "drive" and abs(car.lane - EGO_LANE) < 0.5):
                continue
            gap = self.gap_to(car)
            if 0 < gap < 100:
                car.follow_time += dt
                if car.follow_time >= car.change_after:
                    lanes = [l for l in (EGO_LANE - 1, EGO_LANE + 1)
                             if 0 <= l < LANE_COUNT and self.lane_free(l, car.pos, car)]
                    if lanes:
                        car.start_lane_change(self.rng.choice(lanes))

    def _spawn(self, dt):
        ego = self.ego
        lane_busy = any(abs(c.lane - EGO_LANE) < 0.5 and c.pos > ego.pos for c in self.traffic)
        if lane_busy:
            self.lead_timer = self.next_lead_delay
        else:
            self.lead_timer -= dt
            if self.lead_timer <= 0 and ego.speed > 60 * KMH:
                v = clamp(ego.speed - self.rng.uniform(25, 45) * KMH, 40 * KMH, 110 * KMH)
                lead = Car(ego.pos + SPAWN_AHEAD, EGO_LANE, v,
                           self.rng.choice(TRAFFIC_COLORS), is_lead=True)
                lead.change_after = self.rng.uniform(3.0, 6.0)
                self.traffic.append(lead)
                self.next_lead_delay = self.rng.uniform(7.0, 12.0)

        self.ambient_timer -= dt
        if self.ambient_timer <= 0:
            self.ambient_timer = self.rng.uniform(2.5, 6.0)
            lane = self.rng.choice([l for l in range(LANE_COUNT) if l != EGO_LANE])
            pos = ego.pos + SPAWN_AHEAD
            if self.lane_free(lane, pos, margin=60):
                v = max(40 * KMH, ego.speed - self.rng.uniform(10, 30) * KMH)
                self.traffic.append(Car(pos, lane, v, self.rng.choice(TRAFFIC_COLORS)))

    def _check_collision(self):
        t = self.find_target()
        if t is not None and self.gap_to(t) <= 0:
            self.ego.pos = t.pos - CAR_LEN_M
            self.ego.speed = t.speed * 0.8
            self.ego.accel = 0.0
            self.collisions += 1
            self.collision_timer = 2.0
            self.cc.cancel("COLLISION")

    def _update_warning(self):
        t, ego = self.target, self.ego
        self.fcw = False
        if t is None or ego.speed < 2:
            return
        gap = self.gap_to(t)
        closing = ego.speed - t.speed
        ttc = gap / closing if closing > 0.1 else math.inf
        desired = STANDSTILL_GAP + TIME_GAP * ego.speed
        self.fcw = ttc < 2.5 or gap < 0.5 * desired
