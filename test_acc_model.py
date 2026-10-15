"""Unit tests for the cruise control logic (run with: python -m pytest -q)."""
from acc_model import (EGO_LANE, KMH, STANDSTILL_GAP, TIME_GAP, A_COAST, Car,
                       Inputs, World)

DT = 1 / 60


def run(world, seconds, inp=None):
    for _ in range(int(seconds / DT)):
        world.step(DT, inp or Inputs())


def engaged_world(speed_kmh):
    w = World(seed=1, traffic=False)
    w.ego.speed = speed_kmh * KMH
    w.step(DT, Inputs(cruise_pressed=True))
    return w


def test_cruise_off_by_default_normal_driving():
    w = World(seed=1, traffic=False)
    assert not w.cc.enabled and w.mode == "OFF"
    v0 = w.ego.speed
    run(w, 2)                                   # no pedal -> car coasts
    assert w.ego.speed < v0 and w.a_cmd == A_COAST
    run(w, 2, Inputs(accel=True))               # driver accelerates
    assert w.ego.speed > v0 - 2


def test_engage_needs_min_speed():
    w = engaged_world(20)
    assert not w.cc.enabled


def test_cruise_button_toggles_on_and_off():
    w = engaged_world(100)
    assert w.cc.enabled and w.mode == "CRUISE"
    w.step(DT, Inputs(cruise_pressed=True))
    assert not w.cc.enabled


def test_brake_accel_decel_cancel_cruise():
    for inp in (Inputs(brake=True), Inputs(accel=True), Inputs(decel=True)):
        w = engaged_world(100)
        assert w.cc.enabled
        w.step(DT, inp)
        assert not w.cc.enabled


def test_holds_set_speed():
    w = engaged_world(100)
    run(w, 20)
    assert abs(w.ego.speed - 100 * KMH) < 0.5 * KMH


def test_follows_slower_car_at_time_gap():
    w = engaged_world(120)
    lead = Car(w.ego.pos + 130, EGO_LANE, 80 * KMH, (200, 0, 0))
    w.traffic.append(lead)
    run(w, 40)
    assert w.cc.following and w.mode == "FOLLOWING"
    assert abs(w.ego.speed - 80 * KMH) < 2 * KMH
    desired = STANDSTILL_GAP + TIME_GAP * w.ego.speed
    assert abs(w.gap_to(lead) - desired) < 3


def test_lead_signals_then_changes_lane_and_ego_resumes():
    w = engaged_world(120)
    lead = Car(w.ego.pos + 60, EGO_LANE, 80 * KMH, (200, 0, 0), is_lead=True)
    lead.change_after = 5.0
    w.traffic.append(lead)
    run(w, 6)
    assert w.cc.following
    assert lead.indicator != 0                  # indicator blinking before moving
    run(w, 40)
    assert round(lead.lane) != EGO_LANE         # lead moved over
    assert w.cc.enabled and not w.cc.following
    assert abs(w.ego.speed - 120 * KMH) < 1 * KMH


def test_long_random_drive_has_no_collision():
    w = World(seed=7, traffic=True)
    w.ego.speed = 130 * KMH
    w.step(DT, Inputs(cruise_pressed=True))
    run(w, 300)
    assert w.collisions == 0
    assert w.cc.enabled
