"""Unity preview mappings, without hardware servo calibration or transport."""
from dataclasses import dataclass
import numpy as np

from .model import ROLES, ELEVATOR_UPPER_METERS


def finite_vector(value, size):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.all(np.isfinite(a)):
        raise ValueError(f"Expected {size} finite numbers")
    return a


def unity_arm(side, degrees, closure=100):
    """Raw standalone-rig degrees -> full robot URDF radians (FullRobotVisualDriver).

    Input order: pan, lift, elbow, logical wrist flex, logical wrist roll.
    This is NOT RobotAction's calibrated hardware .pos representation.
    """
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    pan, lift, elbow, flex, roll = finite_vector(degrees, 5)
    values = [pan - 45 if side == "left" else -pan - 45,
              -lift if side == "left" else lift, elbow,
              flex if side == "left" else -flex, roll, gripper_degrees(closure)]
    return dict(zip((side + "_" + role for role in ROLES), np.deg2rad(values)))


def gripper_degrees(closure):
    closure = finite_vector([closure], 1)[0]
    return 60 - 65 * np.clip(closure, 0, 100) / 100


def elevator_position(command, full_range=False):
    value = np.clip(finite_vector([command], 1)[0], -100, 100)
    bottom = -0.315 if full_range else -0.3104
    return bottom + (value + 100) / 200 * (ELEVATOR_UPPER_METERS - bottom)


def wheel_rates(command):
    """Normalized forward/left/CCW -> CAD wheel rad/s, FL FR RL RR."""
    v, s, w = np.clip(finite_vector(command, 3), -1, 1) * [0.6, 0.6, np.pi / 2]
    return np.array([v-s-.30*w, -(v+s+.30*w), v+s-.30*w, -(v-s+.30*w)]) / .052


@dataclass
class HandTarget:
    """Target in the moving shoulder-mount frame; hand deltas in tracking-world meters.

    Tracking axes must be converted to right-handed Z-up before calling update.
    Unfreeze captures a fresh hand baseline, so motion while frozen never jumps.
    """
    local: np.ndarray
    frozen: bool = True
    previous: np.ndarray | None = None
    gain: float = 1.0

    def set_frozen(self, frozen, hand_position=None):
        if not frozen and hand_position is None:
            raise ValueError("Unfreeze requires a current hand sample to rebase")
        self.previous = None if hand_position is None else finite_vector(hand_position, 3).copy()
        self.frozen = bool(frozen)

    def update(self, hand_position, robot_rotation):
        hand = finite_vector(hand_position, 3)
        if not self.frozen and self.previous is not None:
            self.local += robot_rotation.T @ (hand - self.previous) * self.gain
        self.previous = hand.copy()
        return self.local.copy()

    def world(self, mount_position, robot_rotation):
        return mount_position + robot_rotation @ self.local
