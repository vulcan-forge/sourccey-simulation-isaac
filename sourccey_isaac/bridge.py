"""Optional receive-only localhost UDP preview bridge. Never sends robot commands."""
import json
import socket
import time

from .control import finite_vector


def validate_packet(packet):
    if not isinstance(packet, dict) or packet.get("schema") != "sourccey.mujoco.v1":
        raise ValueError("Expected sourccey.mujoco.v1 preview packet")
    # Validate everything first: a malformed arm must not partially update the base.
    base = finite_vector(packet["base"], 3)
    elevator = finite_vector([packet["elevator"]], 1)[0]
    arms = {}
    for side in ("left", "right"):
        arm = packet[side]
        arms[side] = (finite_vector(arm["degrees"], 5), finite_vector([arm["closure"]], 1)[0])
    return base, elevator, arms


class PreviewReceiver:
    def __init__(self, port=8765, timeout=.3):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", port))
        self.socket.setblocking(False)
        self.timeout = timeout
        self.last_packet = 0.0
        self.rejected = 0

    def poll(self, simulation):
        latest = None
        # Bounded work so a noisy sender cannot starve physics/UI.
        for _ in range(64):
            try:
                raw, _ = self.socket.recvfrom(8192)
            except BlockingIOError:
                break
            try:
                latest = validate_packet(json.loads(raw))
            except (ValueError, KeyError, TypeError, UnicodeDecodeError):
                self.rejected += 1
        if latest is not None:
            base, elevator, arms = latest
            simulation.set_base(*base)
            simulation.set_elevator(elevator)
            for side, (degrees, closure) in arms.items():
                simulation.set_unity_arm(side, degrees, closure)
            self.last_packet = time.monotonic()
        if time.monotonic() - self.last_packet > self.timeout:
            simulation.set_base()

    def close(self):
        self.socket.close()
