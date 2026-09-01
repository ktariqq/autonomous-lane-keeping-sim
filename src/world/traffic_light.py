"""Ground-truth traffic-light state machine, independent of what the
perception stack detects — allows truth-vs-detection comparison in the HUD."""


class TrafficLightSystem:
    CYCLE = [("green", 5.0), ("yellow", 1.6), ("red", 4.5)]

    def __init__(self):
        self._phase_i = 0
        self._timer = self.CYCLE[0][1]
        self.state = self.CYCLE[0][0]
        self.override_timer = 0.0

    def force(self, state):
        self.state = state
        self.override_timer = 6.0

    def update(self, dt):
        if self.override_timer > 0:
            self.override_timer -= dt
            return
        self._timer -= dt
        if self._timer <= 0:
            self._phase_i = (self._phase_i + 1) % len(self.CYCLE)
            self.state, self._timer = self.CYCLE[self._phase_i]