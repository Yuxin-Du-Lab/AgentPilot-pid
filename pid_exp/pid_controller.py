"""1D PID controller with heading wrap-around support."""


def normalize_heading_error(error: float) -> float:
    """折叠到 (-180, 180]: 350 → -10, -350 → +10."""
    return ((error + 180) % 360) - 180


class PIDController:
    """1D PID controller. 支持 heading wrap-around。"""

    def __init__(self, kp: float, ki: float, kd: float, axis: str):
        if axis not in ("heading", "altitude"):
            raise ValueError(f"axis must be 'heading' or 'altitude', got {axis!r}")
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.axis = axis
        self._integral = 0.0
        self._prev_error: float | None = None

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = None

    def _compute_error(self, current: float, target: float) -> float:
        raw = target - current
        if self.axis == "heading":
            return normalize_heading_error(raw)
        return raw

    def update(self, current: float, target: float, dt: float) -> dict:
        error = self._compute_error(current, target)

        # Integral 累积
        self._integral += error * dt

        # Derivative（首次调用或 dt<=0 时返回 0）
        if self._prev_error is None or dt <= 0:
            derivative = 0.0
        else:
            derivative = (error - self._prev_error) / dt

        p_term = self.kp * error
        i_term = self.ki * self._integral
        d_term = self.kd * derivative
        output = p_term + i_term + d_term

        self._prev_error = error

        return {
            "output": output,
            "error": error,
            "derivative": derivative,
            "integral": self._integral,
            "p_term": p_term,
            "i_term": i_term,
            "d_term": d_term,
        }
