"""Synthetic stateless wrench fixtures; no EDF model."""
from dataclasses import dataclass

from .types import BodyWrench, StageState, scalar


@dataclass(frozen=True)
class ConstantBodyWrench:
    """Constant non-gravity body force N and COM torque N m."""
    wrench: BodyWrench

    def evaluate(self, t_s: float, state: StageState) -> BodyWrench:
        """Return the prescribed body wrench at finite nonnegative time in seconds."""
        if scalar(t_s, "t_s") < 0:
            raise ValueError("t_s must be nonnegative")
        return self.wrench


class ZeroWrench:
    """Zero force and zero COM torque."""
    def evaluate(self, t_s: float, state: StageState) -> BodyWrench:
        """Return zero wrench at finite nonnegative time in seconds."""
        if scalar(t_s, "t_s") < 0:
            raise ValueError("t_s must be nonnegative")
        return BodyWrench([0, 0, 0], [0, 0, 0])
