from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from ksb.motion.item_pair import PairRecord
from ksb.motion.trajectories import CompositeTrajectory, TrajectoryProfile
from ksb.analysis import SegmentEvents, SegmentSyncResponse


@dataclass(frozen=True)
class SimulationResult:
    cfg: dict
    t_spawn: np.ndarray
    t_control_start: np.ndarray
    assigned_slots: np.ndarray
    time_horizons: np.ndarray
    skip_indices: np.ndarray
    phi_u: np.ndarray
    phi_0: np.ndarray
    system_trajectories: List[CompositeTrajectory]
    buffer_trajectories: List[TrajectoryProfile]
    pair_records: List[PairRecord]
    segment_events: Optional[SegmentEvents] = None  # SegmentEvents if batch >= 2, else None
    segment_sync_response: Optional[SegmentSyncResponse] = None
