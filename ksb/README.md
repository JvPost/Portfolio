# KSB — Kinematic Synchronization Buffer

A pure-Python library for planning jerk-limited motion profiles in systems that must synchronize stochastic item arrivals to a fixed, deterministic departure schedule. Given an upstream flow of items with random inter-arrival times and a downstream slot conveyor running at a fixed gap, KSB computes bounded-jerk trajectories that deliver each item to its assigned slot within a correction buffer of length `L_buffer`, respecting configurable velocity, acceleration, and jerk limits throughout.
