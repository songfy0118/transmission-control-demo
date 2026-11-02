# Validation

Verified November 1, 2026 (America/Chicago) using GitHub Actions, not a local Windows compiler.

- Implementation commit: `545ad2a0c350a82fad4a3d6a68372f85004fadd8`.
- [Build/test run](https://github.com/songfy0118/transmission-control-demo/actions/runs/41377365632): GNU C/C++ 15.2.0, Ubuntu; both C11 and C++17 compiled successfully.
- CTest: 1/1 executable passed; it contains 8 named test groups (not disabled-in-release assertions).
- Synthetic simulation: 100 seconds, 10000 steps, dt=0.01 s; whole-run speed RMSE=2.59022 m/s, including acceleration and braking transients; 5 gear changes.
- [CSV artifact](https://github.com/songfy0118/transmission-control-demo/actions/runs/41377365632/artifacts/13111462595). Artifacts have finite retention; rerun the simulator to regenerate it.

Tests check flat-road and uphill steady tracking within 0.5 m/s after settling, deceleration, at least 1.2 s between shifts, bounded commands and valid gears. These are toy-plant results, not real-vehicle performance/safety evidence or an old internship outcome. No CarSim integration is claimed.

No local compiler was found and no software was installed. Later documentation-only changes do not alter the tested source.
