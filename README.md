# Transmission Control Demo

An educational C/C++ project connecting motion-control concepts to a four-speed automatic transmission. A C PID controller tracks vehicle speed; a C++ state machine selects the gear. It is a new learning implementation, not recovered internship code or evidence of past commercial work.

## What it does

- Uses speed and throttle to select one of four forward gears.
- Separates upshift/downshift thresholds (hysteresis) and enforces a 1.2 s minimum dwell time to reduce gear hunting.
- Uses a speed PID with filtered derivative-on-measurement, signed throttle/brake commands, saturation, and conditional-integration anti-windup.
- Simulates acceleration, a target-speed reduction, and a 5% uphill disturbance.
- Writes target/measured speed, gear, throttle/brake command and estimated engine RPM to CSV.

## Build and run

Requires CMake >=3.16 and C11/C++17 compilers. No third-party runtime library, CarSim, MATLAB or paid service is needed.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
./build/simulate simulation.csv
```

For multi-configuration Windows builds, use `build/Release/simulate.exe simulation.csv`.

The synthetic scenario lasts 100 s with a 0.01 s time step. Speeds are in m/s in the CSV; multiply by 3.6 for km/h. `grade=0.05` means a 5% slope, not a 5-degree incline. Each row contains the post-step vehicle state and the reference/commands used for that step. A full-run RMSE includes acceleration and braking transients; it is not a production performance claim.

## How shifting works

For the current gear, compare vehicle speed against two different thresholds:

| Transition | Low-throttle threshold | At full throttle |
|---|---:|---:|
| 1 -> 2 | Above 20 km/h | Above 32 km/h |
| 2 -> 3 | Above 38 km/h | Above 56 km/h |
| 3 -> 4 | Above 60 km/h | Above 85 km/h |
| 2 -> 1 | Below 11 km/h | Below 17 km/h |
| 3 -> 2 | Below 25 km/h | Below 33 km/h |
| 4 -> 3 | Below 43 km/h | Below 55 km/h |

Intermediate thresholds interpolate linearly with throttle. Example: at low throttle, first gear shifts up above20 km/h, but second gear will not shift back down until below11 km/h. That gap and the dwell timer prevent repeated shifts near one threshold. Higher throttle delays upshifting and encourages downshifting under load. These are illustrative calibration values, not manufacturer data.

Gear ratios are3.2,1.9,1.25,0.9. A lower gear multiplies wheel torque more but also raises engine speed at the same vehicle speed. Estimated RPM comes from wheel speed, gear ratio and final drive; the displayed800 RPM floor is only a crude idle approximation. A0.2 s torque reduction after a shift illustrates torque interruption; it is not a clutch model.

## Where PID fits

The discrete speed error is `target_speed - measured_speed`. The proportional term reacts immediately, the integral term corrects persistent error (such as uphill load), and the filtered derivative reacts to changes in measured speed. Derivative-on-measurement avoids a target-step derivative kick. When the output saturates, conditional integration prevents the integral from growing in the same direction as saturation.

PID controls the throttle/brake request here, **not the gear number**. A gear is a discrete state, so shift scheduling uses a state machine. Real transmissions may also use feedback controllers for hydraulic pressure, clutch slip and actuator position; this project does not implement those loops.

The simplified longitudinal plant is:

```text
drive_force = throttle * engine_torque * gear_ratio * final_drive * efficiency / wheel_radius
acceleration = (drive_force - brake_force - rolling/aero resistance - grade_force) / mass
speed_next = max(0, speed + acceleration * dt)
```

The numerical update is explicit Euler with a bounded time step. Throttle/brake actuation has a first-order lag.

## Tests and boundaries

`control_tests` contains8 named tests: input validation, saturation/anti-windup, derivative kick, hysteresis/dwell, load-dependent shifting, gear bounds, flat/uphill tracking, and deceleration/scenario invariants. GitHub Actions compiles C/C++, runs tests and publishes the synthetic CSV. See the actual Actions run for the result; the existence of a workflow is not proof of a pass.

This is **simulation only**. No real-vehicle connection, CAN interface, calibrated engine map, torque-converter dynamics, clutch-slip dynamics, tire traction model, reverse gear, rollback, safety validation or ECU deployment is provided. Do not use it to operate a vehicle. No CarSim integration or performance equivalence is claimed.

## References and provenance

- [MathWorks automatic transmission controller](https://www.mathworks.com/help/simulink/slref/modeling-an-automatic-transmission-controller.html): shift scheduling and the separation of discrete control logic from drivetrain dynamics.
- [CarSim built-in controllers](https://www.carsim.com/products/supporting/controllers/index.php): shift schedules, speed control and clutch-control context.
- [MathWorks PID controller documentation](https://www.mathworks.com/help/simulink/slref/pidcontroller.html): filtered derivative and anti-windup concepts.

The implementation is independently written with AI assistance from these documented concepts. No vendor source/model files or third-party GitHub implementation are copied. The simple equations and calibration choices are intentionally not a reproduction of either vendor's model. Created in November2026; not an old internship deliverable.

## Short project description

Built an educational C/C++ vehicle-control simulator combining a PID speed controller with a four-speed shift state machine. Added throttle-dependent shift schedules, hysteresis, dwell timing, and synthetic acceleration/braking/uphill tests.
