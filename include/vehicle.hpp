#pragma once
#include "pid.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace demo {
inline void require_finite(double value) {
    if (!std::isfinite(value)) throw std::invalid_argument("non-finite input");
}

class Gearbox {
    int gear_ = 1;
    double since_shift_ = 1.2;
public:
    static constexpr double dwell = 1.2;
    int gear() const { return gear_; }
    double since_shift() const { return since_shift_; }
    double ratio() const {
        constexpr std::array<double, 4> ratios{3.2, 1.9, 1.25, 0.9};
        return ratios.at(static_cast<std::size_t>(gear_ - 1));
    }
    bool update(double speed_mps, double throttle, double dt) {
        require_finite(speed_mps); require_finite(throttle); require_finite(dt);
        if (speed_mps < 0 || throttle < 0 || throttle > 1 || dt <= 0)
            throw std::invalid_argument("invalid gearbox input");
        since_shift_ += dt;
        if (since_shift_ < dwell) return false;
        constexpr std::array<double, 3> up{20, 38, 60};
        constexpr std::array<double, 3> up_load{12, 18, 25};
        constexpr std::array<double, 4> down{0, 11, 25, 43};
        constexpr std::array<double, 4> down_load{0, 6, 8, 12};
        const double kmh = speed_mps * 3.6;
        const auto i = static_cast<std::size_t>(gear_ - 1);
        if (gear_ < 4 && kmh > up[i] + up_load[i] * throttle) ++gear_;
        else if (gear_ > 1 && kmh < down[i] + down_load[i] * throttle) --gear_;
        else return false;
        since_shift_ = 0;
        return true;
    }
};

struct Sample {
    double time, target_mps, speed_mps, grade, command, actuator, rpm;
    int gear;
    bool shifted;
};

class Vehicle {
    double speed_ = 0;
    double actuator_ = 0;
    double time_ = 0;
    SpeedPid pid_{};
    Gearbox gearbox_;
public:
    Vehicle() {
        if (!pid_init(&pid_, 0.22, 0.07, 0.02, 0.15))
            throw std::logic_error("invalid PID gains");
    }
    Sample step(double target_mps, double grade, double dt) {
        require_finite(target_mps); require_finite(grade); require_finite(dt);
        if (target_mps < 0 || std::abs(grade) > 0.2 || dt <= 0 || dt > 0.05)
            throw std::invalid_argument("invalid simulation input");
        double command;
        if (!pid_step(&pid_, target_mps, speed_, dt, &command))
            throw std::runtime_error("PID step failed");
        actuator_ += dt / (0.25 + dt) * (command - actuator_);
        const double throttle = std::max(0.0, actuator_);
        const bool shifted = gearbox_.update(speed_, throttle, dt);
        const double torque_factor = gearbox_.since_shift() < 0.2 ? 0.35 : 1.0;
        const double drive = throttle * 240 * gearbox_.ratio() * 3.7 * 0.9 / 0.31 * torque_factor;
        const double brake = std::max(0.0, -actuator_) * 8000;
        const double resistance = (speed_ > 0 ? 150 : 0) + 0.38 * speed_ * speed_;
        const double slope_force = 1500 * 9.81 * std::sin(std::atan(grade));
        const double acceleration = (drive - brake - resistance - slope_force) / 1500;
        speed_ = std::max(0.0, speed_ + acceleration * dt);
        time_ += dt;
        constexpr double pi = 3.141592653589793;
        const double rpm = std::max(800.0, speed_ / 0.31 * gearbox_.ratio() * 3.7 * 60 / (2 * pi));
        return {time_, target_mps, speed_, grade, command, actuator_, rpm, gearbox_.gear(), shifted};
    }
};

inline double target_speed(double t) {
    if (t < 12) return std::min(18.0, 1.8 * t);
    if (t < 35) return 18;
    if (t < 55) return 27;
    if (t < 75) return 12;
    return 22;
}
inline double road_grade(double t) { return t >= 45 && t < 65 ? 0.05 : 0; }
}
