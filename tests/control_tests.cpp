#include "vehicle.hpp"
#include <functional>
#include <iostream>
#include <limits>
#include <string>

static void expect(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}
static void rejects(const std::function<void()> &f) {
    bool caught = false;
    try { f(); } catch (const std::invalid_argument &) { caught = true; }
    expect(caught, "expected invalid_argument");
}
int main() {
    int count = 0;
    auto test = [&](const std::string &name, const std::function<void()> &f) {
        f(); ++count; std::cout << "PASS " << name << '\n';
    };
    try {
        test("PID validation", [] {
            SpeedPid p{}; double u = 9;
            expect(!pid_init(nullptr, 1, 0, 0, 0), "null PID accepted");
            expect(!pid_init(&p, -1, 0, 0, 0), "negative gain accepted");
            pid_init(&p, 0.2, 0.1, 0, 0.1);
            expect(!pid_step(&p, 1, 0, 0, &u) && u == 0, "invalid dt not safe");
            expect(!pid_step(&p, std::numeric_limits<double>::quiet_NaN(), 0, 0.01, &u), "NaN accepted");
        });
        test("PID saturation and anti-windup", [] {
            SpeedPid p{}; double u;
            pid_init(&p, 1, 1, 0, 0);
            for (int i = 0; i < 1000; ++i) {
                expect(pid_step(&p, 100, 0, 0.01, &u), "PID rejected valid input");
                expect(u == 1, "output not clamped");
            }
            pid_step(&p, 0, 0, 0.01, &u);
            expect(std::abs(u) < 1e-12, "integral wound up while saturated");
        });
        test("no derivative kick from target step", [] {
            SpeedPid p{}; double u;
            pid_init(&p, 0, 0, 1, 0.1);
            pid_step(&p, 0, 10, 0.01, &u);
            pid_step(&p, 20, 10, 0.01, &u);
            expect(u == 0, "target generated derivative kick");
        });
        test("upshift hysteresis and dwell", [] {
            demo::Gearbox g;
            expect(g.update(22.0 / 3.6, 0, 0.01) && g.gear() == 2, "upshift missing");
            expect(!g.update(100.0 / 3.6, 0, 0.1), "dwell bypassed");
            expect(!g.update(16.0 / 3.6, 0, 1.3) && g.gear() == 2, "hunting inside hysteresis band");
            expect(g.update(9.0 / 3.6, 0, 1.3) && g.gear() == 1, "downshift missing");
        });
        test("throttle delays upshift", [] {
            demo::Gearbox g;
            expect(!g.update(25.0 / 3.6, 1, 0.01), "high-load upshift too early");
            expect(g.update(34.0 / 3.6, 1, 0.01), "high-load upshift missing");
        });
        test("gear limits and invalid inputs", [] {
            demo::Gearbox g;
            for (int i = 0; i < 10; ++i) g.update(40, 0, 1.3);
            expect(g.gear() == 4, "exceeded top gear");
            for (int i = 0; i < 10; ++i) g.update(0, 0, 1.3);
            expect(g.gear() == 1, "went below first gear");
            rejects([&] { g.update(1, 2, 0.01); });
            demo::Vehicle v;
            rejects([&] { v.step(10, 0, 0.1); });
            rejects([&] { v.step(-1, 0, 0.01); });
        });
        test("flat-road tracking and uphill recovery", [] {
            demo::Vehicle v; demo::Sample s{};
            for (int i = 0; i < 6000; ++i) s = v.step(20, 0, 0.01);
            expect(std::abs(s.speed_mps - 20) < 0.5, "flat-road tracking failed");
            for (int i = 0; i < 3000; ++i) s = v.step(20, 0.05, 0.01);
            expect(std::abs(s.speed_mps - 20) < 0.5, "uphill recovery failed");
        });
        test("deceleration and scenario invariants", [] {
            demo::Vehicle v; demo::Sample s{};
            int last_shift = -10000;
            for (int i = 0; i < 10000; ++i) {
                s = v.step(demo::target_speed(i * 0.01), demo::road_grade(i * 0.01), 0.01);
                expect(std::isfinite(s.speed_mps) && s.speed_mps >= 0, "invalid speed");
                expect(std::abs(s.command) <= 1 && std::abs(s.actuator) <= 1, "invalid command");
                expect(s.gear >= 1 && s.gear <= 4, "invalid gear");
                if (s.shifted) {
                    expect(i - last_shift >= 120, "shift chatter"); last_shift = i;
                }
                if (i == 7400) expect(std::abs(s.speed_mps - 12) < 0.6, "braking failed");
            }
            expect(std::abs(s.speed_mps - 22) < 0.6, "final target not reached");
        });
    } catch (const std::exception &e) {
        std::cerr << "FAIL after " << count << " tests: " << e.what() << '\n'; return 1;
    }
    std::cout << count << " tests passed\n";
}
