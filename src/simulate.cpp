#include "vehicle.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
    if (argc > 2) { std::cerr << "Usage: simulate [output.csv]\n"; return 2; }
    const std::string path = argc == 2 ? argv[1] : "simulation.csv";
    std::ofstream out(path);
    if (!out) { std::cerr << "Cannot open output: " << path << '\n'; return 1; }
    out << "time_s,target_mps,speed_mps,grade,command,actuator,rpm,gear,shifted\n";
    out << std::fixed << std::setprecision(6);
    demo::Vehicle vehicle;
    double sum_squared_error = 0;
    int shifts = 0;
    for (int i = 0; i < 10000; ++i) {
        const double t = i * 0.01;
        const auto s = vehicle.step(demo::target_speed(t), demo::road_grade(t), 0.01);
        out << s.time << ',' << s.target_mps << ',' << s.speed_mps << ',' << s.grade << ','
            << s.command << ',' << s.actuator << ',' << s.rpm << ',' << s.gear << ',' << s.shifted << '\n';
        sum_squared_error += std::pow(s.target_mps - s.speed_mps, 2);
        shifts += s.shifted;
    }
    out.close();
    if (!out) { std::cerr << "Failed to write output\n"; return 1; }
    std::cout << "100 s synthetic scenario; RMSE=" << std::sqrt(sum_squared_error / 10000)
              << " m/s; shifts=" << shifts << "; output=" << path << '\n';
}
