#include "sim/simulation.h"

#include <iostream>
#include <random>
#include <vector>

#include "config.h"
int run_simulation_suite() {
    int f = 0;
    odneat::SimulationParams sp;
    odneat::sim::RobotPose pose{0.0, 0.0, 0.0};
    odneat::sim::integrateDifferentialDrive(pose, 1.0, 1.0, 0.1, sp);

    if (pose.position_x <= 0.0) {
        std::cout << "FAIL: forward\n";
        ++f;
    }

    std::vector<odneat::sim::SimulatedRobot> robots(2);
    robots[0].pose = {0.0, 0.0, 0.0};
    robots[1].pose = {0.1, 0.0, 0.0};

    if (odneat::sim::countNeighboursInRange(robots, 0, 0.25) != 1) {
        std::cout << "FAIL: neighbours\n";
        ++f;
    }

    if (odneat::sim::computeWallSensors(pose, sp).size() != 8) {
        std::cout << "FAIL: walls\n";
        ++f;
    }

    std::mt19937_64 rng(1);
    odneat::sim::SimulatedRobot faulty;

    if (!odneat::sim::injectRandomSensorFault(faulty, rng)) {
        std::cout << "FAIL: inject\n";
        ++f;
    }

    if (odneat::sim::countFaultySensors(faulty) != 1) {
        std::cout << "FAIL: count\n";
        ++f;
    }

    odneat::sim::SensorReadings sr;
    sr.robot_sensors = std::vector<double>(8, 0.1);
    sr.wall_sensors = std::vector<double>(8, 0.2);
    sr.light_sensors = std::vector<double>(8, 0.3);

    if (odneat::sim::buildAggregationInputs(sr, 0.5, 0.5).size() != 18) {
        std::cout << "FAIL: agg\n";
        ++f;
    }

    if (odneat::sim::buildNavigationInputs(sr, 0.5).size() != 17) {
        std::cout << "FAIL: nav\n";
        ++f;
    }

    if (odneat::sim::buildPhototaxisInputs(sr, 0.5).size() != 25) {
        std::cout << "FAIL: photo\n";
        ++f;
    }

    // Wall stall clamps inside without reflecting heading (no bounce).
    {
        odneat::sim::RobotPose deep{99.0, -99.0, 0.7};
        const double heading = deep.heading_radians;
        odneat::sim::resolveWallCollision(deep, sp, sp.robot_diameter_metres * 0.5);
        const double half = sp.arena_size_metres * 0.5 - sp.robot_diameter_metres * 0.5;
        if (deep.heading_radians != heading || deep.position_x > half || deep.position_y < -half) {
            std::cout << "FAIL: stall\n";
            ++f;
        }
    }

    // Light is directional: front sensor lit, rear dark, far field zero.
    {
        odneat::sim::RobotPose at_origin{0.0, 0.0, 0.0};
        const std::vector<double> near = odneat::sim::computeLightSensors(at_origin, 0.4, 0.0, sp);
        if (near.size() != 8 || near[0] <= 0.1 || near[4] != 0.0) {
            std::cout << "FAIL: directional light\n";
            ++f;
        }

        const std::vector<double> far = odneat::sim::computeLightSensors(at_origin, 9.0, 9.0, sp);
        bool any_lit = false;
        for (double v : far) {
            if (v != 0.0) any_lit = true;
        }

        if (any_lit) {
            std::cout << "FAIL: out-of-range dark\n";
            ++f;
        }
    }

    // Robot sensing ignores self, misses out-of-range, and fires the facing sensor.
    {
        std::vector<odneat::sim::SimulatedRobot> trio(3);
        trio[0].pose = {0.0, 0.0, 0.0};
        trio[1].pose = {0.1, 0.0, 0.0};
        trio[2].pose = {9.0, 9.0, 0.0};
        const std::vector<double> seen = odneat::sim::computeRobotSensors(trio[0].pose, trio, 0, sp);
        if (seen.size() != 8 || seen[0] <= 0.0) {
            std::cout << "FAIL: facing sensor\n";
            ++f;
        }

        double total = 0.0;
        for (double v : seen) total += v;
        if (total <= 0.0 || total > 8.0) {
            std::cout << "FAIL: sensing range\n";
            ++f;
        }

        if (odneat::sim::findNeighboursInRange(trio, 0, 0.25).size() != 1) {
            std::cout << "FAIL: excludes self/far\n";
            ++f;
        }
    }

    // Noise with zero stddev only clamps; nonzero noise stays in [0,1].
    {
        std::mt19937_64 nrng(3);
        if (odneat::sim::addSensorNoise(1.4, nrng, 0.0) != 1.0 || odneat::sim::addSensorNoise(-0.2, nrng, 0.0) != 0.0) {
            std::cout << "FAIL: clamp no-noise\n";
            ++f;
        }

        for (int i = 0; i < 50; ++i) {
            const double noisy = odneat::sim::addSensorNoise(0.5, nrng, 0.05);
            if (noisy < 0.0 || noisy > 1.0) {
                std::cout << "FAIL: noise bounds\n";
                ++f;
                break;
            }
        }
    }

    // Differential drive: spin changes heading with ~zero translation; wheels clamp.
    {
        odneat::sim::RobotPose spin{0.0, 0.0, 0.0};
        odneat::sim::integrateDifferentialDrive(spin, -1.0, 1.0, 0.1, sp);
        if (spin.heading_radians == 0.0) {
            std::cout << "FAIL: spin turns\n";
            ++f;
        }

        odneat::sim::RobotPose wild{0.0, 0.0, 0.0};
        odneat::sim::integrateDifferentialDrive(wild, 5.0, 5.0, 0.1, sp);
        if (wild.position_x > sp.maximum_wheel_speed * 0.1 + 1e-9) {
            std::cout << "FAIL: wheel clamp\n";
            ++f;
        }
    }

    // Placement respects separation (sentinel-origin bug) and arena bounds.
    {
        std::mt19937_64 prng(11);
        std::vector<odneat::sim::SimulatedRobot> placed(5);
        odneat::sim::placeRobotsWithSeparation(placed, prng, sp, 1.5);
        for (std::size_t i = 0; i < placed.size(); ++i) {
            for (std::size_t j = i + 1; j < placed.size(); ++j) {
                const double dx = placed[i].pose.position_x - placed[j].pose.position_x;
                const double dy = placed[i].pose.position_y - placed[j].pose.position_y;
                if (dx * dx + dy * dy < 1.5 * 1.5 - 1e-9) {
                    // Best-effort placement may still collide in tight arenas; only fail if many collide.
                    std::cout << "FAIL: separation\n";
                    ++f;
                    i = placed.size();
                    break;
                }
            }
        }

        std::vector<odneat::sim::SimulatedRobot> uniform(3);
        odneat::sim::placeRobotsUniformly(uniform, prng, sp);
        const double half = sp.arena_size_metres * 0.5;
        for (const auto &r : uniform) {
            if (r.pose.position_x < -half || r.pose.position_x > half) {
                std::cout << "FAIL: uniform bounds\n";
                ++f;
                break;
            }
        }
    }

    // Fault banks exhaust: all 24 faulty -> further injection fails.
    {
        std::mt19937_64 frng(21);
        odneat::sim::SimulatedRobot saturated;
        int injected = 0;
        for (int i = 0; i < 30; ++i) {
            if (odneat::sim::injectRandomSensorFault(saturated, frng)) ++injected;
        }

        if (injected != 24 || odneat::sim::countFaultySensors(saturated) != 24) {
            std::cout << "FAIL: fault exhaustion\n";
            ++f;
        }
    }

    // Input builders clamp energy/count into [0,1] without changing layout.
    {
        odneat::sim::SensorReadings clamp_sr;
        clamp_sr.robot_sensors = std::vector<double>(8, 0.1);
        clamp_sr.wall_sensors = std::vector<double>(8, 0.2);
        clamp_sr.light_sensors = std::vector<double>(8, 0.3);
        const std::vector<double> agg = odneat::sim::buildAggregationInputs(clamp_sr, 5.0, 99.0);
        if (agg.size() != 18 || agg[16] != 1.0 || agg[17] != 1.0) {
            std::cout << "FAIL: input clamp\n";
            ++f;
        }
    }

    if (f == 0) std::cout << "test_simulation passed\n";

    return f == 0 ? 0 : 1;
}
