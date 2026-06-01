#include "trajectory.hpp"
#include <cmath>
#include <limits>

namespace tools
{
constexpr double g = 9.7833;

Trajectory::Trajectory(const double v0, const double d, const double h)
{
  auto a = g * d * d / (2 * v0 * v0);
  auto b = -d;
  auto c = a + h;
  auto delta = b * b - 4 * a * c;

  if (delta < 0) {
    unsolvable = true;
    return;
  }

  unsolvable = false;
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  auto pitch_1 = std::atan(tan_pitch_1);
  auto pitch_2 = std::atan(tan_pitch_2);
  auto t_1 = d / (v0 * std::cos(pitch_1));
  auto t_2 = d / (v0 * std::cos(pitch_2));

  pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  fly_time = (t_1 < t_2) ? t_1 : t_2;
}

double TrajectoryWithDrag::calcTime(double v0, double d, double k, double theta)
{
    double denom = v0 * std::cos(theta);
    if (denom <= 0) return -1.0;
    double ratio = k * d / denom;
    if (ratio >= 1.0) return -1.0;
    return -std::log(1.0 - ratio) / k;
}

double TrajectoryWithDrag::f(double v0, double d, double h, double k, double theta)
{
    double t = calcTime(v0, d, k, theta);
    if (t < 0) return std::numeric_limits<double>::infinity();
    return v0 * std::sin(theta) * t - 0.5 * g * t * t - h;
}

void TrajectoryWithDrag::solve(double v0, double d, double h, double k)
{
    double left  = -1.0;
    double right =  1.0;
    bool found = false;
    double prev_theta = left;
    double prev_f = f(v0, d, h, k, prev_theta);

    for (int i = 1; i <= 100; i++) {
        double theta = left + (right - left) * i / 100.0;
        double cur_f = f(v0, d, h, k, theta);
        if (std::isfinite(prev_f) && std::isfinite(cur_f) && prev_f * cur_f < 0) {
            left = prev_theta;
            right = theta;
            found = true;
            break;
        }
        prev_theta = theta;
        prev_f = cur_f;
    }

    if (!found) { unsolvable = true; return; }

    for (int i = 0; i < 60; i++) {
        double mid = 0.5 * (left + right);
        double val = f(v0, d, h, k, mid);
        if (!std::isfinite(val)) { right = mid; continue; }
        if (val > 0) right = mid;
        else         left  = mid;
    }

    pitch    = 0.5 * (left + right);
    fly_time = calcTime(v0, d, k, pitch);
    if (fly_time < 0) unsolvable = true;
}

TrajectoryWithDrag::TrajectoryWithDrag(double v0, double d, double h, double k)
{
    solve(v0, d, h, k);
}
}  // namespace tools