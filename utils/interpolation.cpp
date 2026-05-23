#include "interpolation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

void validateInput(const std::vector<double>& x, const std::vector<double>& y) {
    if (x.size() != y.size() || x.size() < 2) {
        throw std::invalid_argument("Interpolation input vectors must have same size >= 2.");
    }

    for (std::size_t i = 1; i < x.size(); ++i) {
        if (x[i] <= x[i - 1]) {
            throw std::invalid_argument("Interpolation x values must be strictly increasing.");
        }
    }
}

std::size_t findSegment(const std::vector<double>& x, double targetX) {
    const auto it = std::upper_bound(x.begin(), x.end(), targetX);
    if (it == x.begin()) {
        return 0;
    }
    if (it == x.end()) {
        return x.size() - 2;
    }
    return static_cast<std::size_t>(std::distance(x.begin(), it) - 1);
}

double interpolateLinear(const std::vector<double>& x,
                         const std::vector<double>& y,
                         double targetX) {
    if (targetX <= x.front()) {
        return y.front();
    }
    if (targetX >= x.back()) {
        return y.back();
    }

    const std::size_t i = findSegment(x, targetX);
    return Interpolation::linearInterpolate(x[i], y[i], x[i + 1], y[i + 1], targetX);
}

double interpolateNearest(const std::vector<double>& x,
                          const std::vector<double>& y,
                          double targetX) {
    const auto lower = std::lower_bound(x.begin(), x.end(), targetX);
    if (lower == x.begin()) {
        return y.front();
    }
    if (lower == x.end()) {
        return y.back();
    }

    const std::size_t right = static_cast<std::size_t>(std::distance(x.begin(), lower));
    const std::size_t left = right - 1;
    const double leftDist = std::abs(targetX - x[left]);
    const double rightDist = std::abs(x[right] - targetX);

    return (leftDist <= rightDist) ? y[left] : y[right];
}

double catmullRom(double y0, double y1, double y2, double y3, double t) {
    const double a0 = -0.5 * y0 + 1.5 * y1 - 1.5 * y2 + 0.5 * y3;
    const double a1 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
    const double a2 = -0.5 * y0 + 0.5 * y2;
    const double a3 = y1;
    return ((a0 * t + a1) * t + a2) * t + a3;
}

double interpolateCubic(const std::vector<double>& x,
                        const std::vector<double>& y,
                        double targetX) {
    if (x.size() < 4) {
        return interpolateLinear(x, y, targetX);
    }

    if (targetX <= x.front()) {
        return y.front();
    }
    if (targetX >= x.back()) {
        return y.back();
    }

    const std::size_t i1 = findSegment(x, targetX);
    const std::size_t i2 = i1 + 1;
    const std::size_t i0 = (i1 == 0) ? i1 : i1 - 1;
    const std::size_t i3 = (i2 + 1 < x.size()) ? i2 + 1 : i2;

    const double denom = x[i2] - x[i1];
    if (std::abs(denom) < 1e-12) {
        return y[i1];
    }

    const double t = (targetX - x[i1]) / denom;
    return catmullRom(y[i0], y[i1], y[i2], y[i3], t);
}

} // namespace

namespace Interpolation {

double linearInterpolate(double x1, double y1, double x2, double y2, double x) {
    if (std::abs(x2 - x1) < 1e-12) {
        throw std::invalid_argument("x1 and x2 must be different for linear interpolation.");
    }
    return y1 + (y2 - y1) * (x - x1) / (x2 - x1);
}

double interpolate(const std::vector<double>& x,
                   const std::vector<double>& y,
                   double targetX,
                   Method method) {
    validateInput(x, y);

    switch (method) {
        case Method::Linear:
            return interpolateLinear(x, y, targetX);
        case Method::Cubic:
            return interpolateCubic(x, y, targetX);
        case Method::Nearest:
            return interpolateNearest(x, y, targetX);
        default:
            throw std::invalid_argument("Unsupported interpolation method.");
    }
}

} // namespace Interpolation