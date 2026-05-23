#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <vector>

namespace Interpolation {

enum class Method {
    Linear,
    Cubic,
    Nearest
};

double linearInterpolate(double x1, double y1, double x2, double y2, double x);

double interpolate(const std::vector<double>& x,
                   const std::vector<double>& y,
                   double targetX,
                   Method method = Method::Linear);

} // namespace Interpolation

#endif // INTERPOLATION_H