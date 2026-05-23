#ifndef CUDA_LINEAR_SOLVER_H
#define CUDA_LINEAR_SOLVER_H

#include <string>
#include <vector>

namespace LinearSolver {
namespace Cuda {

bool isCompiledWithCuda();

bool solveDense(const std::vector<std::vector<double>>& a,
                const std::vector<double>& b,
                std::vector<double>& x,
                std::string* errorMessage = nullptr);

} // namespace Cuda
} // namespace LinearSolver

#endif // CUDA_LINEAR_SOLVER_H
