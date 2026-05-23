#include "cuda_linear_solver.h"

#ifdef HAS_CUDA

#include <cuda_runtime.h>
#include <cublas_v2.h>
#include <cusolverDn.h>

#include <sstream>

namespace {

void setError(std::string* errorMessage, const std::string& message) {
    if (errorMessage != nullptr) {
        *errorMessage = message;
    }
}

} // namespace

namespace LinearSolver {
namespace Cuda {

bool isCompiledWithCuda() {
    return true;
}

bool solveDense(const std::vector<std::vector<double>>& a,
                const std::vector<double>& b,
                std::vector<double>& x,
                std::string* errorMessage) {
    const int n = static_cast<int>(a.size());
    if (n <= 0 || static_cast<int>(b.size()) != n) {
        setError(errorMessage, "invalid matrix dimensions");
        return false;
    }
    for (int i = 0; i < n; ++i) {
        if (static_cast<int>(a[static_cast<std::size_t>(i)].size()) != n) {
            setError(errorMessage, "matrix is not square");
            return false;
        }
    }

    int deviceCount = 0;
    if (cudaGetDeviceCount(&deviceCount) != cudaSuccess || deviceCount <= 0) {
        setError(errorMessage, "no CUDA device available");
        return false;
    }

    double* dA = nullptr;
    double* dB = nullptr;
    int* dPivots = nullptr;
    int* dInfo = nullptr;
    double* dWork = nullptr;
    cusolverDnHandle_t handle = nullptr;

    auto cleanup = [&]() {
        if (handle != nullptr) {
            cusolverDnDestroy(handle);
        }
        if (dWork != nullptr) {
            cudaFree(dWork);
        }
        if (dInfo != nullptr) {
            cudaFree(dInfo);
        }
        if (dPivots != nullptr) {
            cudaFree(dPivots);
        }
        if (dB != nullptr) {
            cudaFree(dB);
        }
        if (dA != nullptr) {
            cudaFree(dA);
        }
    };

    // cuSOLVER dense LU expects column-major storage.
    std::vector<double> aFlat(static_cast<std::size_t>(n * n), 0.0);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            aFlat[static_cast<std::size_t>(j * n + i)] =
                a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
        }
    }

    if (cudaMalloc(reinterpret_cast<void**>(&dA), sizeof(double) * aFlat.size()) != cudaSuccess ||
        cudaMalloc(reinterpret_cast<void**>(&dB), sizeof(double) * static_cast<std::size_t>(n)) != cudaSuccess ||
        cudaMalloc(reinterpret_cast<void**>(&dPivots), sizeof(int) * static_cast<std::size_t>(n)) != cudaSuccess ||
        cudaMalloc(reinterpret_cast<void**>(&dInfo), sizeof(int)) != cudaSuccess) {
        setError(errorMessage, "cudaMalloc failed");
        cleanup();
        return false;
    }

    if (cudaMemcpy(dA,
                   aFlat.data(),
                   sizeof(double) * aFlat.size(),
                   cudaMemcpyHostToDevice) != cudaSuccess ||
        cudaMemcpy(dB,
                   b.data(),
                   sizeof(double) * static_cast<std::size_t>(n),
                   cudaMemcpyHostToDevice) != cudaSuccess) {
        setError(errorMessage, "cudaMemcpy host->device failed");
        cleanup();
        return false;
    }

    if (cusolverDnCreate(&handle) != CUSOLVER_STATUS_SUCCESS) {
        setError(errorMessage, "cusolverDnCreate failed");
        cleanup();
        return false;
    }

    int workElements = 0;
    if (cusolverDnDgetrf_bufferSize(handle, n, n, dA, n, &workElements) != CUSOLVER_STATUS_SUCCESS) {
        setError(errorMessage, "cusolverDnDgetrf_bufferSize failed");
        cleanup();
        return false;
    }
    if (workElements <= 0) {
        setError(errorMessage, "invalid CUDA workspace size");
        cleanup();
        return false;
    }

    if (cudaMalloc(reinterpret_cast<void**>(&dWork),
                   sizeof(double) * static_cast<std::size_t>(workElements)) != cudaSuccess) {
        setError(errorMessage, "cudaMalloc workspace failed");
        cleanup();
        return false;
    }

    if (cusolverDnDgetrf(handle, n, n, dA, n, dWork, dPivots, dInfo) != CUSOLVER_STATUS_SUCCESS) {
        setError(errorMessage, "cusolverDnDgetrf failed");
        cleanup();
        return false;
    }

    int infoValue = 0;
    if (cudaMemcpy(&infoValue, dInfo, sizeof(int), cudaMemcpyDeviceToHost) != cudaSuccess) {
        setError(errorMessage, "cudaMemcpy info failed");
        cleanup();
        return false;
    }
    if (infoValue != 0) {
        std::ostringstream oss;
        oss << "LU factorization failed, info=" << infoValue;
        setError(errorMessage, oss.str());
        cleanup();
        return false;
    }

    if (cusolverDnDgetrs(handle, CUBLAS_OP_N, n, 1, dA, n, dPivots, dB, n, dInfo) !=
        CUSOLVER_STATUS_SUCCESS) {
        setError(errorMessage, "cusolverDnDgetrs failed");
        cleanup();
        return false;
    }

    infoValue = 0;
    if (cudaMemcpy(&infoValue, dInfo, sizeof(int), cudaMemcpyDeviceToHost) != cudaSuccess) {
        setError(errorMessage, "cudaMemcpy solve info failed");
        cleanup();
        return false;
    }
    if (infoValue != 0) {
        std::ostringstream oss;
        oss << "linear solve failed, info=" << infoValue;
        setError(errorMessage, oss.str());
        cleanup();
        return false;
    }

    x.assign(static_cast<std::size_t>(n), 0.0);
    if (cudaMemcpy(x.data(),
                   dB,
                   sizeof(double) * static_cast<std::size_t>(n),
                   cudaMemcpyDeviceToHost) != cudaSuccess) {
        setError(errorMessage, "cudaMemcpy device->host failed");
        cleanup();
        return false;
    }

    cleanup();
    return true;
}

} // namespace Cuda
} // namespace LinearSolver

#else

namespace LinearSolver {
namespace Cuda {

bool isCompiledWithCuda() {
    return false;
}

bool solveDense(const std::vector<std::vector<double>>&,
                const std::vector<double>&,
                std::vector<double>&,
                std::string* errorMessage) {
    if (errorMessage != nullptr) {
        *errorMessage = "CUDA support is not enabled in this build";
    }
    return false;
}

} // namespace Cuda
} // namespace LinearSolver

#endif // HAS_CUDA
