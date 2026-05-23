#ifndef SPARSE_MATRIX_H
#define SPARSE_MATRIX_H

#include <cmath>
#include <vector>

namespace LinearSolver {

class SparseMatrix {
public:
    std::vector<double> values;
    std::vector<int> colIndices;
    std::vector<int> rowPtr;
    int nRows;
    int nCols;

    explicit SparseMatrix(int n = 0) : nRows(n), nCols(n) { rowPtr.push_back(0); }

    static SparseMatrix fromDense(const std::vector<std::vector<double>>& a, double tol = 1e-14) {
        const int n = static_cast<int>(a.size());
        SparseMatrix s(n);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (std::abs(a[i][j]) > tol) {
                    s.values.push_back(a[i][j]);
                    s.colIndices.push_back(j);
                }
            }
            s.rowPtr.push_back(static_cast<int>(s.values.size()));
        }
        return s;
    }

    void matvec(const std::vector<double>& x, std::vector<double>& y) const {
        y.assign(static_cast<std::size_t>(nRows), 0.0);
        for (int i = 0; i < nRows; ++i) {
            double sum = 0.0;
            for (int k = rowPtr[static_cast<std::size_t>(i)];
                 k < rowPtr[static_cast<std::size_t>(i + 1)];
                 ++k) {
                sum += values[static_cast<std::size_t>(k)] *
                       x[static_cast<std::size_t>(colIndices[static_cast<std::size_t>(k)])];
            }
            y[static_cast<std::size_t>(i)] = sum;
        }
    }
};

inline double dot(const std::vector<double>& u, const std::vector<double>& v) {
    double sum = 0.0;
    for (std::size_t i = 0; i < u.size(); ++i) {
        sum += u[i] * v[i];
    }
    return sum;
}

inline void saxpy(double alpha, const std::vector<double>& x, std::vector<double>& y) {
    for (std::size_t i = 0; i < x.size(); ++i) {
        y[i] += alpha * x[i];
    }
}

} // namespace LinearSolver

#endif // SPARSE_MATRIX_H
