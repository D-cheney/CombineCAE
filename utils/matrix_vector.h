#ifndef MATRIX_VECTOR_H
#define MATRIX_VECTOR_H

#include <cstddef>
#include <vector>

class Vector {
private:
    std::vector<double> data_;

public:
    Vector(std::size_t size = 0);
    Vector(const std::vector<double>& vec);

    std::size_t size() const;
    double& operator[](std::size_t index);
    const double& operator[](std::size_t index) const;

    Vector operator+(const Vector& other) const;
    Vector operator-(const Vector& other) const;
    Vector operator*(double scalar) const;
    double dot(const Vector& other) const;
};

class Matrix {
private:
    std::vector<std::vector<double>> data_;
    std::size_t rows_;
    std::size_t cols_;

public:
    Matrix(std::size_t r = 0, std::size_t c = 0);

    std::size_t getRows() const;
    std::size_t getCols() const;

    std::vector<double>& operator[](std::size_t row);
    const std::vector<double>& operator[](std::size_t row) const;

    Matrix operator+(const Matrix& other) const;
    Matrix operator-(const Matrix& other) const;
    Matrix operator*(double scalar) const;
    Vector operator*(const Vector& vec) const;
};

#endif // MATRIX_VECTOR_H