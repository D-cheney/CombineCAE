#include "matrix_vector.h"

#include <stdexcept>

Vector::Vector(std::size_t size) : data_(size, 0.0) {}

Vector::Vector(const std::vector<double>& vec) : data_(vec) {}

std::size_t Vector::size() const {
    return data_.size();
}

double& Vector::operator[](std::size_t index) {
    return data_.at(index);
}

const double& Vector::operator[](std::size_t index) const {
    return data_.at(index);
}

Vector Vector::operator+(const Vector& other) const {
    if (size() != other.size()) {
        throw std::invalid_argument("Vector sizes must match for addition.");
    }

    Vector result(size());
    for (std::size_t i = 0; i < size(); ++i) {
        result[i] = data_[i] + other.data_[i];
    }
    return result;
}

Vector Vector::operator-(const Vector& other) const {
    if (size() != other.size()) {
        throw std::invalid_argument("Vector sizes must match for subtraction.");
    }

    Vector result(size());
    for (std::size_t i = 0; i < size(); ++i) {
        result[i] = data_[i] - other.data_[i];
    }
    return result;
}

Vector Vector::operator*(double scalar) const {
    Vector result(size());
    for (std::size_t i = 0; i < size(); ++i) {
        result[i] = data_[i] * scalar;
    }
    return result;
}

double Vector::dot(const Vector& other) const {
    if (size() != other.size()) {
        throw std::invalid_argument("Vector sizes must match for dot product.");
    }

    double result = 0.0;
    for (std::size_t i = 0; i < size(); ++i) {
        result += data_[i] * other.data_[i];
    }
    return result;
}

Matrix::Matrix(std::size_t r, std::size_t c) : rows_(r), cols_(c) {
    data_.assign(rows_, std::vector<double>(cols_, 0.0));
}

std::size_t Matrix::getRows() const {
    return rows_;
}

std::size_t Matrix::getCols() const {
    return cols_;
}

std::vector<double>& Matrix::operator[](std::size_t row) {
    return data_.at(row);
}

const std::vector<double>& Matrix::operator[](std::size_t row) const {
    return data_.at(row);
}

Matrix Matrix::operator+(const Matrix& other) const {
    if (rows_ != other.rows_ || cols_ != other.cols_) {
        throw std::invalid_argument("Matrix dimensions must match for addition.");
    }

    Matrix result(rows_, cols_);
    for (std::size_t i = 0; i < rows_; ++i) {
        for (std::size_t j = 0; j < cols_; ++j) {
            result[i][j] = data_[i][j] + other.data_[i][j];
        }
    }
    return result;
}

Matrix Matrix::operator-(const Matrix& other) const {
    if (rows_ != other.rows_ || cols_ != other.cols_) {
        throw std::invalid_argument("Matrix dimensions must match for subtraction.");
    }

    Matrix result(rows_, cols_);
    for (std::size_t i = 0; i < rows_; ++i) {
        for (std::size_t j = 0; j < cols_; ++j) {
            result[i][j] = data_[i][j] - other.data_[i][j];
        }
    }
    return result;
}

Matrix Matrix::operator*(double scalar) const {
    Matrix result(rows_, cols_);
    for (std::size_t i = 0; i < rows_; ++i) {
        for (std::size_t j = 0; j < cols_; ++j) {
            result[i][j] = data_[i][j] * scalar;
        }
    }
    return result;
}

Vector Matrix::operator*(const Vector& vec) const {
    if (cols_ != vec.size()) {
        throw std::invalid_argument("Matrix columns must match vector size.");
    }

    Vector result(rows_);
    for (std::size_t i = 0; i < rows_; ++i) {
        double sum = 0.0;
        for (std::size_t j = 0; j < cols_; ++j) {
            sum += data_[i][j] * vec[j];
        }
        result[i] = sum;
    }
    return result;
}