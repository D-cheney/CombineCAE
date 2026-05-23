#include "error_handler.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <set>

ErrorHandler::ErrorHandler()
    : throwOnCritical(false) {}

void ErrorHandler::reportError(ErrorCode code, ErrorSeverity severity,
                               const std::string& message,
                               const std::string& details,
                               int nodeId, int componentIndex) {
    ErrorInfo info;
    info.code = code;
    info.severity = severity;
    info.message = message;
    info.details = details;
    info.nodeId = nodeId;
    info.componentIndex = componentIndex;

    errorHistory.push_back(info);
    notifyCallback(info);

    // 输出到控制台
    const char* severityStr = "";
    switch (severity) {
        case ErrorSeverity::Info: severityStr = "INFO"; break;
        case ErrorSeverity::Warning: severityStr = "WARNING"; break;
        case ErrorSeverity::Error: severityStr = "ERROR"; break;
        case ErrorSeverity::Critical: severityStr = "CRITICAL"; break;
    }

    std::cerr << "[" << severityStr << "] " << message;
    if (nodeId >= 0) {
        std::cerr << " (Node " << nodeId << ")";
    }
    if (componentIndex >= 0) {
        std::cerr << " (Component #" << componentIndex << ")";
    }
    std::cerr << std::endl;

    if (!details.empty()) {
        std::cerr << "  Details: " << details << std::endl;
    }

    if (severity == ErrorSeverity::Critical && throwOnCritical) {
        throw std::runtime_error("Critical error: " + message);
    }
}

void ErrorHandler::reportWarning(const std::string& message, int nodeId) {
    reportError(ErrorCode::UnknownError, ErrorSeverity::Warning, message, "", nodeId);
}

void ErrorHandler::reportError(const std::string& message, int nodeId) {
    reportError(ErrorCode::UnknownError, ErrorSeverity::Error, message, "", nodeId);
}

void ErrorHandler::reportCritical(const std::string& message, int nodeId) {
    reportError(ErrorCode::UnknownError, ErrorSeverity::Critical, message, "", nodeId);
}

bool ErrorHandler::checkNetworkConnectivity(const std::vector<int>& nodeIds,
                                            const std::vector<std::pair<int, int>>& connections,
                                            std::vector<int>& isolatedNodes) {
    isolatedNodes.clear();

    if (nodeIds.empty()) {
        return true;
    }

    // 构建邻接表
    std::map<int, std::vector<int>> adjacency;
    for (int nodeId : nodeIds) {
        adjacency[nodeId] = std::vector<int>();
    }

    for (const auto& conn : connections) {
        adjacency[conn.first].push_back(conn.second);
        adjacency[conn.second].push_back(conn.first);
    }

    // BFS检查连通性
    std::set<int> visited;
    std::queue<int> queue;

    // 从第一个节点开始
    int startNode = nodeIds[0];
    queue.push(startNode);
    visited.insert(startNode);

    while (!queue.empty()) {
        int current = queue.front();
        queue.pop();

        for (int neighbor : adjacency[current]) {
            if (visited.find(neighbor) == visited.end()) {
                visited.insert(neighbor);
                queue.push(neighbor);
            }
        }
    }

    // 检查是否有未访问的节点
    for (int nodeId : nodeIds) {
        if (visited.find(nodeId) == visited.end()) {
            isolatedNodes.push_back(nodeId);
        }
    }

    if (!isolatedNodes.empty()) {
        reportError(ErrorCode::NetworkNotConnected, ErrorSeverity::Error,
                    "Network is not connected",
                    "Isolated nodes found: " + std::to_string(isolatedNodes.size()));
        return false;
    }

    return true;
}

bool ErrorHandler::checkMatrixCondition(const std::vector<std::vector<double>>& matrix,
                                        double& conditionNumber) {
    if (matrix.empty() || matrix[0].empty()) {
        conditionNumber = 0.0;
        return true;
    }

    std::size_t n = matrix.size();

    // 计算矩阵的Frobenius范数
    double frobeniusNorm = 0.0;
    for (const auto& row : matrix) {
        for (double val : row) {
            frobeniusNorm += val * val;
        }
    }
    frobeniusNorm = std::sqrt(frobeniusNorm);

    // 简化的条件数估计
    // 实际应用中应该使用SVD或其他更精确的方法
    double maxDiag = 0.0;
    double minDiag = 1e10;

    for (std::size_t i = 0; i < n && i < matrix[i].size(); ++i) {
        double diagVal = std::abs(matrix[i][i]);
        maxDiag = std::max(maxDiag, diagVal);
        if (diagVal > 1e-12) {
            minDiag = std::min(minDiag, diagVal);
        }
    }

    if (minDiag < 1e-12) {
        conditionNumber = 1e12; // 矩阵奇异
        reportError(ErrorCode::SingularMatrix, ErrorSeverity::Critical,
                    "Matrix is singular or near-singular",
                    "Min diagonal element: " + std::to_string(minDiag));
        return false;
    }

    conditionNumber = maxDiag / minDiag;

    if (conditionNumber > 1e6) {
        reportError(ErrorCode::MatrixIllConditioned, ErrorSeverity::Warning,
                    "Matrix is ill-conditioned",
                    "Condition number: " + std::to_string(conditionNumber));
    }

    return true;
}

bool ErrorHandler::hasErrors() const {
    for (const auto& error : errorHistory) {
        if (error.severity == ErrorSeverity::Error ||
            error.severity == ErrorSeverity::Critical) {
            return true;
        }
    }
    return false;
}

bool ErrorHandler::hasCriticalErrors() const {
    for (const auto& error : errorHistory) {
        if (error.severity == ErrorSeverity::Critical) {
            return true;
        }
    }
    return false;
}

void ErrorHandler::notifyCallback(const ErrorInfo& info) {
    if (callback) {
        callback(info);
    }
}
