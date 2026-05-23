#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H

#include <functional>
#include <string>
#include <vector>

enum class ErrorSeverity {
    Info,
    Warning,
    Error,
    Critical
};

enum class ErrorCode {
    // 网络拓扑错误
    NetworkNotConnected,
    IsolatedNode,
    MissingComponent,

    // 矩阵求解错误
    SingularMatrix,
    MatrixIllConditioned,
    SolverDidNotConverge,

    // 时间积分错误
    TimeStepTooSmall,
    CFLViolation,
    ConservationViolation,

    // 耦合错误
    CouplingDidNotConverge,
    InterfaceDataMismatch,

    // 组件错误
    InvalidComponentParameters,
    ComponentNotConnected,

    // 通用错误
    UnknownError
};

struct ErrorInfo {
    ErrorCode code;
    ErrorSeverity severity;
    std::string message;
    std::string details;
    int nodeId;
    int componentIndex;

    ErrorInfo()
        : code(ErrorCode::UnknownError),
          severity(ErrorSeverity::Error),
          nodeId(-1),
          componentIndex(-1) {}
};

using ErrorCallback = std::function<void(const ErrorInfo&)>;

class ErrorHandler {
private:
    std::vector<ErrorInfo> errorHistory;
    ErrorCallback callback;
    bool throwOnCritical;

public:
    ErrorHandler();

    void setErrorCallback(ErrorCallback cb) { callback = cb; }
    void setThrowOnCritical(bool enable) { throwOnCritical = enable; }

    // 报告错误
    void reportError(ErrorCode code, ErrorSeverity severity,
                     const std::string& message,
                     const std::string& details = "",
                     int nodeId = -1, int componentIndex = -1);

    // 便捷方法
    void reportWarning(const std::string& message, int nodeId = -1);
    void reportError(const std::string& message, int nodeId = -1);
    void reportCritical(const std::string& message, int nodeId = -1);

    // 网络拓扑诊断
    bool checkNetworkConnectivity(const std::vector<int>& nodeIds,
                                  const std::vector<std::pair<int, int>>& connections,
                                  std::vector<int>& isolatedNodes);

    // 矩阵诊断
    bool checkMatrixCondition(const std::vector<std::vector<double>>& matrix,
                              double& conditionNumber);

    // 获取错误历史
    const std::vector<ErrorInfo>& getErrorHistory() const { return errorHistory; }
    void clearErrorHistory() { errorHistory.clear(); }

    // 检查是否有错误
    bool hasErrors() const;
    bool hasCriticalErrors() const;

private:
    void notifyCallback(const ErrorInfo& info);
};

#endif // ERROR_HANDLER_H
