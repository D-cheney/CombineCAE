#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

enum class LogLevel {
    Debug,
    Info,
    Warning,
    Error,
    Fatal
};

enum class LogOutput {
    Console,
    File,
    Both
};

struct LogEntry {
    LogLevel level;
    std::string message;
    std::string timestamp;
    std::string source;

    LogEntry() : level(LogLevel::Info) {}
    LogEntry(LogLevel l, const std::string& msg, const std::string& src = "")
        : level(l), message(msg), source(src) {}
};

class Logger {
private:
    LogLevel minLevel;
    LogOutput output;
    std::string logFile;
    std::ofstream fileStream;
    bool showTimestamp;
    bool showSource;

    std::vector<LogEntry> logHistory;

public:
    Logger();
    ~Logger();

    void setMinLevel(LogLevel level) { minLevel = level; }
    void setOutput(LogOutput out) { output = out; }
    void setLogFile(const std::string& filename);
    void setShowTimestamp(bool show) { showTimestamp = show; }
    void setShowSource(bool show) { showSource = show; }

    // 日志方法
    void debug(const std::string& message, const std::string& source = "");
    void info(const std::string& message, const std::string& source = "");
    void warning(const std::string& message, const std::string& source = "");
    void error(const std::string& message, const std::string& source = "");
    void fatal(const std::string& message, const std::string& source = "");

    // 求解器专用日志
    void logIteration(int iteration, double residual, const std::string& solverName = "");
    void logConvergence(int totalIterations, bool converged, const std::string& solverName = "");
    void logTimeStep(double time, double dt, const std::string& phase = "");
    void logConservationError(double massError, double energyError);

    // 获取日志历史
    const std::vector<LogEntry>& getLogHistory() const { return logHistory; }
    void clearLogHistory() { logHistory.clear(); }

    // 输出残差历史表
    void printResidualTable(const std::vector<double>& residuals,
                            const std::string& solverName = "");

private:
    void log(LogLevel level, const std::string& message, const std::string& source = "");
    std::string getTimestamp() const;
    std::string levelToString(LogLevel level) const;
    void writeToFile(const LogEntry& entry);
    void writeToConsole(const LogEntry& entry);
};

// 全局日志实例
Logger& getLogger();

#endif // LOGGER_H
