#include "logger.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

Logger::Logger()
    : minLevel(LogLevel::Info),
      output(LogOutput::Console),
      showTimestamp(true),
      showSource(false) {}

Logger::~Logger() {
    if (fileStream.is_open()) {
        fileStream.close();
    }
}

void Logger::setLogFile(const std::string& filename) {
    logFile = filename;
    if (fileStream.is_open()) {
        fileStream.close();
    }
    fileStream.open(filename, std::ios::app);
    if (fileStream.is_open()) {
        output = LogOutput::Both;
    }
}

void Logger::debug(const std::string& message, const std::string& source) {
    log(LogLevel::Debug, message, source);
}

void Logger::info(const std::string& message, const std::string& source) {
    log(LogLevel::Info, message, source);
}

void Logger::warning(const std::string& message, const std::string& source) {
    log(LogLevel::Warning, message, source);
}

void Logger::error(const std::string& message, const std::string& source) {
    log(LogLevel::Error, message, source);
}

void Logger::fatal(const std::string& message, const std::string& source) {
    log(LogLevel::Fatal, message, source);
}

void Logger::logIteration(int iteration, double residual, const std::string& solverName) {
    std::ostringstream oss;
    oss << "Iter " << iteration << ": residual = " << residual;
    if (!solverName.empty()) {
        oss << " [" << solverName << "]";
    }
    log(LogLevel::Debug, oss.str(), "Solver");
}

void Logger::logConvergence(int totalIterations, bool converged, const std::string& solverName) {
    std::ostringstream oss;
    if (converged) {
        oss << "Converged after " << totalIterations << " iterations";
    } else {
        oss << "Did not converge after " << totalIterations << " iterations";
    }
    if (!solverName.empty()) {
        oss << " [" << solverName << "]";
    }
    log(converged ? LogLevel::Info : LogLevel::Warning, oss.str(), "Solver");
}

void Logger::logTimeStep(double time, double dt, const std::string& phase) {
    std::ostringstream oss;
    oss << "Time: " << time << " s, dt = " << dt << " s";
    if (!phase.empty()) {
        oss << " [" << phase << "]";
    }
    log(LogLevel::Debug, oss.str(), "Transient");
}

void Logger::logConservationError(double massError, double energyError) {
    std::ostringstream oss;
    oss << "Conservation errors: mass=" << massError << ", energy=" << energyError;
    log(LogLevel::Debug, oss.str(), "Conservation");
}

void Logger::printResidualTable(const std::vector<double>& residuals,
                                const std::string& solverName) {
    if (residuals.empty()) {
        return;
    }

    std::ostringstream oss;
    oss << "\nResidual History";
    if (!solverName.empty()) {
        oss << " (" << solverName << ")";
    }
    oss << ":\n";
    oss << std::string(40, '-') << "\n";
    oss << std::setw(10) << "Iteration" << std::setw(15) << "Residual" << "\n";
    oss << std::string(40, '-') << "\n";

    for (std::size_t i = 0; i < residuals.size(); ++i) {
        oss << std::setw(10) << (i + 1) << std::setw(15) << residuals[i] << "\n";
    }
    oss << std::string(40, '-') << "\n";

    log(LogLevel::Info, oss.str(), "Solver");
}

void Logger::log(LogLevel level, const std::string& message, const std::string& source) {
    if (level < minLevel) {
        return;
    }

    LogEntry entry(level, message, source);
    entry.timestamp = getTimestamp();

    logHistory.push_back(entry);

    if (output == LogOutput::Console || output == LogOutput::Both) {
        writeToConsole(entry);
    }

    if (output == LogOutput::File || output == LogOutput::Both) {
        writeToFile(entry);
    }
}

std::string Logger::getTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S")
        << "." << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

std::string Logger::levelToString(LogLevel level) const {
    switch (level) {
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info: return "INFO";
        case LogLevel::Warning: return "WARNING";
        case LogLevel::Error: return "ERROR";
        case LogLevel::Fatal: return "FATAL";
        default: return "UNKNOWN";
    }
}

void Logger::writeToFile(const LogEntry& entry) {
    if (!fileStream.is_open()) {
        return;
    }

    fileStream << entry.timestamp << " [" << levelToString(entry.level) << "]";
    if (!entry.source.empty()) {
        fileStream << " [" << entry.source << "]";
    }
    fileStream << " " << entry.message << "\n";
    fileStream.flush();
}

void Logger::writeToConsole(const LogEntry& entry) {
    std::ostream& os = (entry.level >= LogLevel::Warning) ? std::cerr : std::cout;

    if (showTimestamp) {
        os << entry.timestamp << " ";
    }

    os << "[" << levelToString(entry.level) << "]";

    if (showSource && !entry.source.empty()) {
        os << " [" << entry.source << "]";
    }

    os << " " << entry.message << "\n";
}

Logger& getLogger() {
    static Logger instance;
    return instance;
}
