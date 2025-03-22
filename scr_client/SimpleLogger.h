#ifndef LOG_RECORDER_H
#define LOG_RECORDER_H

#include <string>
#include <fstream>

// 定义日志级别枚举
enum class LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR
};

class LogRecorder {
private:
    std::ofstream fp;
    std::string getCurrentTime();

public:
    LogRecorder(const std::string& filename);
    void logMessage(LogLevel level, const std::string& file, int line, const std::string& message);
    ~LogRecorder();
};

#endif