#include "SimpleLogger.h"
#include <ctime>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <chrono>

// 获取当前时间的实现
// 获取当前时刻的字符串，格式为 2025-03-06 15:34:42.670
std::string LogRecorder::getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << now_ms.count();
    return ss.str();
}

// 构造函数的实现
LogRecorder::LogRecorder(const std::string& filename) {
    fp.open(filename, std::ios::trunc);
    if (!fp.is_open()) {
        std::cerr << "无法打开文件用于写入。" << std::endl;
    }
    fp.close();
    fp.open(filename, std::ios::app);
    if (!fp.is_open()) {
        std::cerr << "无法打开文件用于追加。" << std::endl;
    }
}

// 记录日志的函数，向文件中追加内容，并加上当前时刻、日志级别、文件名和行号
void LogRecorder::logMessage(LogLevel level, const std::string& file, int line, const std::string& message) {
    if (fp.is_open()) {
        std::string levelStr;
        switch (level) {
            case LogLevel::DEBUG:
                levelStr = "DEBUG";
                break;
            case LogLevel::INFO:
                levelStr = "INFO";
                break;
            case LogLevel::WARNING:
                levelStr = "WARNING";
                break;
            case LogLevel::ERROR:
                levelStr = "ERROR";
                break;
        }
        std::string timeStr = getCurrentTime();
        fp << "" << timeStr << " " << levelStr << " <" << file << ":" << line << "> " << message << std::endl;
    }
}

// 析构函数的实现
LogRecorder::~LogRecorder() {
    if (fp.is_open()) {
        fp.close();
    }
}