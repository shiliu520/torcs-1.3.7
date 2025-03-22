#include "SimpleLogger.h"
#include <iostream>

#define LOG(logger, level, message) logger.logMessage(level, __FILE__, __LINE__, message)

int main() {
    // 创建 LogRecorder 对象，指定日志文件名
    LogRecorder logger("app.log");

    // 记录日志消息
    LOG(logger, LogLevel::INFO, "这是一条信息级别的日志。");
    LOG(logger, LogLevel::WARNING, "这是一条警告级别的日志。");
    LOG(logger, LogLevel::ERROR, "这是一条错误级别的日志。");

    return 0;
}
// g++ -std=c++11 SimpleLogger.cpp SimpleLoggerTest.cpp -o app && ./app
