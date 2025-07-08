#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <iostream>
#include <chrono>

// 通用的消息队列类模板
template <typename T>
class CircularQueue {
public:
    // 构造函数，初始化最大容量
    CircularQueue(size_t maxSize = 10) : maxSize_(maxSize) {}

    // 推送数据到队列
    void push(std::shared_ptr<T> data) {
        std::unique_lock<std::mutex> lock(mutex_);

        // 队列已满时丢弃最老的元素
        if (queue_.size() >= maxSize_) {
            queue_.pop();  // 丢弃最老的元素
        }

        // 将新消息推送到队列
        queue_.push(data);

        // 唤醒消费者线程
        cvEmpty_.notify_one();
    }

    // 从队列中获取数据，支持最大阻塞时间
    std::shared_ptr<T> pop(std::chrono::milliseconds timeout) {         //timeout单位为ms
        std::unique_lock<std::mutex> lock(mutex_);

        // 阻塞直到队列不为空或超时
        if (!cvEmpty_.wait_for(lock, timeout, [this]() { return !queue_.empty(); })) {
            // 超时返回空指针
            return nullptr;
        }

        auto message = queue_.front();
        queue_.pop();
        
        return message;
    }

private:
    std::queue<std::shared_ptr<T>> queue_;  // 使用 std::queue 来存储数据
    size_t maxSize_;  // 队列最大容量
    std::mutex mutex_;
    std::condition_variable cvEmpty_; // 用于队列不为空时通知消费者
};


