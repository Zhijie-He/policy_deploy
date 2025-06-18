// data_buffer.h
// #pragma once
// #include <memory>
// #include <mutex>
// #include <shared_mutex>

// template <typename T>
// class DataBuffer { // 在多线程程序中，用于安全地在一个线程写入数据，另一个线程读取数据，避免数据竞争（race condition）。
//  public:
//   void SetData(const T &newData) {
//     // 这样可以允许多个读操作并发，但写操作必须独占
//     std::unique_lock<std::shared_mutex> lock(mutex); // unique_lock → 独占锁（用于写）
//     data = std::make_shared<T>(newData);
//   }

//   std::shared_ptr<const T> GetData() {
//     std::shared_lock<std::shared_mutex> lock(mutex); // shared_lock → 共享锁（用于读）
//     return data ? data : nullptr;
//   }



//   void Clear() {
//     std::unique_lock<std::shared_mutex> lock(mutex); 
//     data = nullptr;
//   }

//  private:
//   std::shared_ptr<T> data; // data：缓存在里面的数据，使用智能指针托管
//   std::shared_mutex mutex; // mutex：使用 C++17 的 std::shared_mutex 支持读写分离
// };
// data_buffer.h
#pragma once
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <atomic>

template <typename T>
class DataBuffer {
 public:
  DataBuffer() : new_data_flag_(false) {}

  // 写入新数据（线程安全）
  void SetData(const T& newData) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    data_ = std::make_shared<T>(newData);
    new_data_flag_ = true;
  }

  // 获取共享指针（适用于轻量访问）
  std::shared_ptr<const T> GetData() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return data_;
  }

  // 获取副本（适用于控制逻辑）
  T GetCopy() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return data_ ? *data_ : T{};
  }

  // 是否已设置数据
  bool HasData() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return static_cast<bool>(data_);
  }

  // 是否有新数据（读取后清除）
  bool HasNewData() {
    return new_data_flag_.exchange(false);  // 原子判断 + 清除
  }

  // 清空数据
  void Clear() {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    data_.reset();
    new_data_flag_ = false;
  }

 private:
  std::shared_ptr<T> data_;                    // 数据内容
  mutable std::shared_mutex mutex_;            // 读写锁
  std::atomic_bool new_data_flag_{false};      // 是否有新数据
};

