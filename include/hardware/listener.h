#pragma once

#include <atomic>

class Listener{
  public:
    Listener() = default; // 编译器自动生成空构造函数
    void listenKeyboard();
    void stop() { isRunning_.store(false); }  // 外部调用安全停止
    bool isRunning() const { return isRunning_.load(); }  // 外部只读判断

    char* getKeyInputPtr(){return &key_input_;} // 意味着你把 key_input_ 的 地址 直接暴露给了外部，外部可以通过指针 *p = 'w'; 直接 修改内部变量；
    // 这等于你给了外部一把钥匙，外部可以随时打开 Listener 的内部房间改值，完全破坏了封装性和线程安全性。

  private:
    std::atomic<bool> isRunning_{true};   // ✅ 保证读写线程安全
    char key_input_ = '\0'; 
};
