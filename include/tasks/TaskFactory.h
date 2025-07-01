// tasks/TaskFactory.h
#pragma once
#include <unordered_map>
#include <string>
#include <memory>
#include <functional>
#include "tasks/BaseTask.h"

class TaskFactory {
public:
    static bool exists(const std::string& name);
    static std::shared_ptr<BaseTask> create(const std::string& name, float control_dt, torch::Device device);
    static void registerTask(const std::string& name,
        const std::function<std::shared_ptr<BaseTask>(float, torch::Device)>& ctor);
static std::vector<std::string> getAvailableTaskNames();

private:
    static std::unordered_map<std::string, std::function<std::shared_ptr<BaseTask>(float, torch::Device)>>& registry();
};
