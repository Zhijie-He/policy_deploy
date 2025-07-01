// tasks/TaskFactory.cpp
#include "tasks/TaskFactory.h"


bool TaskFactory::exists(const std::string& name) {
    return registry().count(name) > 0;
}

std::shared_ptr<BaseTask> TaskFactory::create(const std::string& name, std::shared_ptr<const BaseRobotConfig> cfg, torch::Device device) {
    if (!exists(name)) {
        throw std::runtime_error("[TaskFactory] Task not registered: " + name);
    }
    return registry()[name](cfg, device);
}

void TaskFactory::registerTask(const std::string& name,
    const std::function<std::shared_ptr<BaseTask>(std::shared_ptr<const BaseRobotConfig>, torch::Device)>& ctor) {
    registry()[name] = ctor;
}

std::unordered_map<std::string, std::function<std::shared_ptr<BaseTask>(std::shared_ptr<const BaseRobotConfig>, torch::Device)>>& TaskFactory::registry() {
    static std::unordered_map<std::string, std::function<std::shared_ptr<BaseTask>(std::shared_ptr<const BaseRobotConfig>, torch::Device)>> instance;
    return instance;
}

std::vector<std::string> TaskFactory::getAvailableTaskNames() {
    std::vector<std::string> names;
    for (const auto& [name, _] : registry()) {
        names.push_back(name);
    }
    return names;
}

