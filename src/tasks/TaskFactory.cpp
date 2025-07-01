// tasks/TaskFactory.cpp
#include "tasks/TaskFactory.h"


bool TaskFactory::exists(const std::string& name) {
    return registry().count(name) > 0;
}

std::shared_ptr<BaseTask> TaskFactory::create(
    const std::string& name,
    std::shared_ptr<const BaseRobotConfig> cfg,
    torch::Device device,
    const std::string& inference_engine_type,
    const std::string& precision) 
{
    if (!exists(name)) {
        throw std::runtime_error("[TaskFactory] Task not registered: " + name);
    }
    return registry()[name](cfg, device, inference_engine_type, precision);
}

void TaskFactory::registerTask(
    const std::string& name,
    const std::function<std::shared_ptr<BaseTask>(
        std::shared_ptr<const BaseRobotConfig>,
        torch::Device,
        std::string,   // inference_engine_type
        std::string    // precision
    )>& ctor) 
{
    registry()[name] = ctor;
}

std::unordered_map<std::string, std::function<std::shared_ptr<BaseTask>(std::shared_ptr<const BaseRobotConfig>, torch::Device, std::string, std::string)>> &TaskFactory::registry() {
    static std::unordered_map<std::string,
    std::function<std::shared_ptr<BaseTask>(
        std::shared_ptr<const BaseRobotConfig>,
        torch::Device,
        std::string,
        std::string)>> instance;
    return instance;
}

std::vector<std::string> TaskFactory::getAvailableTaskNames() {
    std::vector<std::string> names;
    for (const auto& [name, _] : registry()) {
        names.push_back(name);
    }
    return names;
}

