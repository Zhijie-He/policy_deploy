// tasks/TaskFactory.h
#pragma once
#include <unordered_map>
#include <string>
#include <memory>
#include <functional>
#include "tasks/BaseTask.h"
#include "config/BaseRobotConfig.h" 

class TaskFactory {
public:
    static bool exists(const std::string& name);
    static std::shared_ptr<BaseTask> create(
        const std::string& name,
        std::shared_ptr<const BaseRobotConfig> cfg,
        torch::Device device,
        const std::string& hands_type,
        const std::string& inference_engine_type,
        const std::string& precision);

   static void registerTask(
        const std::string& name,
        const std::function<std::shared_ptr<BaseTask>(
            std::shared_ptr<const BaseRobotConfig>,
            torch::Device,
            std::string,    // hands type
            std::string,    // inference_engine_type
            std::string     // precision
        )>& ctor);
    
        static std::vector<std::string> getAvailableTaskNames();

private:
    static std::unordered_map<std::string,std::function<std::shared_ptr<BaseTask>(
            std::shared_ptr<const BaseRobotConfig>,
            torch::Device,
            std::string,
            std::string,
            std::string
        )>>& registry();
};
