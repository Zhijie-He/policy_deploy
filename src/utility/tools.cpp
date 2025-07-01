#include "utility/tools.h"

namespace tools {
    std::shared_ptr<BaseRobotConfig> loadConfig(const std::string& config_name) {
        const std::string config_path = std::string(PROJECT_SOURCE_DIR) + "/config/" + config_name + ".yaml";
        if (config_name == "g1_eman")
            return std::make_shared<EmanRobotConfig>(config_path);
        else
            return std::make_shared<UnitreeRobotConfig>(config_path);
    }

    std::unique_ptr<BasePolicyWrapper> loadPolicyWrapper(const std::string& config_name, 
                                                         std::shared_ptr<const BaseRobotConfig> cfg, 
                                                         torch::Device device,
                                                         const std::string& inference_engine_type,
                                                         const std::string& precision)
    {
        if (config_name == "g1_eman")
            return std::make_unique<EmanPolicyWrapper>(cfg, device, inference_engine_type, precision); 
        else
            return std::make_unique<UnitreePolicyWrapper>(cfg, device, inference_engine_type, precision);
    }

    Eigen::VectorXf pd_control(const Eigen::VectorXf& target_q,
                                  const Eigen::VectorXf& q,
                                  const Eigen::VectorXf& kp,
                                  const Eigen::VectorXf& target_dq,
                                  const Eigen::VectorXf& dq,
                                  const Eigen::VectorXf& kd) {
    return (target_q - q).cwiseProduct(kp) + (target_dq - dq).cwiseProduct(kd);
    }

    void checkMujucoVersion(){
        if (mjVERSION_HEADER != mj_version()) {  // Mujoco版本检查
            FRC_ERROR("[checkMujucoVersion] MuJoCo header and library version mismatch!");
            throw std::runtime_error("MuJoCo header and library version mismatch!");
        }
        FRC_INFO("[checkMujucoVersion] Using MuJoCo("<< mj_version() << ")!");
    }

    torch::Device getDefaultDevice() {
        #if defined(__APPLE__)
            FRC_INFO("[getDefaultDevice] Running on macOS. Using CPU.");
            return torch::kCPU;
        #elif defined(_WIN32) || defined(__linux__)
            if (torch::cuda::is_available()) {
                FRC_INFO("[getDefaultDevice] CUDA available. Using GPU.");
                return torch::kCUDA;
            } else {
                FRC_INFO("[getDefaultDevice] CUDA not available. Using CPU.");
                return torch::kCPU;
            }
        #else
            FRC_WARN("[getDefaultDevice] Unknown platform. Defaulting to CPU.");
            return torch::kCPU;
        #endif
    }

    torch::Dtype parseDtype(const std::string& precision){
        if(precision == "fp32") return torch::kFloat32;
        if(precision == "fp16") return torch::kFloat16;
        if(precision == "int8") return torch::kInt8;
        throw std::invalid_argument("Unknown dtype string: " + precision);
    }


    Eigen::Vector3f get_gravity_orientation(const Eigen::Vector4f& q) {
        float qw = q[0], qx = q[1], qy = q[2], qz = q[3];

        Eigen::Vector3f g;
        g[0] = 2 * (-qz * qx + qw * qy);
        g[1] = -2 * (qz * qy + qw * qx);
        g[2] = 1 - 2 * (qw * qw + qz * qz);
        return g;
    }

    Eigen::Vector3f quat_rotate_inverse_on_gravity(const Eigen::Vector4f& q) {
        Eigen::Vector3f v(0.0f, 0.0f, -1.0f);  // world gravity vector

        float qw = q[0];
        Eigen::Vector3f q_vec = q.tail<3>();

        Eigen::Vector3f a = v * (2.0f * qw * qw - 1.0f);
        Eigen::Vector3f b = 2.0f * qw * q_vec.cross(v);
        Eigen::Vector3f c = 2.0f * q_vec * (q_vec.dot(v));

        return a - b + c;
    }


}

