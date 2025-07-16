#include "utility/tools.h"

namespace tools {
    std::shared_ptr<BaseRobotConfig> loadConfig(const std::string& config_name) {
        const std::string config_path = std::string(PROJECT_SOURCE_DIR) + "/config/" + config_name + ".yaml";
        if (config_name == "g1_eman")
            return std::make_shared<EmanRobotConfig>(config_path);
        else
            return std::make_shared<UnitreeRobotConfig>(config_path);
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

    // 输入四元数格式：[w, x, y, z]
    float getHeadingFromQuat(const Eigen::Vector4f& quat) {
        Eigen::Vector3f vec(1.0f, 0.0f, 0.0f);  // 初始方向向量 (x-axis)
        float w = quat[0];
        Eigen::Vector3f xyz = quat.tail<3>();  // 提取 x, y, z

        Eigen::Vector3f t = 2.0f * xyz.cross(vec);
        Eigen::Vector3f forward = vec + w * t + xyz.cross(t);

        float heading = std::atan2(forward[1], forward[0]);  // atan2(y, x)
        return heading;
    }

    // Eigen::MatrixXf compute_teleop_observation(const Eigen::MatrixXf& motion_body_pos,  // [T, 90]
    //                                            const Eigen::VectorXf& damping,          // [T-1]
    //                                            float dt) 
    // {
    //     // try to use torch rewrite
        
    //     int T = motion_body_pos.rows();      // 时间步数
    //     int D = motion_body_pos.cols();      // 90
    //     int J = D / 3;                       // 30

    //     // 输出
    //     Eigen::MatrixXf local_motion = Eigen::MatrixXf::Zero(T, D);

    //     // 1. 计算 root 起始位置（第0帧，第0个关键点的3D坐标）
    //     Eigen::Vector3f ref_root_start = motion_body_pos.block(0, 0, 1, 3).transpose();  // [3]

    //     // 2. 所有位置减去 ref_root_start，实现相对坐标系（只对 pos[t][j]）
    //     for (int t = 0; t < T; ++t) {
    //         for (int j = 0; j < J; ++j) {
    //             for (int d = 0; d < 3; ++d) {
    //                 local_motion(t, j * 3 + d) = motion_body_pos(t, j * 3 + d) - ref_root_start(d);
    //             }
    //         }
    //     }

    //     // 3. 第1帧到最后一帧：计算速度并除以dt * 5
    //     for (int t = 1; t < T; ++t) {
    //         for (int j = 0; j < J; ++j) {
    //             for (int d = 0; d < 3; ++d) {
    //                 float vel = (motion_body_pos(t, j * 3 + d) - motion_body_pos(0, j * 3 + d)) / dt / 5.f;
    //                 local_motion(t, j * 3 + d) = vel;
    //             }
    //         }
    //     }

    //     // 4. 对于第1帧及以后，将 XY 平面除以 damping（不对 Z 做处理）
    //     for (int t = 1; t < T; ++t) {
    //         float damp = damping(t - 1);
    //         for (int j = 0; j < J; ++j) {
    //             local_motion(t, j * 3 + 0) /= damp;  // x
    //             local_motion(t, j * 3 + 1) /= damp;  // y
    //         }
    //     }

    //     // 5. 把第0帧的 root（第0个点）替换为第1帧的 root
    //     local_motion.row(0).segment(0, 3) = local_motion.row(1).segment(0, 3);

    //     return local_motion;  // shape: [T, 90]
    // }

    Eigen::MatrixXf compute_teleop_observation(const Eigen::MatrixXf& motion_body_pos,  // [T, 90]
                                               const Eigen::VectorXf& damping,          // [T-1]
                                               float dt)
    {
        int T = motion_body_pos.rows();   // 时间步
        int D = motion_body_pos.cols();   // 90
        int J = D / 3;                    // 30
        Eigen::MatrixXf local_motion = Eigen::MatrixXf::Zero(T, D);

        // 1. 替换 root（第 0 个关键点）为左右髋关节中点（关键点 4 和 5）
        Eigen::MatrixXf modified_body_pos = motion_body_pos;
        for (int t = 0; t < T; ++t) {
            for (int d = 0; d < 3; ++d) {
                float left = motion_body_pos(t, 4 * 3 + d);
                float right = motion_body_pos(t, 5 * 3 + d);
                float avg = 0.5f * (left + right);
                modified_body_pos(t, 0 * 3 + d) = avg;
            }
        }

        // 2. 计算 ref_root_start_pos = 第 0 帧的 root（关键点 0）的 3D 坐标
        Eigen::Vector3f ref_root_start = modified_body_pos.block(0, 0, 1, 3).transpose();

        // 3. 相对坐标系：每个点减去 ref_root_start（不含速度）
        for (int t = 0; t < T; ++t) {
            for (int j = 0; j < J; ++j) {
                for (int d = 0; d < 3; ++d) {
                    local_motion(t, j * 3 + d) = modified_body_pos(t, j * 3 + d) - ref_root_start(d);
                }
            }
        }

        // 4. 从第 1 帧开始，计算速度并除以 dt * 5
        for (int t = 1; t < T; ++t) {
            for (int j = 0; j < J; ++j) {
                for (int d = 0; d < 3; ++d) {
                    float vel = (modified_body_pos(t, j * 3 + d) - modified_body_pos(0, j * 3 + d)) / dt / 5.f;
                    local_motion(t, j * 3 + d) = vel;
                }
            }
        }

        // 5. 对第 1 帧及之后，XY 坐标除以 damping（Z 不变）
        for (int t = 1; t < T; ++t) {
            float damp = damping(t - 1);
            for (int j = 0; j < J; ++j) {
                local_motion(t, j * 3 + 0) /= damp;  // x
                local_motion(t, j * 3 + 1) /= damp;  // y
            }
        }

        // 6. 把第 0 帧的 root 替换为第 1 帧的 root（只修改第 0 点的 3D 坐标）
        local_motion.row(0).segment(0, 3) = local_motion.row(1).segment(0, 3);

        return local_motion;
    }

    Eigen::VectorXf resolveCompatibilityConcat(const Eigen::VectorXf& state, const Eigen::VectorXi& joint_concat_index){
        Eigen::VectorXf result(joint_concat_index.size());
        for (int i = 0; i < joint_concat_index.size(); ++i) {
            result(i) = state(joint_concat_index(i));
        }
        return result;
    }

    std::pair<Eigen::VectorXf, Eigen::VectorXf> resolveCompatibilitySplit(const Eigen::VectorXf& state, 
                                                                          const std::unordered_map<std::string, Eigen::VectorXi>& joint_split_index)
    {
        const auto& skeleton_index = joint_split_index.at("skeleton");
        const auto& hands_index    = joint_split_index.at("hands");

        Eigen::VectorXf skeleton(skeleton_index.size());
        Eigen::VectorXf hands(hands_index.size());

        for (int i = 0; i < skeleton_index.size(); ++i) {
            skeleton(i) = state(skeleton_index(i));
        }

        for (int i = 0; i < hands_index.size(); ++i) {
            hands(i) = state(hands_index(i));
        }

        return {skeleton, hands};
    }

    Eigen::VectorXf concatJointAndHand(const Eigen::VectorXf& joint,
                                       const Eigen::VectorXf& hands,
                                       int jointDim,
                                       int handsDim) {
        Eigen::VectorXf result(jointDim + handsDim);
        result.segment(0, jointDim) = joint;

        if (handsDim > 0 && hands.size() == handsDim) {
            result.segment(jointDim, handsDim) = hands;
        } 
        return result;
    }
}

// #include <torch/torch.h>
// #include <Eigen/Dense>

// Eigen::MatrixXf compute_teleop_observation(const Eigen::MatrixXf& motion_body_pos,  // [T, 90]
//                                            const Eigen::VectorXf& damping,          // [T-1]
//                                            float dt)
// {
//     int T = motion_body_pos.rows();
//     int D = motion_body_pos.cols();
//     int J = D / 3;

//     TORCH_CHECK(D == 90, "Expected D == 90");
//     TORCH_CHECK(damping.size() == T - 1, "Damping size mismatch");

//     // 1. Convert Eigen → torch::Tensor
//     torch::Tensor pos = torch::from_blob((void*)motion_body_pos.data(), {T, J, 3}, torch::TensorOptions().dtype(torch::kFloat32)).clone();
//     torch::Tensor damp = torch::from_blob((void*)damping.data(), {T - 1}, torch::TensorOptions().dtype(torch::kFloat32)).clone();

//     // 2. 替换 root 为左右髋中点（关节 4 和 5）
//     torch::Tensor root_avg = 0.5 * (pos.index({Slice(), 4}) + pos.index({Slice(), 5}));
//     pos.index_put_({Slice(), 0}, root_avg);

//     // 3. 相对坐标（减去第0帧 root）
//     torch::Tensor ref_root_start = pos.index({0, 0}).clone();  // [3]
//     torch::Tensor local = pos - ref_root_start.view({1, 1, 3});  // [T, J, 3]

//     // 4. 从第1帧开始计算速度
//     torch::Tensor motion_delta = (pos.index({Slice(1, None)}) - pos.index({0}).unsqueeze(0)) / dt / 5.f;
//     local.index_put_({Slice(1, None)}, motion_delta);

//     // 5. 只对 x/y 做 damping
//     local.index({Slice(1, None), Slice(), 0}) /= damp.view({T - 1, 1});  // x
//     local.index({Slice(1, None), Slice(), 1}) /= damp.view({T - 1, 1});  // y

//     // 6. 替换第0帧 root 为第1帧 root
//     local.index_put_({0, 0}, local.index({1, 0}));

//     // 7. 还原为 [T, 90]
//     torch::Tensor result = local.view({T, D});

//     // 8. 转回 Eigen（拷贝数据）
//     Eigen::MatrixXf final_result(T, D);
//     std::memcpy(final_result.data(), result.contiguous().data_ptr<float>(), sizeof(float) * T * D);
//     return final_result;
// }

