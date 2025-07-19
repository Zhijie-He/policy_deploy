#pragma once
#include <wlrobot/idl/hd/IMUStatePubSubTypes.hpp>
#include <wlrobot/idl/hd/MotorStatesPubSubTypes.hpp>
#include <wlrobot/idl/hd/MotorCmdsPubSubTypes.hpp>
#include <wlrobot/idl/hd/LowStatePubSubTypes.hpp>

namespace wlrobot::robot {

// 默认模板（如果你用到未注册的类型，就报错）
template<typename T>
struct PubSubTypeResolver {
    static_assert(sizeof(T) == -1, "PubSubTypeResolver not defined for this type");
};

// 为 IMUState 提供类型支持
template<>
struct PubSubTypeResolver<wlrobot::msg::IMUState> {
    using type = wlrobot::msg::IMUStatePubSubType;
};

// 为 MotorStates 提供类型支持
template<>
struct PubSubTypeResolver<wlrobot::msg::MotorStates> {
    using type = wlrobot::msg::MotorStatesPubSubType;
};

// 为 MotorCmds 提供类型支持
template<>
struct PubSubTypeResolver<wlrobot::msg::MotorCmds> {
    using type = wlrobot::msg::MotorCmdsPubSubType;
};

// 为 LowState 提供类型支持
template<>
struct PubSubTypeResolver<wlrobot::msg::LowState> {
    using type = wlrobot::msg::LowStatePubSubType;
};
}

