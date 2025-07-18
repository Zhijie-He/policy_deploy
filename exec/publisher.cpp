#include <iostream>
#include <thread>
#include <chrono>
#include <array>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/core/status/PublicationMatchedStatus.hpp>

#include "wlrobot_dds/hd/IMUStatePubSubTypes.hpp"
#include "wlrobot_dds/hd/MotorStatesPubSubTypes.hpp"
using namespace eprosima::fastdds::dds;
using namespace wlrobot::msg;

int main() {
    // 创建 participant
    DomainParticipantQos participant_qos;
    participant_qos.name("MockPublisherParticipant");
    DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(1, participant_qos);
    if (!participant) { std::cerr << "Failed to create participant\n"; return 1; }

    // 注册类型
    TypeSupport imuType(new IMUStatePubSubType());
    TypeSupport motorType(new MotorStatesPubSubType());
    imuType.register_type(participant);
    motorType.register_type(participant);

    // 创建 topic
    Topic* imuTopic = participant->create_topic("/low_level/imu/state", "wlrobot::msg::IMUState", TOPIC_QOS_DEFAULT);
    Topic* motorTopic = participant->create_topic("/low_level/motor/state", "wlrobot::msg::MotorStates", TOPIC_QOS_DEFAULT);
    if (!imuTopic || !motorTopic) { std::cerr << "Failed to create topic\n"; return 1; }

    // 创建 publisher 和 writer
    Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
    DataWriter* imuWriter = publisher->create_datawriter(imuTopic, DATAWRITER_QOS_DEFAULT, nullptr);
    DataWriter* motorWriter = publisher->create_datawriter(motorTopic, DATAWRITER_QOS_DEFAULT, nullptr);
    if (!imuWriter || !motorWriter) { std::cerr << "Failed to create writer\n"; return 1; }

    std::cout << "Start publishing fake IMU and MotorStates data..." << std::endl;

    int64_t timestamp = 0;
    while (true) {
        // 构造假 IMU 数据
        IMUState imu;
        imu.timestamp(timestamp++);
        imu.temperature(36.5f);
        std::array<float, 3> accel = {0.0f, 0.0f, 9.8f};
        std::array<float, 3> gyro = {0.01f, 0.02f, 0.03f};
        std::array<float, 4> quat = {1.0f, 0.0f, 0.0f, 0.0f};
        std::array<float, 3> rpy = {0.1f, 0.2f, 0.3f};
        imu.accelerometer(accel);
        imu.gyroscope(gyro);
        imu.quaternion(quat);
        imu.rpy(rpy);

        // 构造假 MotorStates 数据
        MotorStates motors;
        std::array<float, 12> pos = {0.1f, 0.2f, 0.3f, 0.4f};
        std::array<float, 12> vel = {0.01f, 0.02f, 0.03f, 0.04f};
        motors.pos(pos);
        motors.w(vel);

        imuWriter->write(&imu);
        motorWriter->write(&motors);

        std::cout << "[MockPub] Published timestamp = " << timestamp << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}

