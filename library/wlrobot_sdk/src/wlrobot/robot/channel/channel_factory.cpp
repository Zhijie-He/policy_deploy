#include <wlrobot/robot/channel/channel_factory.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

using namespace eprosima::fastdds::dds;

namespace wlrobot::robot {

// 静态成员定义
std::shared_ptr<DomainParticipant> ChannelFactory::participant_ = nullptr;
std::shared_ptr<eprosima::fastdds::dds::Publisher> ChannelFactory::publisher_ = nullptr;
std::shared_ptr<eprosima::fastdds::dds::Subscriber> ChannelFactory::subscriber_ = nullptr;

bool ChannelFactory::Init(uint32_t domain_id, const std::string& nic) {
    DomainParticipantQos participant_qos;
    participant_qos.name("wlrobot_channel_factory_participant");

    // 创建 participant
    participant_ = std::shared_ptr<DomainParticipant>(
        DomainParticipantFactory::get_instance()->create_participant(domain_id, participant_qos),
        [](DomainParticipant* p) {
            if (p) DomainParticipantFactory::get_instance()->delete_participant(p);
        });

    if (!participant_) return false;

    // 创建 publisher
    publisher_ = std::shared_ptr<eprosima::fastdds::dds::Publisher>(
        participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr),
        [participant = participant_.get()](eprosima::fastdds::dds::Publisher* pub) {
            if (participant && pub) participant->delete_publisher(pub);
        });

    // 创建 subscriber
    subscriber_ = std::shared_ptr<eprosima::fastdds::dds::Subscriber>(
        participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr),
        [participant = participant_.get()](eprosima::fastdds::dds::Subscriber* sub) {
            if (participant && sub) participant->delete_subscriber(sub);
        });

    return publisher_ && subscriber_;
}

}  // namespace wlrobot::robot

