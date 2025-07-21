#pragma once

#include <fastdds/dds/subscriber/DataReader.hpp>
#include <wlrobot/robot/channel/channel_factory.hpp>
#include <wlrobot/robot/channel/pubsub_type_resolver.hpp>

namespace wlrobot::robot {

template<typename T>
class ChannelSubscriber {
public:
    using Callback = std::function<void(const T&)>;

    explicit ChannelSubscriber(const std::string& channel_name)
        : channel_name_(channel_name) {}

    void InitChannel(Callback cb, int queue_len = 1) {
        reader_ = ChannelFactory::Instance()->CreateRecvChannel<T>(channel_name_, cb, queue_len);
    }

    void CloseChannel() {
        reader_ = nullptr;
    }

    const std::string& GetChannelName() const { return channel_name_; }

private:
    std::string channel_name_;
    eprosima::fastdds::dds::DataReader* reader_ = nullptr;
};

template<typename T>
using ChannelSubscriberPtr = std::shared_ptr<ChannelSubscriber<T>>;

} // namespace wlrobot::robot

