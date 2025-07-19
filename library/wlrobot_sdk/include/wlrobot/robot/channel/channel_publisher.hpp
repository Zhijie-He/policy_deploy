#pragma once

#include <fastdds/dds/publisher/DataWriter.hpp>
#include <wlrobot/robot/channel/channel_factory.hpp>
#include <wlrobot/robot/channel/pubsub_type_resolver.hpp>

namespace wlrobot::robot {

template<typename T>
class ChannelPublisher {
public:
    explicit ChannelPublisher(const std::string& channel_name)
        : channel_name_(channel_name) {}

    void InitChannel() {
        writer_ = ChannelFactory::Instance()->CreateSendChannel<T>(channel_name_);
    }

    bool Write(const T& msg) {
        return writer_ && writer_->write((void*)&msg);
    }

    void CloseChannel() {
        writer_ = nullptr;
    }

    const std::string& GetChannelName() const { return channel_name_; }

private:
    std::string channel_name_;
    eprosima::fastdds::dds::DataWriter* writer_ = nullptr;
};

template<typename T>
using ChannelPublisherPtr = std::shared_ptr<ChannelPublisher<T>>;

}// namespace wlrobot::robot

