#pragma once

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>

#include <memory>
#include <mutex>
#include <functional>
#include <string>

#include <wlrobot/robot/channel/pubsub_type_resolver.hpp>

namespace wlrobot::robot {

class ChannelFactory {
public:
    static ChannelFactory* Instance() {
        static ChannelFactory instance;
        return &instance;
    }

    bool Init(uint32_t domain_id, const std::string& nic = "");

    template<typename T>
    eprosima::fastdds::dds::DataWriter* CreateSendChannel(const std::string& channel_name) {
        using namespace eprosima::fastdds::dds;

        auto type = TypeSupport(new typename PubSubTypeResolver<T>::type());
        type.register_type(participant_.get());

        Topic* topic = participant_->create_topic(channel_name, type.get_type_name(), TOPIC_QOS_DEFAULT);
        return publisher_->create_datawriter(topic, DATAWRITER_QOS_DEFAULT, nullptr);
    }

    template<typename T>
    eprosima::fastdds::dds::DataReader* CreateRecvChannel(
        const std::string& channel_name,
        std::function<void(const T&)> callback,
        int /*queue_len*/ = 10) {

        using namespace eprosima::fastdds::dds;

        auto type = TypeSupport(new typename PubSubTypeResolver<T>::type());
        type.register_type(participant_.get());

        Topic* topic = participant_->create_topic(channel_name, type.get_type_name(), TOPIC_QOS_DEFAULT);

        class InternalListener : public DataReaderListener {
        public:
            explicit InternalListener(std::function<void(const T&)> cb) : callback_(std::move(cb)) {}
            void on_data_available(DataReader* reader) override {
                T msg;
                SampleInfo info;
                if (reader->take_next_sample(&msg, &info) == RETCODE_OK && info.valid_data) {
                    if (callback_) callback_(msg);
                }
            }
        private:
            std::function<void(const T&)> callback_;
        };

        auto listener = new InternalListener(callback);  // 留给程序整体管理释放
        return subscriber_->create_datareader(topic, DATAREADER_QOS_DEFAULT, listener);
    }

    eprosima::fastdds::dds::DomainParticipant* Participant() {
        return participant_.get();
    }

    eprosima::fastdds::dds::Publisher* Publisher() {
        return publisher_.get();
    }

    eprosima::fastdds::dds::Subscriber* Subscriber() {
        return subscriber_.get();
    }

private:
    ChannelFactory() = default;
    ~ChannelFactory() = default;

    static std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant_;
    static std::shared_ptr<eprosima::fastdds::dds::Publisher> publisher_;
    static std::shared_ptr<eprosima::fastdds::dds::Subscriber> subscriber_;
};

}  // namespace wlrobot::robot

