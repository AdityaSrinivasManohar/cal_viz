#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct RawMessage {
    std::string          topic;
    std::string          msg_type;
    uint64_t             log_time_ns;
    uint64_t             publish_time_ns;
    std::vector<uint8_t> data;
};

struct TopicInfo {
    std::string topic;
    std::string msg_type;
    uint64_t    message_count;
};

class BagReader {
public:
    virtual ~BagReader() = default;

    virtual std::vector<TopicInfo> topics() = 0;
    virtual bool                   next(RawMessage& out) = 0;
    virtual void                   rewind() = 0;
};
