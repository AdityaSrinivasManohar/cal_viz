#define MCAP_IMPLEMENTATION  // compile .inl bodies in this TU
#include "bag/mcap_reader.hpp"

#include <stdexcept>

McapReader::McapReader(const std::string& path) {
    auto status = reader_.open(path);
    if (!status.ok())
        throw std::runtime_error("McapReader: " + std::string(status.message));

    // In v2.x, open() only reads the header. readSummary() populates
    // channels(), schemas(), and statistics(). AllowFallbackScan falls back
    // to a sequential scan if the summary section is missing or incomplete.
    status = reader_.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
    if (!status.ok())
        throw std::runtime_error("McapReader: readSummary: " + std::string(status.message));

    iter_state_ = std::make_unique<IterState>(reader_);
}

std::vector<TopicInfo> McapReader::topics() {
    const auto& channels = reader_.channels();
    const auto& schemas = reader_.schemas();
    const auto  stats = reader_.statistics();

    std::vector<TopicInfo> result;
    result.reserve(channels.size());

    for (const auto& [ch_id, ch] : channels) {
        uint64_t count = 0;
        if (stats.has_value() && stats->channelMessageCounts.contains(ch_id))
            count = stats->channelMessageCounts.at(ch_id);

        result.push_back({
            ch->topic,
            schemas.contains(ch->schemaId) ? schemas.at(ch->schemaId)->name : "",
            count,
        });
    }
    return result;
}

bool McapReader::next(RawMessage& out) {
    auto& [view, it] = *iter_state_;
    if (it == view.end()) return false;

    const auto& mv = *it;
    out.topic = mv.channel->topic;
    out.msg_type = mv.schema ? mv.schema->name : "";
    out.log_time_ns = mv.message.logTime;
    out.publish_time_ns = mv.message.publishTime;

    auto* begin = reinterpret_cast<const uint8_t*>(mv.message.data);
    out.data.assign(begin, begin + mv.message.dataSize);

    ++it;
    return true;
}

void McapReader::rewind() { iter_state_ = std::make_unique<IterState>(reader_); }
