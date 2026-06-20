#pragma once

#include "bag_reader.hpp"

#include <mcap/reader.hpp>
#include <memory>
#include <string>
#include <vector>

class McapReader : public BagReader {
public:
    explicit McapReader(const std::string& path);

    std::vector<TopicInfo> topics() override;
    bool                   next(RawMessage& out) override;
    void                   rewind() override;

private:
    // IterState bundles the view and its iterator so they share a lifetime.
    // It must be destroyed before reader_, hence declared after it.
    struct IterState {
        mcap::LinearMessageView           view;
        mcap::LinearMessageView::Iterator it;

        explicit IterState(mcap::McapReader& r)
            : view(r.readMessages()), it(view.begin()) {}
    };

    mcap::McapReader           reader_;      // owns the file handle
    std::unique_ptr<IterState> iter_state_;  // destroyed before reader_
};
