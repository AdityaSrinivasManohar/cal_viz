#include "tf/tf_buffer.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

// ── Eigen helpers ─────────────────────────────────────────────────────────────

namespace {

Eigen::Quaterniond to_eigen(const Quat& q) {
    return {q.w, q.x, q.y, q.z};  // Eigen ctor is (w, x, y, z)
}

Eigen::Vector3d to_eigen(const Vec3& v) { return {v.x, v.y, v.z}; }

// Apply transform b after transform a: result maps child→parent across both edges.
Transform chain(const Transform& a, const Transform& b) {
    Eigen::Quaterniond qa = to_eigen(a.rotation);
    Eigen::Quaterniond qb = to_eigen(b.rotation);
    Eigen::Vector3d    ta = to_eigen(a.translation);
    Eigen::Vector3d    tb = to_eigen(b.translation);

    Eigen::Quaterniond q = qb * qa;
    Eigen::Vector3d    t = qb * ta + tb;

    return {{t.x(), t.y(), t.z()}, {q.x(), q.y(), q.z(), q.w()}};
}

// Invert an SE3 transform.
Transform inverse(const Transform& tf) {
    Eigen::Quaterniond q = to_eigen(tf.rotation).conjugate();
    Eigen::Vector3d    t = -(q * to_eigen(tf.translation));
    return {{t.x(), t.y(), t.z()}, {q.x(), q.y(), q.z(), q.w()}};
}

constexpr Transform kIdentity{{0, 0, 0}, {0, 0, 0, 1}};

}  // namespace

// ── TfBuffer ──────────────────────────────────────────────────────────────────

void TfBuffer::add_static(const StampedTransform& tf) {
    static_[{tf.parent_frame, tf.child_frame}] = tf;
}

void TfBuffer::add_dynamic(const StampedTransform& tf) {
    auto& seq = dynamic_[{tf.parent_frame, tf.child_frame}];
    auto  it =
        std::lower_bound(seq.begin(), seq.end(), tf.stamp_ns,
                         [](const StampedTransform& s, uint64_t t) { return s.stamp_ns < t; });
    seq.insert(it, tf);
}

std::vector<std::string> TfBuffer::frames() const {
    std::unordered_set<std::string> seen;
    for (auto& [key, _] : static_) {
        seen.insert(key.first);
        seen.insert(key.second);
    }
    for (auto& [key, _] : dynamic_) {
        seen.insert(key.first);
        seen.insert(key.second);
    }
    return {seen.begin(), seen.end()};
}

bool TfBuffer::can_transform(const std::string& from, const std::string& to) const {
    return bfs_path(from, to).has_value();
}

std::optional<Transform> TfBuffer::lookup(const std::string& from_frame,
                                          const std::string& to_frame, uint64_t stamp_ns) const {
    auto path = bfs_path(from_frame, to_frame);
    if (!path) return std::nullopt;
    if (path->size() == 1) return kIdentity;

    Transform result = kIdentity;

    for (size_t i = 0; i + 1 < path->size(); ++i) {
        const std::string& a = (*path)[i];
        const std::string& b = (*path)[i + 1];

        std::optional<Transform> edge;

        if (auto it = static_.find({a, b}); it != static_.end())
            edge = it->second.tf;
        else if (auto it = static_.find({b, a}); it != static_.end())
            edge = inverse(it->second.tf);
        else if (auto it = dynamic_.find({a, b}); it != dynamic_.end())
            edge = interpolate(it->second, stamp_ns);
        else if (auto it = dynamic_.find({b, a}); it != dynamic_.end())
            edge = inverse(interpolate(it->second, stamp_ns));

        if (!edge) return std::nullopt;
        result = chain(result, *edge);
    }

    return result;
}

// ── private helpers ───────────────────────────────────────────────────────────

std::optional<std::vector<std::string>> TfBuffer::bfs_path(const std::string& from,
                                                           const std::string& to) const {
    if (from == to) return std::vector<std::string>{from};

    // Build undirected adjacency from all known edges.
    std::unordered_map<std::string, std::vector<std::string>> adj;
    for (auto& [key, _] : static_) {
        adj[key.first].push_back(key.second);
        adj[key.second].push_back(key.first);
    }
    for (auto& [key, _] : dynamic_) {
        adj[key.first].push_back(key.second);
        adj[key.second].push_back(key.first);
    }

    std::unordered_map<std::string, std::string> parent;
    std::queue<std::string>                      q;
    q.push(from);
    parent[from] = "";

    while (!q.empty()) {
        auto curr = q.front();
        q.pop();
        if (curr == to) {
            std::vector<std::string> path;
            for (std::string n = to; !n.empty(); n = parent[n]) path.push_back(n);
            std::reverse(path.begin(), path.end());
            return path;
        }
        for (auto& next : adj[curr]) {
            if (!parent.count(next)) {
                parent[next] = curr;
                q.push(next);
            }
        }
    }

    return std::nullopt;
}

Transform TfBuffer::interpolate(const std::vector<StampedTransform>& seq, uint64_t t) const {
    if (seq.empty()) return kIdentity;
    if (t <= seq.front().stamp_ns) return seq.front().tf;
    if (t >= seq.back().stamp_ns) return seq.back().tf;

    auto it = std::lower_bound(
        seq.begin(), seq.end(), t,
        [](const StampedTransform& s, uint64_t stamp) { return s.stamp_ns < stamp; });

    const auto& hi = *it;
    const auto& lo = *std::prev(it);

    double alpha =
        static_cast<double>(t - lo.stamp_ns) / static_cast<double>(hi.stamp_ns - lo.stamp_ns);

    Eigen::Vector3d ta = to_eigen(lo.tf.translation);
    Eigen::Vector3d tb = to_eigen(hi.tf.translation);
    Eigen::Vector3d t_interp = ta + alpha * (tb - ta);

    Eigen::Quaterniond qa = to_eigen(lo.tf.rotation);
    Eigen::Quaterniond qb = to_eigen(hi.tf.rotation);
    Eigen::Quaterniond q = qa.slerp(alpha, qb);

    return {{t_interp.x(), t_interp.y(), t_interp.z()}, {q.x(), q.y(), q.z(), q.w()}};
}
