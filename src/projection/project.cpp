#include "projection/project.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace projection {

namespace {

// ── colormap ──────────────────────────────────────────────────────────────────

// Turbo-like rainbow: t=0 → blue (far), t=1 → red (near).
std::array<uint8_t, 3> turbo(float t) {
    t = std::clamp(t, 0.f, 1.f);
    float r, g, b;
    if (t < 0.25f) {
        float s = t / 0.25f;
        r = 0.f;
        g = s;
        b = 1.f;
    } else if (t < 0.5f) {
        float s = (t - 0.25f) / 0.25f;
        r = 0.f;
        g = 1.f;
        b = 1.f - s;
    } else if (t < 0.75f) {
        float s = (t - 0.5f) / 0.25f;
        r = s;
        g = 1.f;
        b = 0.f;
    } else {
        float s = (t - 0.75f) / 0.25f;
        r = 1.f;
        g = 1.f - s;
        b = 0.f;
    }
    return {static_cast<uint8_t>(r * 255), static_cast<uint8_t>(g * 255),
            static_cast<uint8_t>(b * 255)};
}

// ── distortion ────────────────────────────────────────────────────────────────

// Returns (x'', y'') after applying plumb_bob distortion to normalised (x', y').
std::pair<double, double> distort_plumb_bob(double xp, double yp, const std::vector<double>& D) {
    double k1 = D.size() > 0 ? D[0] : 0;
    double k2 = D.size() > 1 ? D[1] : 0;
    double p1 = D.size() > 2 ? D[2] : 0;
    double p2 = D.size() > 3 ? D[3] : 0;
    double k3 = D.size() > 4 ? D[4] : 0;

    double r2 = xp * xp + yp * yp;
    double scale = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    double xpp = xp * scale + 2 * p1 * xp * yp + p2 * (r2 + 2 * xp * xp);
    double ypp = yp * scale + p1 * (r2 + 2 * yp * yp) + 2 * p2 * xp * yp;
    return {xpp, ypp};
}

// Returns (x'', y'') after applying equidistant (fisheye) distortion.
std::pair<double, double> distort_equidistant(double xp, double yp, const std::vector<double>& D) {
    double k1 = D.size() > 0 ? D[0] : 0;
    double k2 = D.size() > 1 ? D[1] : 0;
    double k3 = D.size() > 2 ? D[2] : 0;
    double k4 = D.size() > 3 ? D[3] : 0;

    double r = std::sqrt(xp * xp + yp * yp);
    if (r < 1e-9) return {xp, yp};

    double theta = std::atan(r);
    double theta2 = theta * theta;
    double td = theta * (1 + k1 * theta2 + k2 * theta2 * theta2 + k3 * theta2 * theta2 * theta2 +
                         k4 * theta2 * theta2 * theta2 * theta2);
    double scale = td / r;
    return {xp * scale, yp * scale};
}

}  // namespace

// ── public API ────────────────────────────────────────────────────────────────

std::vector<ProjectedPoint> project(const msgs::PointCloud& cloud, const Transform& lidar_to_cam,
                                    const msgs::CameraInfo& cam_info) {
    Eigen::Quaterniond q(lidar_to_cam.rotation.w, lidar_to_cam.rotation.x, lidar_to_cam.rotation.y,
                         lidar_to_cam.rotation.z);
    Eigen::Vector3d    t(lidar_to_cam.translation.x, lidar_to_cam.translation.y,
                         lidar_to_cam.translation.z);
    Eigen::Matrix3d    R = q.toRotationMatrix();

    const auto& P = cam_info.P;
    const auto& K = cam_info.K;
    const auto& D = cam_info.D;
    const int   W = static_cast<int>(cam_info.width);
    const int   H = static_cast<int>(cam_info.height);
    bool        rectified = std::all_of(D.begin(), D.end(), [](double v) { return v == 0.0; });

    std::vector<ProjectedPoint> result;
    result.reserve(cloud.points.size() / 4);

    for (const auto& pt : cloud.points) {
        Eigen::Vector3d p = R * Eigen::Vector3d(pt.x, pt.y, pt.z) + t;
        if (p.z() <= 0) continue;

        double u, v;

        if (rectified) {
            // Use the 3×4 P matrix directly (pre-rectified image).
            double uh = P[0] * p.x() + P[1] * p.y() + P[2] * p.z() + P[3];
            double vh = P[4] * p.x() + P[5] * p.y() + P[6] * p.z() + P[7];
            double wh = P[8] * p.x() + P[9] * p.y() + P[10] * p.z() + P[11];
            u = uh / wh;
            v = vh / wh;
        } else {
            // Normalised image coords → apply distortion → apply K.
            double xp = p.x() / p.z();
            double yp = p.y() / p.z();

            auto [xpp, ypp] = (cam_info.distortion_model == "equidistant")
                                  ? distort_equidistant(xp, yp, D)
                                  : distort_plumb_bob(xp, yp, D);

            u = K[0] * xpp + K[2];
            v = K[4] * ypp + K[5];
        }

        int iu = static_cast<int>(u + 0.5);
        int iv = static_cast<int>(v + 0.5);
        if (iu < 0 || iu >= W || iv < 0 || iv >= H) continue;

        result.push_back({iu, iv, static_cast<float>(p.z()), pt.intensity});
    }

    return result;
}

std::array<uint8_t, 3> colorize(const ProjectedPoint& pt, ColorizeMode mode, float range_min,
                                float range_max) {
    float span = range_max - range_min;
    if (span < 1e-6f) span = 1e-6f;

    if (mode == ColorizeMode::Intensity) {
        uint8_t v =
            static_cast<uint8_t>(std::clamp((pt.intensity - range_min) / span, 0.f, 1.f) * 255.f);
        return {v, v, v};
    }

    // Depth: near → red (t=1), far → blue (t=0).
    float t = 1.f - std::clamp((pt.depth - range_min) / span, 0.f, 1.f);
    return turbo(t);
}

void render(msgs::Image& image, const std::vector<ProjectedPoint>& points, ColorizeMode mode,
            int dot_radius) {
    if (points.empty()) return;

    float d_min = points[0].depth, d_max = points[0].depth;
    for (auto& p : points) {
        d_min = std::min(d_min, p.depth);
        d_max = std::max(d_max, p.depth);
    }

    const int W = static_cast<int>(image.width);
    const int H = static_cast<int>(image.height);

    for (const auto& pt : points) {
        auto [r, g, b] = colorize(pt, mode, d_min, d_max);
        for (int dy = -dot_radius; dy <= dot_radius; ++dy) {
            for (int dx = -dot_radius; dx <= dot_radius; ++dx) {
                int px = pt.u + dx, py = pt.v + dy;
                if (px < 0 || px >= W || py < 0 || py >= H) continue;
                int idx = (py * W + px) * 3;
                image.data[idx] = r;
                image.data[idx + 1] = g;
                image.data[idx + 2] = b;
            }
        }
    }
}

}  // namespace projection
