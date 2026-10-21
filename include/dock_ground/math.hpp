#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace dock_ground {

using Mat3 = std::array<double, 9>;

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

[[nodiscard]] inline Vec3 operator+(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

[[nodiscard]] inline Vec3 operator-(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

[[nodiscard]] inline Vec3 operator*(const Vec3& value, double scalar) {
    return {value.x * scalar, value.y * scalar, value.z * scalar};
}

[[nodiscard]] inline Vec3 operator/(const Vec3& value, double scalar) {
    return {value.x / scalar, value.y / scalar, value.z / scalar};
}

[[nodiscard]] inline double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

[[nodiscard]] inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

[[nodiscard]] inline double norm_squared(const Vec3& value) {
    return dot(value, value);
}

[[nodiscard]] inline double norm(const Vec3& value) {
    return std::sqrt(norm_squared(value));
}

[[nodiscard]] inline Vec3 normalized(const Vec3& value) {
    const double magnitude = norm(value);
    if (!std::isfinite(magnitude) || magnitude < 1e-12) {
        return {};
    }
    return value / magnitude;
}

[[nodiscard]] inline bool is_finite(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

[[nodiscard]] inline bool is_finite(const Quaternion& value) {
    return std::isfinite(value.w) && std::isfinite(value.x) &&
           std::isfinite(value.y) && std::isfinite(value.z);
}

[[nodiscard]] inline Quaternion normalized(const Quaternion& value) {
    const double magnitude = std::sqrt(
        value.w * value.w + value.x * value.x + value.y * value.y +
        value.z * value.z);
    if (!std::isfinite(magnitude) || magnitude < 1e-12) {
        return {};
    }
    return {
        value.w / magnitude,
        value.x / magnitude,
        value.y / magnitude,
        value.z / magnitude,
    };
}

[[nodiscard]] inline Quaternion conjugate(const Quaternion& value) {
    return {value.w, -value.x, -value.y, -value.z};
}

[[nodiscard]] inline double clamp(double value, double low, double high) {
    return std::clamp(value, low, high);
}

[[nodiscard]] inline Mat3 operator*(const Mat3& a, const Mat3& b) {
    Mat3 result{};
    for (std::size_t row = 0; row < 3U; ++row) {
        for (std::size_t column = 0; column < 3U; ++column) {
            double value = 0.0;
            for (std::size_t inner = 0; inner < 3U; ++inner) {
                value += a[row * 3U + inner] * b[inner * 3U + column];
            }
            result[row * 3U + column] = value;
        }
    }
    return result;
}

[[nodiscard]] inline Mat3 transpose(const Mat3& value) {
    return {
        value[0], value[3], value[6],
        value[1], value[4], value[7],
        value[2], value[5], value[8],
    };
}

[[nodiscard]] inline Vec3 transform(const Mat3& matrix, const Vec3& value) {
    return {
        matrix[0] * value.x + matrix[1] * value.y + matrix[2] * value.z,
        matrix[3] * value.x + matrix[4] * value.y + matrix[5] * value.z,
        matrix[6] * value.x + matrix[7] * value.y + matrix[8] * value.z,
    };
}

[[nodiscard]] inline bool is_finite(const Mat3& value) {
    for (const double entry : value) {
        if (!std::isfinite(entry)) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] inline double normalized_angle(double angle) {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 2.0 * kPi;
    if (!std::isfinite(angle)) {
        return 0.0;
    }
    angle = std::fmod(angle + kPi, kTwoPi);
    if (angle < 0.0) {
        angle += kTwoPi;
    }
    return angle - kPi;
}

// Builds q_body_to_world from orthonormal body axes in a right-handed
// Z-up world. Body +X is forward/nose and body +Z is up.
[[nodiscard]] inline Quaternion quaternion_from_body_axes(
    const Vec3& body_x, const Vec3& body_y, const Vec3& body_z) {
    const double m00 = body_x.x;
    const double m01 = body_y.x;
    const double m02 = body_z.x;
    const double m10 = body_x.y;
    const double m11 = body_y.y;
    const double m12 = body_z.y;
    const double m20 = body_x.z;
    const double m21 = body_y.z;
    const double m22 = body_z.z;

    Quaternion q{};
    const double trace = m00 + m11 + m22;
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w = 0.25 * s;
        q.x = (m21 - m12) / s;
        q.y = (m02 - m20) / s;
        q.z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        q.w = (m21 - m12) / s;
        q.x = 0.25 * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
    } else if (m11 > m22) {
        const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        q.w = (m02 - m20) / s;
        q.x = (m01 + m10) / s;
        q.y = 0.25 * s;
        q.z = (m12 + m21) / s;
    } else {
        const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
        q.w = (m10 - m01) / s;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = 0.25 * s;
    }
    return normalized(q);
}

}  // namespace dock_ground
