#pragma once

#include <cmath>

namespace hrocket {

struct Vec3 {
    double x{};
    double y{};
    double z{};

    constexpr Vec3 operator+(Vec3 rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    constexpr Vec3 operator-(Vec3 rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    constexpr Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(Vec3 rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }
};

inline Vec3 operator*(double s, Vec3 v) { return v * s; }
inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(Vec3 v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalized(Vec3 v) {
    const double n = norm(v);
    return n > 1.0e-12 ? v / n : Vec3{};
}

struct Quat {
    double w{1.0};
    double x{};
    double y{};
    double z{};
};

inline Quat normalize(Quat q) {
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return n > 1.0e-12 ? Quat{q.w / n, q.x / n, q.y / n, q.z / n} : Quat{};
}

inline Quat multiply(Quat a, Quat b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

inline Vec3 rotate_body_to_ned(Quat q, Vec3 v) {
    q = normalize(q);
    const Quat p{0.0, v.x, v.y, v.z};
    const Quat qc{q.w, -q.x, -q.y, -q.z};
    const Quat r = multiply(multiply(q, p), qc);
    return {r.x, r.y, r.z};
}

inline Vec3 rotate_ned_to_body(Quat q, Vec3 v) {
    q = normalize(q);
    const Quat qc{q.w, -q.x, -q.y, -q.z};
    const Quat p{0.0, v.x, v.y, v.z};
    const Quat r = multiply(multiply(qc, p), q);
    return {r.x, r.y, r.z};
}

inline double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

} // namespace hrocket

