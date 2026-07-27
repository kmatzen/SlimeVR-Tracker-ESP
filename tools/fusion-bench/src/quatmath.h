// Minimal quaternion / vector helpers for the host-side fusion benchmark.
//
// Convention throughout: quaternions are [w, x, y, z] and rotate a vector from
// the body frame into the world frame (the same convention VQF uses). The world
// frame is z-up, so gravity as measured by a level, stationary accelerometer is
// +z.
#pragma once

#include <cmath>

namespace fb {

constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;

inline double rad2deg(double r) { return r * 180.0 / kPi; }
inline double deg2rad(double d) { return d * kPi / 180.0; }

struct Vec3 {
	double x = 0, y = 0, z = 0;
};

struct Quat {
	double w = 1, x = 0, y = 0, z = 0;
};

inline Quat qMul(const Quat& a, const Quat& b) {
	return Quat{
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	};
}

inline Quat qConj(const Quat& q) { return Quat{q.w, -q.x, -q.y, -q.z}; }

inline Quat qNorm(const Quat& q) {
	double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (n <= 0) {
		return Quat{};
	}
	return Quat{q.w / n, q.x / n, q.y / n, q.z / n};
}

// Rotate a body-frame vector into the world frame: v_world = q * v_body * q^-1.
inline Vec3 qRotate(const Quat& q, const Vec3& v) {
	Quat p{0, v.x, v.y, v.z};
	Quat r = qMul(qMul(q, p), qConj(q));
	return Vec3{r.x, r.y, r.z};
}

// Rotate a world-frame vector into the body frame.
inline Vec3 qRotateInv(const Quat& q, const Vec3& v) { return qRotate(qConj(q), v); }

// Rotation about the world z axis by `psi` radians.
inline Quat qFromYaw(double psi) {
	return Quat{std::cos(psi / 2), 0, 0, std::sin(psi / 2)};
}

// Total geodesic angle of a quaternion, in radians, always in [0, pi].
inline double qAngle(const Quat& q) {
	double w = std::fabs(q.w);
	if (w > 1.0) {
		w = 1.0;
	}
	return 2.0 * std::acos(w);
}

// Wrap an angle to (-pi, pi].
inline double wrapPi(double a) {
	while (a > kPi) {
		a -= 2 * kPi;
	}
	while (a <= -kPi) {
		a += 2 * kPi;
	}
	return a;
}

// Heading (rotation about world z) of a quaternion, via swing-twist
// decomposition about the z axis. Exact, and well-defined except at the
// degenerate point where the twist component vanishes.
inline double qHeading(const Quat& q) { return wrapPi(2.0 * std::atan2(q.z, q.w)); }

// Angle between the world z axis and where this rotation sends it. This is the
// heading-independent ("inclination") part of an orientation difference:
// premultiplying q by any rotation about world z leaves it unchanged.
//
// (R * e_z) . e_z = 1 - 2(x^2 + y^2) for a unit quaternion.
inline double qInclination(const Quat& q) {
	double c = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	if (c > 1.0) {
		c = 1.0;
	}
	if (c < -1.0) {
		c = -1.0;
	}
	return std::acos(c);
}

inline double vAngle(const Vec3& a, const Vec3& b) {
	double na = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
	double nb = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
	if (na <= 0 || nb <= 0) {
		return 0.0;
	}
	double c = (a.x * b.x + a.y * b.y + a.z * b.z) / (na * nb);
	if (c > 1.0) {
		c = 1.0;
	}
	if (c < -1.0) {
		c = -1.0;
	}
	return std::acos(c);
}

// Integrate a quaternion forward by a body-frame angular velocity over dt,
// using the exact exponential map (not a first-order approximation), so that
// synthetic ground truth is limited by the trajectory definition rather than by
// integration error.
inline Quat qIntegrate(const Quat& q, const Vec3& omega, double dt) {
	double n = std::sqrt(omega.x * omega.x + omega.y * omega.y + omega.z * omega.z);
	if (n < 1e-12) {
		return q;
	}
	double half = 0.5 * n * dt;
	double s = std::sin(half) / n;
	Quat dq{std::cos(half), omega.x * s, omega.y * s, omega.z * s};
	return qNorm(qMul(q, dq));
}

// Deterministic, portable PRNG. std::mt19937 is reproducible across platforms
// but the standard distributions are not, so both are rolled here to guarantee
// that a dataset generated on one machine is bit-identical on another.
class Rng {
public:
	explicit Rng(uint64_t seed)
		: state_(seed ? seed : 0x9e3779b97f4a7c15ull) {}

	uint64_t next() {
		// xorshift64*
		state_ ^= state_ >> 12;
		state_ ^= state_ << 25;
		state_ ^= state_ >> 27;
		return state_ * 0x2545F4914F6CDD1Dull;
	}

	// Uniform in [0, 1). The shift leaves 53 significant bits, which is exactly
	// what a double can represent, so the cast is lossless.
	double uniform() {
		return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0);
	}

	// Standard normal, via Box-Muller. Caches the second variate.
	double normal() {
		if (hasSpare_) {
			hasSpare_ = false;
			return spare_;
		}
		double u1 = uniform();
		double u2 = uniform();
		if (u1 < 1e-300) {
			u1 = 1e-300;
		}
		double r = std::sqrt(-2.0 * std::log(u1));
		double t = 2.0 * kPi * u2;
		spare_ = r * std::sin(t);
		hasSpare_ = true;
		return r * std::cos(t);
	}

private:
	uint64_t state_;
	double spare_ = 0;
	bool hasSpare_ = false;
};

}  // namespace fb
