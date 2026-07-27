#include "synth.h"

#include <cmath>

namespace fb {
namespace {

// Body-frame angular velocity, rad/s, as a function of time.
Vec3 omegaAt(const std::string& traj, double t, double duration) {
	if (traj == "static" || traj == "static-tilted") {
		return Vec3{0, 0, 0};
	}

	if (traj == "yaw-sweep") {
		// Smooth back-and-forth about the vertical, +-60 deg at 0.2 Hz.
		const double f = 0.2;
		const double amp = deg2rad(60.0);
		return Vec3{0, 0, amp * 2 * kPi * f * std::cos(2 * kPi * f * t)};
	}

	if (traj == "tumble") {
		// Three incommensurate frequencies, so the trajectory never repeats and
		// excites all three axes.
		return Vec3{
			deg2rad(40.0) * std::sin(2 * kPi * 0.13 * t),
			deg2rad(55.0) * std::sin(2 * kPi * 0.17 * t + 1.0),
			deg2rad(35.0) * std::sin(2 * kPi * 0.11 * t + 2.0),
		};
	}

	if (traj == "return-to-origin") {
		// Rotate for the first 80% of the run, then hold. The rotation is
		// constructed so the net rotation over the moving phase is exactly
		// zero: a full 360 deg about x, then y, then z. Whatever heading error
		// remains at the end is accumulated error, which is precisely the
		// quantity the physical return-to-origin bench test measures.
		const double moving = duration * 0.8;
		if (t >= moving) {
			return Vec3{0, 0, 0};
		}
		const double seg = moving / 3.0;
		const double rate = 2 * kPi / seg;  // one full turn per segment
		if (t < seg) {
			return Vec3{rate, 0, 0};
		}
		if (t < 2 * seg) {
			return Vec3{0, rate, 0};
		}
		return Vec3{0, 0, rate};
	}

	if (traj == "walk") {
		// Limb-like: a dominant swing about one axis at ~1 Hz with smaller
		// out-of-plane components, roughly what a shin tracker sees.
		return Vec3{
			deg2rad(70.0) * std::sin(2 * kPi * 1.0 * t),
			deg2rad(12.0) * std::sin(2 * kPi * 2.0 * t + 0.5),
			deg2rad(8.0) * std::sin(2 * kPi * 1.0 * t + 1.2),
		};
	}

	return Vec3{0, 0, 0};
}

// World-frame linear acceleration, m/s^2. Only the walk trajectory has any.
Vec3 linAccAt(const std::string& traj, double t) {
	if (traj != "walk") {
		return Vec3{0, 0, 0};
	}
	return Vec3{
		1.2 * std::sin(2 * kPi * 1.0 * t),
		0.6 * std::sin(2 * kPi * 2.0 * t + 0.3),
		1.8 * std::sin(2 * kPi * 2.0 * t),
	};
}

}  // namespace

bool listTrajectories(std::string& out) {
	out = "static, static-tilted, yaw-sweep, tumble, return-to-origin, walk";
	return true;
}

bool generate(
	const std::string& traj,
	const SynthParams& p,
	Dataset& out,
	std::string& error
) {
	std::string known;
	listTrajectories(known);
	if (known.find(traj) == std::string::npos) {
		error = "unknown trajectory '" + traj + "' (known: " + known + ")";
		return false;
	}
	if (p.rateHz <= 0 || p.durationSec <= 0) {
		error = "rate and duration must be positive";
		return false;
	}

	const double dt = 1.0 / p.rateHz;
	const size_t n = static_cast<size_t>(p.durationSec * p.rateHz);

	out = Dataset{};
	out.name = traj;
	out.note = "synthetic: " + traj;
	out.gyrTs = dt;
	out.accTs = dt;
	out.magTs = dt;
	out.hasMag = p.withMag;
	out.hasGroundTruth = true;
	out.samples.reserve(n);

	Rng rng(p.seed);

	// Starting attitude. "static-tilted" starts 30 deg off level so that the
	// tilt metric has something to be wrong about.
	Quat q{1, 0, 0, 0};
	if (traj == "static-tilted") {
		const double a = deg2rad(30.0) / 2;
		q = Quat{std::cos(a), std::sin(a), 0, 0};
	}

	const Vec3 gravityWorld{0, 0, kGravity};
	// A plausible mid-latitude field: pointing north and downward, ~50 uT.
	const Vec3 magWorld{20.0, 0.0, -45.0};

	const Vec3 gyroBias{
		deg2rad(p.gyroBiasDps),
		deg2rad(p.gyroBiasDps) * 0.6,
		deg2rad(p.gyroBiasDps) * -0.4,
	};

	for (size_t i = 0; i < n; i++) {
		const double t = static_cast<double>(i) * dt;
		const Vec3 omega = omegaAt(traj, t, p.durationSec);

		Sample s;
		s.tUs = static_cast<uint64_t>(std::llround(t * 1e6));
		s.gt = q;

		// Specific force in the world frame is linear acceleration plus the
		// reaction to gravity; rotate it into the body frame.
		const Vec3 la = linAccAt(traj, t);
		const Vec3 sfWorld{
			la.x + gravityWorld.x, la.y + gravityWorld.y, la.z + gravityWorld.z
		};
		const Vec3 accTrue = qRotateInv(q, sfWorld);
		const Vec3 magTrue = qRotateInv(q, magWorld);

		const double gs = 1.0 + p.gyroScaleErr;
		s.gyr = Vec3{
			omega.x * gs + gyroBias.x + deg2rad(p.gyroNoiseDps) * rng.normal(),
			omega.y * gs + gyroBias.y + deg2rad(p.gyroNoiseDps) * rng.normal(),
			omega.z * gs + gyroBias.z + deg2rad(p.gyroNoiseDps) * rng.normal(),
		};
		s.acc = Vec3{
			accTrue.x + p.accelBias + p.accelNoise * rng.normal(),
			accTrue.y + p.accelBias * 0.5 + p.accelNoise * rng.normal(),
			accTrue.z + p.accelBias * -0.3 + p.accelNoise * rng.normal(),
		};
		if (p.withMag) {
			s.mag = Vec3{
				magTrue.x + 0.3 * rng.normal(),
				magTrue.y + 0.3 * rng.normal(),
				magTrue.z + 0.3 * rng.normal(),
			};
		}

		out.samples.push_back(s);

		// Advance the reference attitude with the *true* rate. The reference is
		// defined by this integration, so it carries no error of its own.
		q = qIntegrate(q, omega, dt);
	}

	if (out.samples.size() < 2) {
		error = "generated dataset too short";
		return false;
	}
	return true;
}

}  // namespace fb
