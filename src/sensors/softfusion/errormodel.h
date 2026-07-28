/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
	FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
	IN THE SOFTWARE.
*/

#pragma once

#include <cmath>
#include <cstddef>

// Free of any Arduino or hardware dependency so the arithmetic can be unit
// tested on a host. See tools/fusion-bench/tests/selftest.cpp.

namespace SlimeVR::Sensors::SoftFusion {

/**
 * The full first-order sensor error model: `corrected = M * (raw - bias)`.
 *
 * The runtime calibration currently models bias only. Bias is the largest term
 * and its treatment here is good -- rest-gated estimation with temperature
 * compensation removes about three orders of magnitude of gyroscope drift, as
 * measured. But bias is the term that dominates a *stationary* sensor, and a
 * body tracker is strapped to a moving limb.
 *
 * The terms this adds are the ones that dominate when moving:
 *
 *  - **Scale factor** produces error proportional to rotation *angle*, not to
 *    time. Turn 180 degrees with a 1% scale error and you are 1.8 degrees out
 *    immediately, however good the bias estimate is. Rest-gated calibration
 *    cannot see this at all, because at rest there is no rotation to scale.
 *
 *  - **Axis misalignment** couples motion between axes, turning pitch and roll
 *    into spurious yaw. That is the most damaging possible direction for the
 *    error to go, because yaw is the unobservable axis on a 6-DoF tracker.
 *
 * Both are systematic, so they accumulate; noise does not.
 *
 * `M` defaults to identity, so a device with no fitted model behaves exactly as
 * before and the storage can land ahead of the estimation.
 */
struct ErrorModel {
	/// Row-major 3x3. Identity by default.
	float m[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	float bias[3] = {0, 0, 0};

	[[nodiscard]] bool isIdentity() const {
		constexpr float eps = 1e-6f;
		const float ident[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
		for (int i = 0; i < 9; i++) {
			if (std::fabs(m[i] - ident[i]) > eps) {
				return false;
			}
		}
		return std::fabs(bias[0]) <= eps && std::fabs(bias[1]) <= eps
			&& std::fabs(bias[2]) <= eps;
	}

	void apply(const float in[3], float out[3]) const {
		const float d0 = in[0] - bias[0];
		const float d1 = in[1] - bias[1];
		const float d2 = in[2] - bias[2];
		out[0] = m[0] * d0 + m[1] * d1 + m[2] * d2;
		out[1] = m[3] * d0 + m[4] * d1 + m[5] * d2;
		out[2] = m[6] * d0 + m[7] * d1 + m[8] * d2;
	}
};

namespace detail {

/// Solves `a x = b` for a small dense system. Returns false if singular.
inline bool solveLinear(double* a, double* b, int n) {
	for (int col = 0; col < n; col++) {
		int pivot = col;
		for (int r = col; r < n; r++) {
			if (std::fabs(a[r * n + col]) > std::fabs(a[pivot * n + col])) {
				pivot = r;
			}
		}
		if (std::fabs(a[pivot * n + col]) < 1e-14) {
			return false;
		}
		if (pivot != col) {
			for (int k = 0; k < n; k++) {
				const double t = a[col * n + k];
				a[col * n + k] = a[pivot * n + k];
				a[pivot * n + k] = t;
			}
			const double t = b[col];
			b[col] = b[pivot];
			b[pivot] = t;
		}
		for (int r = 0; r < n; r++) {
			if (r == col) {
				continue;
			}
			const double f = a[r * n + col] / a[col * n + col];
			if (f == 0.0) {
				continue;
			}
			for (int k = col; k < n; k++) {
				a[r * n + k] -= f * a[col * n + k];
			}
			b[r] -= f * b[col];
		}
	}
	for (int i = 0; i < n; i++) {
		b[i] /= a[i * n + i];
	}
	return true;
}

/// Lower-triangular Cholesky of a symmetric positive-definite 3x3.
inline bool cholesky3(const double q[9], double l[9]) {
	for (int i = 0; i < 9; i++) {
		l[i] = 0.0;
	}
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j <= i; j++) {
			double sum = q[i * 3 + j];
			for (int k = 0; k < j; k++) {
				sum -= l[i * 3 + k] * l[j * 3 + k];
			}
			if (i == j) {
				if (sum <= 0.0) {
					return false;
				}
				l[i * 3 + j] = std::sqrt(sum);
			} else {
				l[i * 3 + j] = sum / l[j * 3 + j];
			}
		}
	}
	return true;
}

}  // namespace detail

/**
 * Fits an error model to static measurements taken in many orientations.
 *
 * The physical fact this exploits: however the sensor is oriented, a stationary
 * accelerometer measures a vector of constant magnitude. Uncorrected readings
 * therefore trace an *ellipsoid* rather than a sphere, and the ellipsoid's
 * centre is the bias while its shape is the combined scale and misalignment.
 * Recovering both is a quadric fit.
 *
 * Requires genuinely varied orientations. Samples clustered near one attitude
 * leave the fit under-determined -- it will still return numbers, so the caller
 * has to supply good data rather than relying on a failure. The classic
 * procedure is six positions, each axis up and down.
 *
 * @param samples  `3 * count` floats, xyz per sample, in raw scaled units.
 * @param count    Number of samples. At least 9 for the quadric to be
 *                 determined; far more in practice.
 * @param norm     Expected magnitude, e.g. gravity in m/s^2.
 */
inline bool
fitErrorModel(const float* samples, size_t count, float norm, ErrorModel& out) {
	if (count < 9 || norm <= 0.0f) {
		return false;
	}

	// Quadric: x'Qx + u'x = 1, nine unknowns after fixing the constant. Solving
	// the normal equations directly is adequate here -- the design matrix is
	// well conditioned once the orientations are varied, and this runs once at
	// calibration time rather than in the sample path.
	double ata[81] = {0};
	double atb[9] = {0};
	for (size_t s = 0; s < count; s++) {
		const double x = samples[s * 3 + 0];
		const double y = samples[s * 3 + 1];
		const double z = samples[s * 3 + 2];
		const double row[9]
			= {x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, x, y, z};
		for (int i = 0; i < 9; i++) {
			for (int j = 0; j < 9; j++) {
				ata[i * 9 + j] += row[i] * row[j];
			}
			atb[i] += row[i];
		}
	}
	if (!detail::solveLinear(ata, atb, 9)) {
		return false;
	}

	const double qh[9] = {
		atb[0],
		atb[3],
		atb[4],
		atb[3],
		atb[1],
		atb[5],
		atb[4],
		atb[5],
		atb[2],
	};
	const double uh[3] = {atb[6], atb[7], atb[8]};

	// Centre: b = -0.5 * Q^-1 u. The unknown overall scale cancels here, which
	// is why the bias can be recovered before the scale is known.
	double qinv[9];
	for (int i = 0; i < 9; i++) {
		qinv[i] = qh[i];
	}
	double rhs[3] = {-0.5 * uh[0], -0.5 * uh[1], -0.5 * uh[2]};
	if (!detail::solveLinear(qinv, rhs, 3)) {
		return false;
	}
	const double b[3] = {rhs[0], rhs[1], rhs[2]};

	// Recover the scale the constant-term normalisation threw away.
	double bqb = 0.0;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			bqb += b[i] * qh[i * 3 + j] * b[j];
		}
	}
	const double denom = 1.0 + bqb;
	if (std::fabs(denom) < 1e-12) {
		return false;
	}
	const double scale = static_cast<double>(norm) * norm / denom;

	double q[9];
	for (int i = 0; i < 9; i++) {
		q[i] = qh[i] * scale;
	}

	// M with M'M = Q. Cholesky picks one of the many valid square roots; they
	// differ by a rotation, which is unobservable from magnitudes alone and is
	// handled by the existing mounting calibration rather than here.
	double l[9];
	if (!detail::cholesky3(q, l)) {
		return false;
	}

	// M = L', so that M'M = L L' = Q.
	out.m[0] = static_cast<float>(l[0]);
	out.m[1] = static_cast<float>(l[3]);
	out.m[2] = static_cast<float>(l[6]);
	out.m[3] = 0.0f;
	out.m[4] = static_cast<float>(l[4]);
	out.m[5] = static_cast<float>(l[7]);
	out.m[6] = 0.0f;
	out.m[7] = 0.0f;
	out.m[8] = static_cast<float>(l[8]);
	out.bias[0] = static_cast<float>(b[0]);
	out.bias[1] = static_cast<float>(b[1]);
	out.bias[2] = static_cast<float>(b[2]);
	return true;
}

}  // namespace SlimeVR::Sensors::SoftFusion
