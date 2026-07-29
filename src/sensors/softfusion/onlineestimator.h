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
#include <cstdint>

#include "errormodel.h"

// Free of any Arduino or hardware dependency so the estimator can be unit
// tested on a host. See tools/fusion-bench/tests/selftest.cpp.

/**
 * Whether continuous online accelerometer estimation is built in.
 *
 * Shares `DISABLE_GUIDED_ACCEL_CALIBRATION` because it shares that feature's
 * cost: the same solve, the same soft-float `double` library, and the same
 * boards with no room for either. A board that cannot afford the guided flow
 * certainly cannot afford an estimator that runs all the time.
 *
 * Separately nameable so the two can be split later if a board ever wants one
 * without the other.
 */
#ifdef DISABLE_GUIDED_ACCEL_CALIBRATION
#define ONLINE_ACCEL_ESTIMATION 0
#else
#define ONLINE_ACCEL_ESTIMATION 1
#endif

namespace SlimeVR::Sensors::SoftFusion {

/**
 * Rest samples averaged into one observation before it enters the estimator.
 *
 * The same reasoning as the guided flow: sensor noise is what the fit is
 * fighting, and averaging 32 samples is about 5x down on it. Unlike the guided
 * flow there is no cost to averaging more, since nothing is stored -- but a
 * block is also the unit the novelty gate works on, and long blocks make the
 * estimator slow to notice a new orientation.
 */
constexpr size_t kOnlineBlockSamples = 32;

/**
 * How far a new observation must lie from the last accepted one, as a cosine.
 *
 * 15 degrees. This is the gate that makes online estimation possible at all,
 * and the reason is a property of how trackers are actually used rather than
 * anything about the algebra.
 *
 * A tracker left on a desk overnight is at rest for eight hours. At 240 Hz that
 * is seven million samples in *one* orientation. Fed to the normal equations
 * they would swamp every other direction ever seen, and -- worse -- the
 * estimator would look healthy while doing it: the sample count would be
 * enormous and the fit beautifully conditioned about a single point. Coverage
 * would be the only thing wrong, and coverage is exactly what a count cannot
 * measure.
 *
 * So an observation is only taken when the tracker is somewhere it was not
 * already. A long rest in one position contributes one observation, which is
 * precisely what it is worth.
 */
constexpr float kOnlineNoveltyCos = 0.9659f;

/**
 * Effective memory, in observations.
 *
 * Sets the exponential forgetting factor. Bias drifts with temperature and age
 * while scale does not, so the estimator has to be able to follow the one
 * without being dragged by history on the other. 64 observations is long enough
 * that the estimate is not chasing noise and short enough that a day-old
 * reading at a different temperature is no longer dominant.
 */
constexpr double kOnlineMemoryObservations = 64.0;

/**
 * Observations required before a solve is attempted.
 *
 * Six unknowns need six observations to be nominally determined; asking for
 * twelve means the system is over-determined by the time anyone acts on it.
 * Coverage is checked separately and is the harder condition by far.
 */
constexpr double kOnlineMinObservations = 12.0;

/**
 * Continuously estimates accelerometer bias and scale from ordinary use.
 *
 * The guided six-position flow asks the user to do something. This asks
 * nothing: whenever the tracker is at rest, the accelerometer is measuring
 * gravity, and gravity has a known magnitude. Every still moment is therefore a
 * constraint on the error model, and a tracker accumulates them by being worn,
 * set down, picked up and put away.
 *
 * What makes it fit on a microcontroller is that the batch fit's normal
 * equations are a *sum over samples* -- see `DiagonalNormalEquations`. There is
 * no history to keep. The entire state is twenty-seven numbers plus a little
 * coverage bookkeeping, fixed regardless of how long the tracker runs, and the
 * per-observation cost is a rank-1 update. This is the point on which an
 * on-device version of #5's option 4 turns: it is not a port of the offline
 * estimator, which re-integrates the whole capture per candidate, but it is
 * also not a new algorithm. It is the same algorithm with the loop turned
 * inside out.
 *
 * Samples must be *uncorrected*. The estimator measures the error the sample
 * path is about to remove; feeding it samples that already had a model applied
 * would estimate the residual of the current model and compound the two.
 */
class OnlineErrorEstimator {
public:
	/// Whether an observation was taken, and if not, why not.
	enum class Result : uint8_t {
		/// Sample consumed into the current block; nothing else happened.
		Accumulating,
		/// Not at rest, or the block was interrupted. Block discarded.
		NotAtRest,
		/// Block completed but the tracker had not moved since the last one.
		NotNovel,
		/// Block completed and became an observation.
		Observed,
	};

	/**
	 * Offers one accelerometer sample.
	 *
	 * @param accel   Scaled but uncorrected reading, same units as `norm`.
	 * @param atRest  Whether the fusion filter currently reports rest.
	 * @param norm    Expected magnitude, e.g. gravity in m/s^2.
	 */
	Result feed(const float accel[3], bool atRest, float norm) {
		if (!atRest) {
			blockCount = 0;
			return Result::NotAtRest;
		}

		blockSum[0] += accel[0];
		blockSum[1] += accel[1];
		blockSum[2] += accel[2];
		if (++blockCount < kOnlineBlockSamples) {
			return Result::Accumulating;
		}

		const float inverse = 1.0f / static_cast<float>(kOnlineBlockSamples);
		const float mean[3] = {
			blockSum[0] * inverse,
			blockSum[1] * inverse,
			blockSum[2] * inverse,
		};
		blockSum[0] = 0.0f;
		blockSum[1] = 0.0f;
		blockSum[2] = 0.0f;
		blockCount = 0;

		const float length
			= std::sqrt(mean[0] * mean[0] + mean[1] * mean[1] + mean[2] * mean[2]);
		if (!(length > 0.0f) || !(norm > 0.0f)) {
			return Result::NotAtRest;
		}

		// A steady reading that is not gravity is not a rest observation --
		// a sensor mid-range-change, or one being moved at a constant velocity
		// in a lift. Rest detection cannot tell the difference; magnitude can.
		if (std::fabs(length - norm) > kOnlineMagnitudeTolerance * norm) {
			return Result::NotAtRest;
		}

		const float direction[3]
			= {mean[0] / length, mean[1] / length, mean[2] / length};

		if (hasLastDirection) {
			const float alignment = direction[0] * lastDirection[0]
								  + direction[1] * lastDirection[1]
								  + direction[2] * lastDirection[2];
			if (alignment > kOnlineNoveltyCos) {
				return Result::NotNovel;
			}
		}

		for (int i = 0; i < 3; i++) {
			lastDirection[i] = direction[i];
		}
		hasLastDirection = true;

		// Forget before adding, so the new observation carries full weight and
		// everything already present is discounted relative to it.
		const double retain = 1.0 - 1.0 / kOnlineMemoryObservations;
		normal.forget(retain);
		normal.add(mean);

		for (int i = 0; i < 3; i++) {
			for (int j = i; j < 3; j++) {
				covariance[coverageIndex(i, j)]
					= covariance[coverageIndex(i, j)] * retain
					+ static_cast<double>(direction[i]) * direction[j];
			}
			positiveSeen[i] = positiveSeen[i] * retain
							+ (mean[i] > kMinAxisComponent * norm ? 1.0 : 0.0);
			negativeSeen[i] = negativeSeen[i] * retain
							+ (mean[i] < -kMinAxisComponent * norm ? 1.0 : 0.0);
		}
		coverageWeight = coverageWeight * retain + 1.0;

		return Result::Observed;
	}

	/// Effective number of observations, discounted by forgetting.
	[[nodiscard]] double observations() const { return coverageWeight; }

	/**
	 * Smallest eigenvalue of the direction covariance: 1/3 for isotropic
	 * coverage, 0 when every observation lay in one plane.
	 */
	[[nodiscard]] double directionSpread() const {
		if (coverageWeight <= 0.0) {
			return 0.0;
		}
		double cov[9];
		for (int i = 0; i < 3; i++) {
			for (int j = i; j < 3; j++) {
				const double v = covariance[coverageIndex(i, j)] / coverageWeight;
				cov[i * 3 + j] = v;
				cov[j * 3 + i] = v;
			}
		}
		return detail::smallestEigenvalue3(cov);
	}

	/**
	 * Whether every axis has been seen pointing both up and down.
	 *
	 * The hardest of the three conditions to satisfy from ordinary use, and the
	 * one that decides whether this converges at all -- see `isReady`.
	 */
	[[nodiscard]] bool eachAxisBothWays() const {
		// Half an observation of weight, so a single sighting still counts once
		// forgetting has eroded it a little but a long-decayed one does not.
		constexpr double seen = 0.5;
		for (int i = 0; i < 3; i++) {
			if (positiveSeen[i] < seen || negativeSeen[i] < seen) {
				return false;
			}
		}
		return true;
	}

	/**
	 * Whether enough varied observations have accumulated to trust a solve.
	 *
	 * Three conditions, and it is worth being clear which one bites. Count is
	 * nearly free. Spread is usually satisfied by any tracker that gets handled.
	 * **Both-ways on every axis is the real gate**, and for a tracker that is
	 * only ever worn it may never be satisfied: a shin tracker sees a narrow
	 * band of orientations, and no amount of walking will point its -Z at the
	 * sky. Convergence therefore depends on the tracker being taken off, set
	 * down, charged and stored -- which happens daily, but is not walking.
	 *
	 * That is a real limitation rather than a tuning problem, and it is why
	 * this supplements the guided flow rather than replacing it.
	 */
	[[nodiscard]] bool isReady() const {
		return coverageWeight >= kOnlineMinObservations
			&& directionSpread() >= kMinDirectionSpread && eachAxisBothWays();
	}

	/**
	 * Solves for the current best estimate.
	 *
	 * Refuses unless `isReady`, so a caller cannot get a confident answer from
	 * coverage that does not support one.
	 */
	[[nodiscard]] bool solve(float norm, ErrorModel& out) const {
		if (!isReady()) {
			return false;
		}
		return normal.solve(norm, out);
	}

	void reset() { *this = OnlineErrorEstimator{}; }

private:
	static constexpr int coverageIndex(int i, int j) {
		return i * 3 - (i * (i - 1)) / 2 + (j - i);
	}

	/// A steady reading this far from gravity is not gravity.
	static constexpr float kOnlineMagnitudeTolerance = 0.25f;

	DiagonalNormalEquations normal;

	/// Symmetric 3x3, upper triangle packed by rows.
	double covariance[6] = {0};
	double positiveSeen[3] = {0, 0, 0};
	double negativeSeen[3] = {0, 0, 0};
	double coverageWeight = 0.0;

	float lastDirection[3] = {0, 0, 0};
	bool hasLastDirection = false;

	float blockSum[3] = {0.0f, 0.0f, 0.0f};
	size_t blockCount = 0;
};

}  // namespace SlimeVR::Sensors::SoftFusion
