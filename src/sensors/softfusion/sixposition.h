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

// Free of any Arduino or hardware dependency so the state machine can be unit
// tested on a host. See tools/fusion-bench/tests/selftest.cpp.

namespace SlimeVR::Sensors::SoftFusion {

/// The six static orientations: each sensor axis pointing up in turn.
constexpr size_t kSixPositionCount = 6;

/**
 * Block means retained per position, and raw samples averaged into each block.
 *
 * The fit wants at least nine rows and gets `6 * kBlocksPerPosition = 24`. One
 * mean per position would be only six, which the quadric cannot determine --
 * so the choice is not arbitrary, it is the smallest split that keeps the
 * procedure at six positions while leaving the system over-determined.
 *
 * Blocks rather than raw samples because averaging is what makes a hand-held
 * capture usable: sensor noise is what the fit is fighting, and 32 samples is
 * about 5x down on it. Keeping every raw sample instead would cost kilobytes to
 * buy a fit that is no better.
 */
constexpr size_t kBlocksPerPosition = 4;
constexpr size_t kSamplesPerBlock = 32;

/// Consecutive in-tolerance samples required before a capture starts.
///
/// Rest detection has its own hold-off, but it reports "the tracker is not
/// moving", not "the tracker has been put down in the position I asked for".
/// This is the second condition: it stops a capture beginning during the
/// moment a hand is still settling into place.
constexpr size_t kDwellSamples = 24;

/**
 * How far off-axis a position may be held, as a cosine.
 *
 * 20 degrees. Deliberately looser than it needs to be for the fit's own sake:
 * `kMinDirectionSpread` already refuses a sample set that fails to span three
 * dimensions, so this threshold is not the safeguard, it is the prompt. Its job
 * is to tell a user holding the tracker at 45 degrees that they have not yet
 * done what was asked, and then get out of the way.
 */
constexpr float kAxisAlignmentCos = 0.9397f;

/// Accepted magnitude band around the expected norm, as a fraction.
///
/// Rest detection already rejects a tracker in motion; this catches the
/// remaining case of a steady but non-gravitational reading -- a sensor
/// mid-range-change, or a part so far out of spec that fitting it would be
/// meaningless. Real scale error is a few percent at worst, so 25% cannot
/// reject a part this procedure was meant to fix.
constexpr float kMagnitudeTolerance = 0.25f;

/// What `feed` did with the sample it was given.
enum class SixPositionEvent : uint8_t {
	/// Nothing worth reporting.
	None,
	/// A capture just started for `activePosition()`.
	Started,
	/// A capture just finished; positions remain.
	Captured,
	/// A capture in progress was abandoned -- the tracker moved.
	Disturbed,
	/// The final position was captured. The fit can now be run.
	Complete,
};

/**
 * Collects the six static orientations the accelerometer error-model fit needs.
 *
 * The estimator in `errormodel.h` has been able to recover accelerometer bias,
 * scale and misalignment since it landed, but only from a capture replayed
 * through a host tool. This is the missing half: the part that lets a tracker
 * gather that data by itself, while a user turns it over in their hands.
 *
 * The procedure is the classic one -- hold each axis up in turn, keeping still
 * -- and the collector's whole job is to decide when "still, in a position not
 * yet seen" is true, because that is the judgement a user cannot make for
 * themselves through a serial log.
 *
 * Time is deliberately absent from this class. It counts samples, not
 * milliseconds, so it is exercisable on a host at any rate and carries no
 * dependency on the Arduino clock. Timeouts belong to the caller.
 */
class SixPositionCollector {
public:
	enum class State : uint8_t {
		/// Not running.
		Idle,
		/// Running, waiting for the tracker to settle into an uncaptured
		/// position.
		Waiting,
		/// Running, accumulating samples for `activePosition()`.
		Capturing,
		/// All six positions captured; `fit` will succeed if the data allows.
		Complete,
	};

	/// Starts, or restarts, the procedure. Any partial progress is discarded.
	void begin() {
		*this = SixPositionCollector{};
		state = State::Waiting;
	}

	void abort() { state = State::Idle; }

	[[nodiscard]] State getState() const { return state; }
	[[nodiscard]] bool isRunning() const {
		return state == State::Waiting || state == State::Capturing;
	}

	/// Bitmask of captured positions, bit `i` for position `i`.
	[[nodiscard]] uint8_t capturedMask() const { return captured; }
	[[nodiscard]] size_t capturedCount() const {
		size_t n = 0;
		for (size_t i = 0; i < kSixPositionCount; i++) {
			if ((captured & (1u << i)) != 0) {
				n++;
			}
		}
		return n;
	}

	/// Position currently being captured, or -1 when not capturing.
	[[nodiscard]] int activePosition() const {
		return state == State::Capturing ? activeIndex : -1;
	}

	/// First position not yet captured, or -1 when all are done.
	[[nodiscard]] int nextPosition() const {
		for (size_t i = 0; i < kSixPositionCount; i++) {
			if ((captured & (1u << i)) == 0) {
				return static_cast<int>(i);
			}
		}
		return -1;
	}

	/// Blocks stored for the position being captured, of `kBlocksPerPosition`.
	[[nodiscard]] size_t activeProgress() const {
		return state == State::Capturing ? blocksFilled : 0;
	}

	/**
	 * Human-readable name of a position, e.g. `"+Z up"`.
	 *
	 * Named for the sensor axis that ends up pointing at the sky, because that
	 * is the one a user can act on. A stationary accelerometer reads positive
	 * on whichever axis points up, so `"+Z up"` is also literally the axis the
	 * reading lands on -- the prompt and the data agree, which makes a log
	 * legible when something goes wrong.
	 */
	static const char* positionName(int index) {
		static const char* names[kSixPositionCount]
			= {"+X up", "-X up", "+Y up", "-Y up", "+Z up", "-Z up"};
		if (index < 0 || static_cast<size_t>(index) >= kSixPositionCount) {
			return "?";
		}
		return names[index];
	}

	/**
	 * Classifies a reading into one of the six positions.
	 *
	 * @param norm  Expected magnitude, e.g. gravity in m/s^2.
	 * @return Position index, or -1 if the reading is not close enough to any
	 *         axis or its magnitude is not close enough to `norm`.
	 */
	static int classify(const float accel[3], float norm) {
		const float len = std::sqrt(
			accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]
		);
		if (!(len > 0.0f) || !(norm > 0.0f)) {
			return -1;
		}
		if (std::fabs(len - norm) > kMagnitudeTolerance * norm) {
			return -1;
		}

		size_t axis = 0;
		for (size_t i = 1; i < 3; i++) {
			if (std::fabs(accel[i]) > std::fabs(accel[axis])) {
				axis = i;
			}
		}
		if (std::fabs(accel[axis]) / len < kAxisAlignmentCos) {
			return -1;
		}
		return static_cast<int>(axis) * 2 + (accel[axis] > 0.0f ? 0 : 1);
	}

	/**
	 * Offers one accelerometer sample to the procedure.
	 *
	 * @param accel   Scaled but *uncorrected* reading, in the same units as
	 *                `norm`. Uncorrected matters: the fit measures the error
	 *                the sample path is about to remove, so feeding it
	 *                already-corrected samples would fit the residual of
	 *                whatever model is loaded and quietly compound it.
	 * @param atRest  Whether the fusion filter currently reports rest.
	 * @param norm    Expected magnitude, e.g. gravity in m/s^2.
	 */
	SixPositionEvent feed(const float accel[3], bool atRest, float norm) {
		if (!isRunning()) {
			return SixPositionEvent::None;
		}

		const int position = classify(accel, norm);

		if (state == State::Capturing) {
			if (!atRest || position != activeIndex) {
				state = State::Waiting;
				dwellIndex = -1;
				dwellCount = 0;
				return SixPositionEvent::Disturbed;
			}
			return accumulate(accel);
		}

		// Waiting. An already-captured position is not a candidate, which is
		// also what stops the tracker being captured six times over without
		// moving: the position it was just captured in is no longer eligible.
		if (!atRest || position < 0 || (captured & (1u << position)) != 0) {
			dwellIndex = -1;
			dwellCount = 0;
			return SixPositionEvent::None;
		}

		if (position != dwellIndex) {
			dwellIndex = position;
			dwellCount = 1;
			return SixPositionEvent::None;
		}

		if (++dwellCount < kDwellSamples) {
			return SixPositionEvent::None;
		}

		state = State::Capturing;
		activeIndex = position;
		blocksFilled = 0;
		resetBlock();
		return SixPositionEvent::Started;
	}

	/**
	 * Fits bias and scale to the collected positions.
	 *
	 * Diagonal, not the full model, and that is a deliberate limit rather than
	 * a shortcut -- see `fitErrorModelDiagonal`. Six axis-aligned positions
	 * determine bias and scale exactly and misalignment not at all, so fitting
	 * the full matrix here would be reporting hand-placement error as a sensor
	 * property. Misalignment stays with the host tool, where a capture can
	 * cover orientations that actually observe it.
	 *
	 * Only callable once complete, and still able to refuse: the coverage check
	 * is independent of this class's own tolerance.
	 *
	 * @param norm  Expected magnitude, e.g. gravity in m/s^2.
	 */
	[[nodiscard]] bool fit(float norm, ErrorModel& out) const {
		if (state != State::Complete) {
			return false;
		}
		return fitErrorModelDiagonal(
			&blocks[0][0][0],
			kSixPositionCount * kBlocksPerPosition,
			norm,
			out
		);
	}

	/// The collected block means, `6 * kBlocksPerPosition` rows of xyz.
	[[nodiscard]] const float* samples() const { return &blocks[0][0][0]; }
	[[nodiscard]] static size_t sampleCount() {
		return kSixPositionCount * kBlocksPerPosition;
	}

private:
	void resetBlock() {
		blockSum[0] = 0.0f;
		blockSum[1] = 0.0f;
		blockSum[2] = 0.0f;
		blockCount = 0;
	}

	SixPositionEvent accumulate(const float accel[3]) {
		blockSum[0] += accel[0];
		blockSum[1] += accel[1];
		blockSum[2] += accel[2];
		if (++blockCount < kSamplesPerBlock) {
			return SixPositionEvent::None;
		}

		const float inverse = 1.0f / static_cast<float>(kSamplesPerBlock);
		for (size_t axis = 0; axis < 3; axis++) {
			blocks[activeIndex][blocksFilled][axis] = blockSum[axis] * inverse;
		}
		resetBlock();

		if (++blocksFilled < kBlocksPerPosition) {
			return SixPositionEvent::None;
		}

		captured |= static_cast<uint8_t>(1u << activeIndex);
		dwellIndex = -1;
		dwellCount = 0;

		if (capturedCount() == kSixPositionCount) {
			state = State::Complete;
			return SixPositionEvent::Complete;
		}
		state = State::Waiting;
		return SixPositionEvent::Captured;
	}

	State state = State::Idle;
	uint8_t captured = 0;

	int activeIndex = -1;
	size_t blocksFilled = 0;

	int dwellIndex = -1;
	size_t dwellCount = 0;

	float blockSum[3] = {0.0f, 0.0f, 0.0f};
	size_t blockCount = 0;

	float blocks[kSixPositionCount][kBlocksPerPosition][3] = {};
};

}  // namespace SlimeVR::Sensors::SoftFusion
