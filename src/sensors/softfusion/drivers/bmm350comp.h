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

#include <cstdint>

// BMM350 OTP trim decoding and magnetic compensation.
//
// Ported from Bosch's BMM350_SensorAPI (bmm350.c, update_mag_off_sens and
// bmm350_get_compensated_mag_xyz_temp_data). Deliberately free of any hardware
// dependency so the arithmetic can be unit tested on a host -- see
// tools/fusion-bench/tests/selftest.cpp.
//
// Why this matters for a tracker rather than being optional polish: the
// per-axis sensitivity and the cross-axis terms change the *direction* of the
// measured field vector, not just its magnitude. Direction is the entire signal
// as far as heading is concerned, so an uncompensated part gives a heading
// error that varies with orientation -- which is exactly the kind of error a
// filter cannot average away.

namespace SlimeVR::Sensors::SoftFusion {

struct Bmm350Calibration {
	// Per-axis offset, in raw LSB.
	float offsetX = 0, offsetY = 0, offsetZ = 0;
	// Per-axis sensitivity error, as a fraction (0 means nominal).
	float sensX = 0, sensY = 0, sensZ = 0;
	// Temperature channel offset and sensitivity.
	float tOffs = 0, tSens = 0;
	// Temperature coefficient of offset, per degree.
	float tcoX = 0, tcoY = 0, tcoZ = 0;
	// Temperature coefficient of sensitivity, per degree.
	float tcsX = 0, tcsY = 0, tcsZ = 0;
	// Reference temperature the coefficients are defined against.
	float dutT0 = 23.0f;
	// Cross-axis coupling.
	float crossXY = 0, crossYX = 0, crossZX = 0, crossZY = 0;

	bool valid = false;
};

/// Raw-count to physical-unit scaling, from the vendor API.
constexpr float kBmm350LsbToUtXY = 0.007069979f;
constexpr float kBmm350LsbToUtZ = 0.007174964f;
constexpr float kBmm350LsbToDegC = 0.000981282f;
/// Piecewise offset applied to the temperature channel.
constexpr float kBmm350TempOffset = 25.49f;

/// Number of OTP words the trim data occupies.
constexpr uint8_t kBmm350OtpWordCount = 32;

/**
 * Two's-complement conversion for a value held in the low `bits` of `value`.
 *
 * The OTP packs signed coefficients into 8- and 12-bit fields, so they have to
 * be sign-extended by hand before use. Getting this wrong flips the sign of
 * roughly half the trim values and is invisible without a reference.
 */
inline int32_t bmm350FixSign(uint32_t value, uint8_t bits) {
	const uint32_t signBit = 1u << (bits - 1);
	if (value & signBit) {
		// Subtract 2^bits to reinterpret as negative.
		return static_cast<int32_t>(value) - static_cast<int32_t>(1u << bits);
	}
	return static_cast<int32_t>(value);
}

/**
 * Decodes the 32 OTP words into usable coefficients.
 *
 * Word indices, field positions and scale divisors follow
 * BMM350_SensorAPI's update_mag_off_sens exactly.
 */
inline void bmm350DecodeOtp(const uint16_t* otp, Bmm350Calibration& out) {
	const uint16_t w0 = otp[0];
	const uint16_t w1 = otp[1];
	const uint16_t w2 = otp[2];

	out.offsetX = static_cast<float>(bmm350FixSign(w0 & 0x0fffu, 12));

	// offset_y spans the top nibble of word 0 and the low byte of word 1.
	uint32_t offYRaw = static_cast<uint32_t>((w0 & 0xf000u) >> 4)
					 | static_cast<uint32_t>(w1 & 0x00ffu);
	out.offsetY = static_cast<float>(bmm350FixSign(offYRaw, 12));

	// offset_z spans the second nibble of word 1 and the low byte of word 2.
	uint32_t offZRaw
		= static_cast<uint32_t>(w1 & 0x0f00u) | static_cast<uint32_t>(w2 & 0x00ffu);
	out.offsetZ = static_cast<float>(bmm350FixSign(offZRaw, 12));

	out.tOffs = static_cast<float>(bmm350FixSign(otp[3] & 0x00ffu, 8)) / 5.0f;
	out.tSens = static_cast<float>(bmm350FixSign((otp[3] & 0xff00u) >> 8, 8)) / 512.0f;

	out.sensX = static_cast<float>(bmm350FixSign((otp[4] & 0xff00u) >> 8, 8)) / 256.0f;
	out.sensY = static_cast<float>(bmm350FixSign(otp[5] & 0x00ffu, 8)) / 256.0f;
	out.sensZ = static_cast<float>(bmm350FixSign((otp[6] & 0xff00u) >> 8, 8)) / 256.0f;

	out.tcoX = static_cast<float>(bmm350FixSign(otp[7] & 0x00ffu, 8)) / 32.0f;
	out.tcoY = static_cast<float>(bmm350FixSign(otp[8] & 0x00ffu, 8)) / 32.0f;
	out.tcoZ = static_cast<float>(bmm350FixSign(otp[9] & 0x00ffu, 8)) / 32.0f;

	out.tcsX
		= static_cast<float>(bmm350FixSign((otp[10] & 0xff00u) >> 8, 8)) / 16384.0f;
	out.tcsY
		= static_cast<float>(bmm350FixSign((otp[11] & 0xff00u) >> 8, 8)) / 16384.0f;
	out.tcsZ
		= static_cast<float>(bmm350FixSign((otp[12] & 0xff00u) >> 8, 8)) / 16384.0f;

	out.dutT0 = static_cast<float>(bmm350FixSign(otp[13], 16)) / 512.0f + 23.0f;

	out.crossXY = static_cast<float>(bmm350FixSign(otp[14] & 0x00ffu, 8)) / 800.0f;
	out.crossYX
		= static_cast<float>(bmm350FixSign((otp[15] & 0xff00u) >> 8, 8)) / 800.0f;
	out.crossZX = static_cast<float>(bmm350FixSign(otp[16] & 0x00ffu, 8)) / 800.0f;
	out.crossZY
		= static_cast<float>(bmm350FixSign((otp[17] & 0xff00u) >> 8, 8)) / 800.0f;

	out.valid = true;
}

/** Converts the raw temperature channel to degrees Celsius. */
inline float bmm350RawToCelsius(int32_t rawTemp) {
	const float scaled = static_cast<float>(rawTemp) * kBmm350LsbToDegC;
	if (scaled > 0.0f) {
		return scaled - kBmm350TempOffset;
	}
	if (scaled < 0.0f) {
		return scaled + kBmm350TempOffset;
	}
	return 0.0f;
}

/**
 * Applies offset, sensitivity, temperature and cross-axis compensation.
 *
 * `rawXyz` are the raw 24-bit magnetometer counts; `tempC` is the die
 * temperature in degrees Celsius. Output is microtesla.
 *
 * When the temperature channel is not being read, pass `cal.dutT0`: the TCO and
 * TCS terms are both defined relative to it, so they vanish and the result is
 * the offset/sensitivity/cross-axis compensation alone. That is a real
 * degradation, not a no-op, but it is a well-defined one -- and the terms it
 * drops are the second-order ones.
 */
inline void bmm350Compensate(
	const int32_t* rawXyz,
	float tempC,
	const Bmm350Calibration& cal,
	float* outUt
) {
	if (!cal.valid) {
		// No trim data: fall back to nominal scaling so a failed OTP read
		// degrades to "uncompensated" rather than to "zero".
		outUt[0] = static_cast<float>(rawXyz[0]) * kBmm350LsbToUtXY;
		outUt[1] = static_cast<float>(rawXyz[1]) * kBmm350LsbToUtXY;
		outUt[2] = static_cast<float>(rawXyz[2]) * kBmm350LsbToUtZ;
		return;
	}

	float out[3] = {
		static_cast<float>(rawXyz[0]) * kBmm350LsbToUtXY,
		static_cast<float>(rawXyz[1]) * kBmm350LsbToUtXY,
		static_cast<float>(rawXyz[2]) * kBmm350LsbToUtZ,
	};

	const float sens[3] = {cal.sensX, cal.sensY, cal.sensZ};
	const float offset[3] = {cal.offsetX, cal.offsetY, cal.offsetZ};
	const float tco[3] = {cal.tcoX, cal.tcoY, cal.tcoZ};
	const float tcs[3] = {cal.tcsX, cal.tcsY, cal.tcsZ};

	const float dT = tempC - cal.dutT0;

	for (int i = 0; i < 3; i++) {
		out[i] *= 1.0f + sens[i];
		out[i] += offset[i];
		out[i] += tco[i] * dT;
		const float tcsDenom = 1.0f + tcs[i] * dT;
		if (tcsDenom != 0.0f) {
			out[i] /= tcsDenom;
		}
	}

	// Orthogonalise. The denominator is 1 for an uncoupled part, so this
	// reduces to a pass-through when the cross terms are zero.
	const float denom = 1.0f - cal.crossYX * cal.crossXY;
	if (denom == 0.0f) {
		outUt[0] = out[0];
		outUt[1] = out[1];
		outUt[2] = out[2];
		return;
	}

	outUt[0] = (out[0] - cal.crossXY * out[1]) / denom;
	outUt[1] = (out[1] - cal.crossYX * out[0]) / denom;
	outUt[2] = out[2]
			 + (out[0] * (cal.crossYX * cal.crossZY - cal.crossZX)
				- out[1] * (cal.crossZY - cal.crossXY * cal.crossZX))
				   / denom;
}

}  // namespace SlimeVR::Sensors::SoftFusion
