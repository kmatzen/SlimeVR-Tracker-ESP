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

// Which accelerometer calibration features are compiled in.
//
// Split into its own header because these guards reach code that has no
// business knowing how calibration works -- `sensor.h` needs one to decide
// whether to declare a virtual, and it should not have to include an estimator
// to find that out.
//
// Both are on by default and both are disabled together, because they share
// their cost: the same solve, the same soft-float `double` library that an
// ESP32-C3 has no FPU for, and the same boards with no room for either.
// `BOARD_GLOVE_IMU_SLIMEVR_DEV` builds every IMU driver into a 1280 kB
// partition and was 99.8% full before any of this existed.
//
// Separately named so they can be split later if a board ever wants the guided
// procedure without the continuous estimator, or the reverse.

#ifdef DISABLE_GUIDED_ACCEL_CALIBRATION
#define GUIDED_ACCEL_CALIBRATION 0
#define ONLINE_ACCEL_ESTIMATION 0
#else
#define GUIDED_ACCEL_CALIBRATION 1
#define ONLINE_ACCEL_ESTIMATION 1
#endif
