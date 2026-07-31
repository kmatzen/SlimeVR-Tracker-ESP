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

// Whether raw sample streaming (#23) is compiled in.
//
// Split out for the same reason `calibrationfeatures.h` is: the guard reaches
// code that has no business knowing how the streamer works.
//
// On by default. Disabled only where there is no flash for it --
// `BOARD_GLOVE_IMU_SLIMEVR_DEV` builds every IMU driver into a 1280 kB
// partition and had 1978 bytes spare before this existed.
//
// **This feature deliberately adds no `SensorConfig` field.** Issue #8 measured
// roughly 350 bytes lost per feature on that board *even with the feature
// compiled out*, and identified the residual as `SensorConfig` growth -- which
// cannot be guarded away, because the config layout has to stay common across
// boards or calibrations stop being portable between builds. Streaming is
// started and stopped by a command from the server and nothing about it is
// persisted, so a board that compiles it out pays nothing at all.
//
// That is also the honest model of what this is: you turn capture on for a
// session, not forever.

#ifdef DISABLE_RAW_SAMPLE_STREAMING
#define RAW_SAMPLE_STREAMING 0
#else
#define RAW_SAMPLE_STREAMING 1
#endif
