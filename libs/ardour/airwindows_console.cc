/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/* The transfer functions in this file are ported from the Airwindows
 * Console plugins (PurestConsole, Console6 and Console7, Channel and Buss),
 * https://github.com/airwindows/airwindows
 *
 * MIT License
 *
 * Copyright (c) 2018 Chris Johnson
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Airwindows Console is an encode/decode pair: the Channel model goes on
 * every source (track, or sub-bus feeding another bus) and the Buss model on
 * the bus they are summed into. The Buss decode is the inverse of the
 * Channel encode, so a single source passes (nearly) unchanged, while the
 * sum of several sources picks up the console character.
 *
 * Differences to the plugins: the plugins' gain knobs are fixed at their
 * defaults (use the Ardour fader, drive and auto-trim instead), and the
 * plugins' 32-bit floating point dither is omitted.
 */

#include <cmath>
#include <vector>

#include "ardour/summing_model.h"

using namespace ARDOUR;

namespace {

/** Stateless, per-sample transfer function applied to every channel */
template <double (*F) (double)>
class AirwindowsStateless : public SummingModel
{
public:
	void configure (uint32_t, samplecnt_t) {}

	void
	run (float* const* bufs, uint32_t n_channels, pframes_t n_samples)
	{
		for (uint32_t c = 0; c < n_channels; ++c) {
			float* b = bufs[c];
			for (pframes_t s = 0; s < n_samples; ++s) {
				b[s] = F (b[s]);
			}
		}
	}
};

/* PurestConsole: sin() encode, asin() decode */

static double
purest_channel (double x)
{
	return sin (x);
}

static double
purest_buss (double x)
{
	/* without the clamp, asin () returns NaN */
	if (x > 1.0) {
		x = 1.0;
	} else if (x < -1.0) {
		x = -1.0;
	}
	return asin (x);
}

/* Console6: inverse square 1-(1-x)^2 encode, 1-(1-x)^0.5 decode
 * (encode/decode courtesy of torridgristle under the MIT license)
 */

static double
console6_channel (double x)
{
	if (x > 1.0) {
		return 1.0;
	} else if (x > 0.0) {
		return x * (2.0 - x);
	} else if (x < -1.0) {
		return -1.0;
	} else if (x < 0.0) {
		return x * (x + 2.0);
	}
	return x;
}

static double
console6_buss (double x)
{
	if (x > 1.0) {
		return 1.0;
	} else if (x > 0.0) {
		return x / (1.0 + sqrt (1.0 - x));
	} else if (x < -1.0) {
		return -1.0;
	} else if (x < 0.0) {
		return x / (sqrt (x + 1.0) + 1.0);
	}
	return x;
}

/** Direct form I biquad lowpass, as used by Console7 */
struct Biquad {
	Biquad () : b0 (1), b1 (0), b2 (0), a1 (0), a2 (0), x1 (0), x2 (0), y1 (0), y2 (0) {}

	void
	lowpass (double freq, double q, double rate)
	{
		x1 = x2 = y1 = y2 = 0;
		double const f = freq / rate;
		if (f >= 0.49) {
			/* the plugin's 20kHz filter is only defined above ~41kHz sample rate */
			b0 = 1; b1 = b2 = a1 = a2 = 0;
			return;
		}
		double const K    = tan (M_PI * f);
		double const norm = 1.0 / (1.0 + K / q + K * K);
		b0 = K * K * norm;
		b1 = 2.0 * b0;
		b2 = b0;
		a1 = 2.0 * (K * K - 1.0) * norm;
		a2 = (1.0 - K / q + K * K) * norm;
	}

	double
	process (double x)
	{
		double const y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
		x2 = x1; x1 = x;
		y2 = y1; y1 = y;
		return y;
	}

	double b0, b1, b2, a1, a2;
	double x1, x2, y1, y2;
};

/* Console7: "Spiral" blended with plain sin/asin, and 20kHz lowpass filters.
 * Gain knobs fixed at the plugins' defaults (Channel 0.772, Buss 1.0),
 * which are unity gain for the whole system with a full scale sine.
 */

static double const phi       = 1.618033988749894848204586;
static double const phi_recip = 0.618033988749894848204586;

class AirwindowsConsole7Channel : public SummingModel
{
public:
	AirwindowsConsole7Channel () : _gain (0.772 * 1.272019649514069) {}

	void
	configure (uint32_t n_channels, samplecnt_t rate)
	{
		_lpf.assign (n_channels, Biquad ());
		for (auto& f : _lpf) {
			f.lowpass (20000.0, phi, rate);
		}
	}

	void
	run (float* const* bufs, uint32_t n_channels, pframes_t n_samples)
	{
		double const pre = _gain * _gain * _gain;
		for (uint32_t c = 0; c < n_channels; ++c) {
			Biquad& lpf (_lpf[c]);
			float*  b = bufs[c];
			for (pframes_t s = 0; s < n_samples; ++s) {
				double x = lpf.process (b[s]) * pre;
				/* the trim cuts back extra hard, and re-amplifies after the
				 * distortion; that makes faded settings slightly expanded */
				if (x > 1.097) {
					x = 1.097;
				} else if (x < -1.097) {
					x = -1.097;
				}
				double const ax = fabs (x);
				x = (sin (x * ax) / (ax == 0.0 ? 1.0 : ax)) * 0.8 + sin (x) * 0.2;
				b[s] = x / _gain;
			}
		}
	}

private:
	double              _gain;
	std::vector<Biquad> _lpf;
};

class AirwindowsConsole7Buss : public SummingModel
{
public:
	AirwindowsConsole7Buss () : _gain (sqrt (1.0 * 1.03)) {}

	void
	configure (uint32_t n_channels, samplecnt_t rate)
	{
		_lpf_in.assign (n_channels, Biquad ());
		_lpf_out.assign (n_channels, Biquad ());
		for (uint32_t c = 0; c < n_channels; ++c) {
			_lpf_in[c].lowpass (20000.0, phi_recip, rate);
			_lpf_out[c].lowpass (20000.0, 0.5, rate);
		}
	}

	void
	run (float* const* bufs, uint32_t n_channels, pframes_t n_samples)
	{
		for (uint32_t c = 0; c < n_channels; ++c) {
			Biquad& lpf_in (_lpf_in[c]);
			Biquad& lpf_out (_lpf_out[c]);
			float*  b = bufs[c];
			for (pframes_t s = 0; s < n_samples; ++s) {
				double x = lpf_in.process (b[s]) * _gain;
				if (x > 1.0) {
					x = 1.0;
				} else if (x < -1.0) {
					x = -1.0;
				}
				double const ax = fabs (x);
				x = (asin (x * ax) / (ax == 0.0 ? 1.0 : ax)) * phi_recip + asin (x) * (1.0 - phi_recip);
				b[s] = lpf_out.process (x) * _gain;
			}
		}
	}

private:
	double              _gain;
	std::vector<Biquad> _lpf_in;
	std::vector<Biquad> _lpf_out;
};

static struct AirwindowsRegistrar {
	AirwindowsRegistrar ()
	{
		SummingModel::register_model ("Airwindows PurestConsole Channel",
		                              [] () -> SummingModel* { return new AirwindowsStateless<purest_channel>; }, SummingChannel);
		SummingModel::register_model ("Airwindows PurestConsole Buss",
		                              [] () -> SummingModel* { return new AirwindowsStateless<purest_buss>; }, SummingBus);
		SummingModel::register_model ("Airwindows Console6 Channel",
		                              [] () -> SummingModel* { return new AirwindowsStateless<console6_channel>; }, SummingChannel);
		SummingModel::register_model ("Airwindows Console6 Buss",
		                              [] () -> SummingModel* { return new AirwindowsStateless<console6_buss>; }, SummingBus);
		SummingModel::register_model ("Airwindows Console7 Channel",
		                              [] () -> SummingModel* { return new AirwindowsConsole7Channel; }, SummingChannel);
		SummingModel::register_model ("Airwindows Console7 Buss",
		                              [] () -> SummingModel* { return new AirwindowsConsole7Buss; }, SummingBus);
	}
} airwindows_registrar;

} // anonymous namespace
