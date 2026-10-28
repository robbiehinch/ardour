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

#include <algorithm>
#include <cassert>
#include <cmath>

#include "ardour/dB.h"
#include "ardour/summing_model.h"
#include "ardour/summing_stage.h"

using namespace ARDOUR;

/* ****************************************************************************
 * Oversampler
 */

static double
bessel_i0 (double x)
{
	double sum  = 1.0;
	double term = 1.0;
	for (int k = 1; k < 64; ++k) {
		term *= (x / (2.0 * k)) * (x / (2.0 * k));
		sum += term;
		if (term < 1e-12 * sum) {
			break;
		}
	}
	return sum;
}

Oversampler::Oversampler ()
	: _factor (1)
{
}

void
Oversampler::configure (uint32_t factor, uint32_t n_channels)
{
	if (factor != 2 && factor != 4 && factor != 8 && factor != 16) {
		factor = 1;
	}
	_factor = factor;
	_chn.clear ();
	_h.clear ();

	if (_factor == 1) {
		return;
	}

	/* odd length, so that the round trip delay is an integer
	 * number of base-rate samples: (n_taps - 1) / factor
	 */
	uint32_t const n_taps = taps_per_phase * _factor + 1;
	double const   center = (n_taps - 1) * .5;
	double const   fc     = .5 / _factor; // base-rate Nyquist, relative to the high rate
	double const   beta   = 9.0;
	double const   i0b    = bessel_i0 (beta);

	_h.resize (n_taps);
	double sum = 0;
	for (uint32_t j = 0; j < n_taps; ++j) {
		double const t    = j - center;
		double const sinc = t == 0 ? 1.0 : sin (M_PI * 2.0 * fc * t) / (M_PI * 2.0 * fc * t);
		double const r    = t / center;
		double const win  = bessel_i0 (beta * sqrt (std::max (0.0, 1.0 - r * r))) / i0b;
		_h[j] = 2.0 * fc * sinc * win;
		sum += _h[j];
	}
	for (auto& h : _h) {
		h /= sum;
	}

	_chn.resize (n_channels);
	for (auto& c : _chn) {
		c.up.assign (2 * (taps_per_phase + 1), 0.f);
		c.down.assign (2 * n_taps, 0.f);
		c.up_pos   = 0;
		c.down_pos = 0;
	}
}

void
Oversampler::upsample (uint32_t chn, float const* in, float* out, pframes_t n)
{
	Channel&       c (_chn[chn]);
	uint32_t const L       = _factor;
	uint32_t const hist    = taps_per_phase + 1;
	uint32_t const n_taps  = _h.size ();
	float const    gain    = L; // compensate for zero-stuffing
	float const*   h       = &_h[0];

	for (pframes_t m = 0; m < n; ++m) {
		uint32_t const w = c.up_pos;
		c.up[w] = c.up[w + hist] = in[m];
		/* x[m - k] == x0[-k] */
		float const* x0 = &c.up[w + hist];
		for (uint32_t p = 0; p < L; ++p) {
			float acc = 0;
			for (uint32_t j = p, k = 0; j < n_taps; j += L, ++k) {
				acc += h[j] * x0[-(int)k];
			}
			out[m * L + p] = acc * gain;
		}
		c.up_pos = (w + 1) % hist;
	}
}

void
Oversampler::downsample (uint32_t chn, float const* in, float* out, pframes_t n)
{
	Channel&       c (_chn[chn]);
	uint32_t const L      = _factor;
	uint32_t const n_taps = _h.size ();
	float const*   h      = &_h[0];

	for (pframes_t m = 0; m < n; ++m) {
		for (uint32_t p = 0; p < L; ++p) {
			uint32_t const w = c.down_pos;
			c.down[w] = c.down[w + n_taps] = in[m * L + p];
			if (p == 0) {
				/* output aligned with the first high-rate sample of each
				 * group, so that the round trip delay is an integer */
				float const* z0  = &c.down[w + n_taps];
				float        acc = 0;
				for (uint32_t j = 0; j < n_taps; ++j) {
					acc += h[j] * z0[-(int)j];
				}
				out[m] = acc;
			}
			c.down_pos = (w + 1) % n_taps;
		}
	}
}

/* ****************************************************************************
 * SummingStage
 */

const float SummingStage::max_boost_db = 24.f;
const float SummingStage::max_cut_db   = 24.f;
const float SummingStage::release_sec  = 1.f;
const float SummingStage::smooth_sec   = .05f;

/** auto-trim reduces gain quickly, to catch transients */
static const float attack_sec = .002f;

SummingStage::SummingStage ()
	: _n_channels (0)
	, _max_block (0)
	, _rate (48000)
	, _drive (1.f)
	, _auto_trim (false)
	, _target (.5f)
	, _env (0)
	, _gain (1.f)
	, _release (0)
	, _smooth (1.f)
	, _min_gain (1.f)
	, _max_gain (1.f)
	, _gain_pos (0)
	, _extra_latency (0)
{
}

SummingStage::~SummingStage ()
{
}

void
SummingStage::configure (SummingModel* model, uint32_t n_channels, samplecnt_t sample_rate, pframes_t max_block,
                         uint32_t oversampling, samplecnt_t extra_latency)
{
	_model.reset (model);
	_n_channels    = n_channels;
	_max_block     = max_block;
	_rate          = sample_rate;
	_extra_latency = extra_latency;

	_os.configure (_model ? oversampling : 1, n_channels);

	uint32_t const L = _os.factor ();
	_os_bufs.assign (L > 1 ? n_channels : 0, std::vector<float> (max_block * L, 0.f));
	_os_ptrs.clear ();
	for (auto& b : _os_bufs) {
		_os_ptrs.push_back (&b[0]);
	}

	if (_model) {
		_model->configure (n_channels, sample_rate * L);
	}

	_release  = expf (-1.f / (release_sec * _rate));
	_smooth   = 1.f - expf (-1.f / (smooth_sec * _rate));
	_min_gain = dB_to_coefficient (-max_cut_db);
	_max_gain = dB_to_coefficient (max_boost_db);
	/* start from silence, so that the first signal sets the gain quickly */
	_env      = 0;
	_gain     = _drive;

	_gains.assign (max_block, _gain);
	_gain_delay.assign (latency (), _gain);
	_gain_pos = 0;
}

void
SummingStage::set_levels (float drive_db, bool auto_trim, float target_dbfs)
{
	_drive     = dB_to_coefficient (drive_db);
	_auto_trim = auto_trim;
	_target    = dB_to_coefficient (target_dbfs);
}

samplecnt_t
SummingStage::latency () const
{
	return _os.latency () + _extra_latency;
}

void
SummingStage::pre (float* const* bufs, uint32_t n_channels, pframes_t n_samples)
{
	assert (n_samples <= _max_block && n_channels <= _n_channels);
	float const attack = 1.f - expf (-1.f / (attack_sec * _rate));

	for (pframes_t s = 0; s < n_samples; ++s) {
		float target_gain = _drive;
		if (_auto_trim) {
			/* peak follower, instant attack, linked across channels */
			float peak = 0;
			for (uint32_t c = 0; c < n_channels; ++c) {
				peak = std::max (peak, fabsf (bufs[c][s]));
			}
			_env = peak > _env ? peak : _env * _release;
			target_gain *= std::min (_max_gain, std::max (_min_gain, _target / std::max (_env, 1e-9f)));
		}
		_gain += (target_gain < _gain ? attack : _smooth) * (target_gain - _gain);
		_gains[s] = _gain;
		for (uint32_t c = 0; c < n_channels; ++c) {
			bufs[c][s] *= _gain;
		}
	}
}

void
SummingStage::process (float* const* bufs, uint32_t n_channels, pframes_t n_samples)
{
	if (!_model) {
		return;
	}
	uint32_t const L = _os.factor ();
	if (L == 1) {
		_model->run (bufs, n_channels, n_samples);
		return;
	}
	for (uint32_t c = 0; c < n_channels; ++c) {
		_os.upsample (c, bufs[c], _os_ptrs[c], n_samples);
	}
	_model->run (&_os_ptrs[0], n_channels, n_samples * L);
	for (uint32_t c = 0; c < n_channels; ++c) {
		_os.downsample (c, _os_ptrs[c], bufs[c], n_samples);
	}
}

void
SummingStage::post (float* const* bufs, uint32_t n_channels, pframes_t n_samples)
{
	uint32_t const delay = _gain_delay.size ();

	for (pframes_t s = 0; s < n_samples; ++s) {
		float g = _gains[s];
		if (delay > 0) {
			float const d = _gain_delay[_gain_pos];
			_gain_delay[_gain_pos] = g;
			_gain_pos = (_gain_pos + 1) % delay;
			g = d;
		}
		float const inv = 1.f / g;
		for (uint32_t c = 0; c < n_channels; ++c) {
			bufs[c][s] *= inv;
		}
	}
}
