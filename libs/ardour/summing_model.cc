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

#include <cmath>
#include <mutex>

#include "ardour/summing_model.h"

using namespace ARDOUR;

namespace {

struct ModelEntry {
	std::string           name;
	SummingModel::Factory factory;
	int                   roles;
};

static std::mutex&
registry_lock ()
{
	static std::mutex m;
	return m;
}

/** Example model of a classic analog console summing stage.
 *
 * Per channel pair (L/R):
 *  - capacitive crosstalk between adjacent channels (rising 6dB/oct, -80dB @ 1kHz)
 *  - summing amplifier with 16dB headroom above 0dBFS (tanh), with a small
 *    bias for asymmetric saturation (adds 2nd harmonic at moderate levels)
 *  - output coupling capacitor (5Hz high-pass), which also removes the DC
 *    offset introduced by the asymmetric saturation.
 *
 * Small-signal gain is unity, so at low levels the model is transparent.
 * With 16dB of headroom it is subtle at normal mix levels; use the stage's
 * drive or auto-trim to push it harder.
 */
class AnalogConsoleSumming : public SummingModel
{
public:
	void
	configure (uint32_t n_channels, samplecnt_t sample_rate)
	{
		_chn.assign (n_channels, Channel ());

		float const rate = sample_rate;

		_crosstalk = 1e-4f * rate / (2.f * M_PI * 1000.f);
		_dc_r      = 1.f - 2.f * M_PI * 5.f / rate;

		_drive      = powf (10.f, -16.f / 20.f);
		_bias       = 0.05f;
		_bias_tanh  = tanhf (_bias);
		_norm       = 1.f / (_drive * (1.f - _bias_tanh * _bias_tanh));
	}

	void
	run (float* const* bufs, uint32_t n_channels, pframes_t n_samples)
	{
		uint32_t c = 0;
		for (; c + 1 < n_channels; c += 2) {
			Channel& l (_chn[c]);
			Channel& r (_chn[c + 1]);
			float* bl = bufs[c];
			float* br = bufs[c + 1];
			for (pframes_t s = 0; s < n_samples; ++s) {
				float const xl = bl[s];
				float const xr = br[s];
				bl[s] = process (l, xl + _crosstalk * (xr - r.x1));
				br[s] = process (r, xr + _crosstalk * (xl - l.x1));
				l.x1 = xl;
				r.x1 = xr;
			}
		}
		if (c < n_channels) {
			/* odd channel count, last channel has no neighbor */
			Channel& m (_chn[c]);
			float* b = bufs[c];
			for (pframes_t s = 0; s < n_samples; ++s) {
				b[s] = process (m, b[s]);
			}
		}
	}

private:
	struct Channel {
		Channel () : x1 (0), dc_x1 (0), dc_y1 (0) {}
		float x1;    // previous input sample (crosstalk differentiator)
		float dc_x1; // DC blocker state
		float dc_y1;
	};

	float
	process (Channel& ch, float x)
	{
		/* summing amp */
		float const s = (tanhf (_drive * x + _bias) - _bias_tanh) * _norm;
		/* output coupling cap */
		float const y = s - ch.dc_x1 + _dc_r * ch.dc_y1;
		ch.dc_x1 = s;
		ch.dc_y1 = y;
		return y;
	}

	std::vector<Channel> _chn;

	float _crosstalk;
	float _dc_r;
	float _drive;
	float _bias;
	float _bias_tanh;
	float _norm;
};

/* function-local static, so that models can register from static
 * initializers in other translation units, regardless of init order.
 */
static std::vector<ModelEntry>&
registry ()
{
	static std::vector<ModelEntry> r {
		{ "Analog Console", [] () -> SummingModel* { return new AnalogConsoleSumming; }, SummingAnyRole },
	};
	return r;
}

} // anonymous namespace

bool
SummingModel::register_model (std::string const& name, Factory factory, int roles)
{
	if (name.empty () || !factory || !(roles & SummingAnyRole)) {
		return false;
	}

	std::lock_guard<std::mutex> lm (registry_lock ());
	for (auto const& e : registry ()) {
		if (e.name == name) {
			return false;
		}
	}
	registry ().push_back (ModelEntry { name, factory, roles });
	return true;
}

SummingModel*
SummingModel::create (std::string const& name)
{
	std::lock_guard<std::mutex> lm (registry_lock ());
	for (auto const& e : registry ()) {
		if (e.name == name) {
			return e.factory ();
		}
	}
	return 0;
}

std::vector<std::string>
SummingModel::available_models (int role)
{
	std::vector<std::string> rv;
	std::lock_guard<std::mutex> lm (registry_lock ());
	for (auto const& e : registry ()) {
		if (e.roles & role) {
			rv.push_back (e.name);
		}
	}
	return rv;
}
