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

#pragma once

#include <memory>
#include <vector>

#include "ardour/libardour_visibility.h"
#include "ardour/types.h"

namespace ARDOUR {

class SummingModel;

/** Integer-factor polyphase FIR up/down sampler.
 *
 * Linear phase Kaiser-windowed sinc, flat to 20kHz with ~90dB image
 * rejection at 44.1kHz and above. A round trip (upsample, then downsample)
 * delays the signal by exactly latency() samples at the base rate.
 */
class LIBARDOUR_API Oversampler
{
public:
	Oversampler ();

	/** (re)initialize, not realtime safe. @param factor 1 (bypass), 2, 4, 8 or 16 */
	void configure (uint32_t factor, uint32_t n_channels);

	uint32_t    factor () const { return _factor; }
	samplecnt_t latency () const { return _factor > 1 ? taps_per_phase : 0; }

	/** @param out receives n * factor() samples */
	void upsample (uint32_t chn, float const* in, float* out, pframes_t n);
	/** @param in holds n * factor() samples */
	void downsample (uint32_t chn, float const* in, float* out, pframes_t n);

	static const uint32_t taps_per_phase = 64;

private:
	struct Channel {
		std::vector<float> up;   // base-rate history, mirrored for linear reads
		std::vector<float> down; // high-rate history, mirrored
		uint32_t           up_pos;
		uint32_t           down_pos;
	};

	uint32_t             _factor;
	std::vector<float>   _h;     // prototype lowpass, taps_per_phase * factor + 1 taps
	std::vector<Channel> _chn;
};

/** Gain staging and oversampling around a SummingModel (or another
 * processing step, such as a plugin).
 *
 * Processing is split in three steps, called in order for each cycle:
 *  - pre (): apply drive and auto-trim gain, remember the gain applied
 *  - process (): run the model, oversampled if configured; or the caller
 *    processes the (gained) buffers itself in between pre() and post()
 *  - post (): apply the inverse of the gain applied in pre (), delayed by
 *    the latency of the processing in between, so the stage's output
 *    level matches its input level and only the character remains.
 *
 * Auto-trim follows the input peak level with instant attack and a
 * release_sec release, and limits the gain to +max_boost_db/-max_cut_db.
 * When the processing has latency (oversampling, plugins), sudden level
 * changes cause a brief, small mismatch while the gain moves, because the
 * processing smears samples taken at different gains.
 */
class LIBARDOUR_API SummingStage
{
public:
	SummingStage ();
	~SummingStage ();

	/** Not realtime safe.
	 *
	 * @param model model to run in process(), may be null; the stage takes ownership
	 * @param oversampling oversampling factor for the model, 1 to disable
	 * @param extra_latency latency of external processing between pre() and post()
	 */
	void configure (SummingModel* model, uint32_t n_channels, samplecnt_t sample_rate, pframes_t max_block,
	                uint32_t oversampling, samplecnt_t extra_latency);

	/** Set level handling. Realtime safe, changes are smoothed.
	 *
	 * @param drive_db fixed gain into the model (compensated afterwards)
	 * @param auto_trim if true, additionally normalize the input's peak
	 *        level to \p target_dbfs before the model
	 * @param target_dbfs peak level to normalize to
	 */
	void set_levels (float drive_db, bool auto_trim, float target_dbfs);

	samplecnt_t latency () const;
	uint32_t    n_channels () const { return _n_channels; }
	pframes_t   max_block () const { return _max_block; }
	bool        has_model () const { return _model.get () != 0; }

	/* realtime */
	void pre (float* const* bufs, uint32_t n_channels, pframes_t n_samples);
	void process (float* const* bufs, uint32_t n_channels, pframes_t n_samples);
	void post (float* const* bufs, uint32_t n_channels, pframes_t n_samples);

	/** gain currently applied by pre(), for metering and tests */
	float current_gain () const { return _gain; }

	/* auto-trim limits and time constants */
	static const float max_boost_db;
	static const float max_cut_db;
	static const float release_sec;
	static const float smooth_sec;

private:
	std::unique_ptr<SummingModel> _model;
	Oversampler                   _os;

	uint32_t  _n_channels;
	pframes_t _max_block;
	float     _rate;

	std::vector<std::vector<float> > _os_bufs;
	std::vector<float*>              _os_ptrs;

	/* level handling */
	float _drive;
	bool  _auto_trim;
	float _target;
	float _env;
	float _gain;
	float _release;
	float _smooth;
	float _min_gain;
	float _max_gain;

	std::vector<float> _gains;       // per sample gain of the current cycle
	std::vector<float> _gain_delay;  // ring buffer, delays gains to match latency
	uint32_t           _gain_pos;
	samplecnt_t        _extra_latency;
};

} // namespace ARDOUR
