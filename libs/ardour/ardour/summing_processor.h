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

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "pbd/mutex.h"

#include "ardour/plugin_types.h"
#include "ardour/processor.h"
#include "ardour/summing_model.h"
#include "ardour/types.h"

namespace ARDOUR {

/** Internal processor for analog summing emulation.
 *
 * A route has up to two of these (see SummingRole): a channel stage before
 * the fader of tracks and sub-busses, and a bus stage on the summed input
 * of busses and the master bus.
 *
 * The processing is either a built-in SummingModel, or an arbitrary
 * plugin (e.g. an existing saturation plugin), with drive, auto-trim and
 * oversampling (built-in models only) around it.
 */
class LIBARDOUR_API SummingProcessor : public Processor
{
public:
	struct Settings {
		Settings () : drive_db (0), auto_trim (false), trim_target_db (-6), oversampling (1) {}

		/** built-in model name, plugin_model_id (), or empty for linear (bypass) */
		std::string model;
		/** preset to load, plugins only */
		std::string preset;
		/** gain into the model in dB, compensated after it */
		float drive_db;
		/** normalize the input peak level to trim_target_db before the model,
		 * and restore the original level after it */
		bool auto_trim;
		float trim_target_db;
		/** oversampling factor (1, 2, 4, 8 or 16), built-in models only */
		uint32_t oversampling;
	};

	SummingProcessor (Session&, SummingRole);
	~SummingProcessor ();

	SummingRole role () const { return _role; }

	bool display_to_user () const { return false; }
	void run (BufferSet& bufs, samplepos_t start_sample, samplepos_t end_sample, double speed, pframes_t nframes, bool result_required);
	bool configure_io (ChanCount in, ChanCount out);
	bool can_support_io_configuration (const ChanCount& in, ChanCount& out);
	int  set_block_size (pframes_t);
	samplecnt_t signal_latency () const { return _latency.load (); }

	/** Apply settings, (re)instantiates the model or plugin. Not realtime safe. */
	void     set_settings (Settings const&);
	Settings settings () const;

	/** Read this stage's settings from the session configuration */
	void apply_session_config ();

	/* convenience, keeps all other settings */
	void        set_model (std::string const& name);
	std::string model_name () const;

	/** Encode a plugin for use in Settings::model */
	static std::string plugin_model_id (PluginType, std::string const& unique_id);
	/** @return true if \p model refers to a plugin */
	static bool parse_plugin_model_id (std::string const& model, PluginType&, std::string& unique_id);

protected:
	XMLNode& state () const;

private:
	struct Engine;

	void    rebuild ();
	Engine* build (Settings const&, uint32_t n_channels) const;
	void    run_plugins (Engine&, BufferSet&, samplepos_t, double, pframes_t ns, pframes_t offset);

	SummingRole _role;

	mutable PBD::Mutex _settings_lock;
	Settings           _settings;

	/* serializes (re)building engines */
	PBD::Mutex _build_lock;

	/* protects _engine, the process thread only try-locks */
	mutable PBD::Mutex      _engine_lock;
	std::unique_ptr<Engine> _engine;

	std::atomic<uint32_t>    _n_channels;
	std::atomic<samplecnt_t> _latency;
};

} // namespace ARDOUR
