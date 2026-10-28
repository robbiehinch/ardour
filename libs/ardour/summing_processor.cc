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

#include "pbd/enumwriter.h"
#include "pbd/error.h"

#include "ardour/audio_buffer.h"
#include "ardour/audioengine.h"
#include "ardour/buffer_set.h"
#include "ardour/chan_mapping.h"
#include "ardour/lua_api.h"
#include "ardour/plugin.h"
#include "ardour/route.h"
#include "ardour/session.h"
#include "ardour/summing_model.h"
#include "ardour/summing_processor.h"
#include "ardour/summing_stage.h"

#include "pbd/i18n.h"

using namespace ARDOUR;
using namespace PBD;

static const std::string plugin_prefix = "plugin:";

struct SummingProcessor::Engine {
	Engine () : plugin_in (0), plugin_out (0), replicated (false) {}

	~Engine ()
	{
		for (auto const& p : plugins) {
			p->deactivate ();
		}
	}

	SummingStage                          stage;
	std::vector<std::shared_ptr<Plugin> > plugins;
	uint32_t                              plugin_in;  // audio pins per instance
	uint32_t                              plugin_out;
	bool                                  replicated; // one mono instance per channel
	BufferSet                             plugin_bufs; // plugin I/O: inputs, then outputs
	std::vector<float*>                   ptrs;
};

SummingProcessor::SummingProcessor (Session& s, SummingRole role)
	: Processor (s, role == SummingChannel ? "Channel Summing" : "Bus Summing", Temporal::TimeDomainProvider (Temporal::AudioTime))
	, _role (role)
	, _n_channels (0)
	, _latency (0)
{
}

SummingProcessor::~SummingProcessor ()
{
}

std::string
SummingProcessor::plugin_model_id (PluginType type, std::string const& unique_id)
{
	return plugin_prefix + enum_2_string (type) + ":" + unique_id;
}

bool
SummingProcessor::parse_plugin_model_id (std::string const& model, PluginType& type, std::string& unique_id)
{
	if (model.compare (0, plugin_prefix.size (), plugin_prefix) != 0) {
		return false;
	}
	/* the type contains no colon, the unique-id (e.g. an LV2 URI) may */
	std::string::size_type const sep = model.find (':', plugin_prefix.size ());
	if (sep == std::string::npos) {
		return false;
	}
	type      = (PluginType) string_2_enum (model.substr (plugin_prefix.size (), sep - plugin_prefix.size ()), type);
	unique_id = model.substr (sep + 1);
	return true;
}

bool
SummingProcessor::can_support_io_configuration (const ChanCount& in, ChanCount& out)
{
	out = in;
	return true;
}

bool
SummingProcessor::configure_io (ChanCount in, ChanCount out)
{
	if (out != in) { // always 1:1
		return false;
	}

	if (_n_channels.exchange (in.n_audio ()) != in.n_audio ()) {
		rebuild ();
	}

	return Processor::configure_io (in, out);
}

int
SummingProcessor::set_block_size (pframes_t)
{
	rebuild ();
	return 0;
}

void
SummingProcessor::set_settings (Settings const& s)
{
	{
		PBD::Mutex::Lock lm (_settings_lock);
		_settings = s;
	}
	rebuild ();
}

SummingProcessor::Settings
SummingProcessor::settings () const
{
	PBD::Mutex::Lock lm (_settings_lock);
	return _settings;
}

void
SummingProcessor::set_model (std::string const& name)
{
	Settings s (settings ());
	s.model = name;
	set_settings (s);
}

std::string
SummingProcessor::model_name () const
{
	return settings ().model;
}

void
SummingProcessor::apply_session_config ()
{
	SessionConfiguration const& c (_session.config);
	Settings s;
	if (_role == SummingChannel) {
		s.model          = c.get_summing_channel_model ();
		s.preset         = c.get_summing_channel_preset ();
		s.drive_db       = c.get_summing_channel_drive ();
		s.auto_trim      = c.get_summing_channel_auto_trim ();
		s.trim_target_db = c.get_summing_channel_trim_target ();
		s.oversampling   = c.get_summing_channel_oversampling ();
	} else {
		s.model          = c.get_summing_bus_model ();
		s.preset         = c.get_summing_bus_preset ();
		s.drive_db       = c.get_summing_bus_drive ();
		s.auto_trim      = c.get_summing_bus_auto_trim ();
		s.trim_target_db = c.get_summing_bus_trim_target ();
		s.oversampling   = c.get_summing_bus_oversampling ();
	}
	set_settings (s);
}

void
SummingProcessor::rebuild ()
{
	PBD::Mutex::Lock bl (_build_lock);

	uint32_t const n_chn = _n_channels.load ();
	std::unique_ptr<Engine> engine (build (settings (), n_chn));

	samplecnt_t const latency = engine ? engine->stage.latency () : 0;
	{
		PBD::Mutex::Lock lm (_engine_lock);
		_engine.swap (engine);
	}

	if (_latency.exchange (latency) != latency) {
		Route* r = dynamic_cast<Route*> (owner ());
		if (r) {
			r->processor_latency_changed (); /* EMIT SIGNAL */
		}
	}
	/* previous engine, if any, is destroyed here, outside the lock */
}

SummingProcessor::Engine*
SummingProcessor::build (Settings const& s, uint32_t n_chn) const
{
	if (s.model.empty () || n_chn == 0) {
		return 0;
	}

	std::unique_ptr<Engine> e (new Engine);
	SummingModel*           model = 0;
	samplecnt_t             extra_latency = 0;
	uint32_t                oversampling  = s.oversampling;

	PluginType  type;
	std::string unique_id;

	if (parse_plugin_model_id (s.model, type, unique_id)) {
		PluginInfoPtr pip = LuaAPI::new_plugin_info (unique_id, type);
		if (!pip) {
			warning << string_compose (_("Summing plugin '%1' is not available, using linear summing."), unique_id) << endmsg;
			return 0;
		}

		e->plugin_in  = pip->n_inputs.n_audio ();
		e->plugin_out = pip->n_outputs.n_audio ();
		if (e->plugin_in == 0 || e->plugin_out == 0) {
			warning << string_compose (_("Summing plugin '%1' has no audio inputs or outputs, using linear summing."), pip->name) << endmsg;
			return 0;
		}
		e->replicated = e->plugin_in == 1 && e->plugin_out == 1 && n_chn > 1;

		uint32_t const n_instances = e->replicated ? n_chn : 1;
		for (uint32_t i = 0; i < n_instances; ++i) {
			PluginPtr p = pip->load (_session);
			if (!p) {
				warning << string_compose (_("Summing plugin '%1' could not be instantiated, using linear summing."), pip->name) << endmsg;
				return 0;
			}
			p->set_owner (owner ());
			if (!s.preset.empty ()) {
				Plugin::PresetRecord const* pr = p->preset_by_label (s.preset);
				if (pr) {
					p->load_preset (*pr);
				} else if (i == 0) {
					warning << string_compose (_("Summing plugin '%1' has no preset '%2'."), pip->name, s.preset) << endmsg;
				}
			}
			p->reconfigure_io (ChanCount (DataType::AUDIO, e->plugin_in), ChanCount (), ChanCount (DataType::AUDIO, e->plugin_out));
			p->set_block_size (_session.get_block_size ());
			p->activate ();
			e->plugins.push_back (p);
		}

		ChanCount const io (DataType::AUDIO, e->replicated ? 2 : e->plugin_in + e->plugin_out);
		e->plugin_bufs.ensure_buffers (io, _session.get_block_size ());
		e->plugin_bufs.set_count (io);

		extra_latency = e->plugins.front ()->signal_latency ();
		/* plugins run at the session rate, most saturation plugins
		 * offer oversampling themselves */
		oversampling = 1;

	} else {
		model = SummingModel::create (s.model);
		if (!model) {
			warning << string_compose (_("Summing model '%1' is not available, using linear summing."), s.model) << endmsg;
			return 0;
		}
	}

	e->ptrs.resize (n_chn);
	e->stage.set_levels (s.drive_db, s.auto_trim, s.trim_target_db);
	e->stage.configure (model, n_chn, _session.nominal_sample_rate (), _session.get_block_size (), oversampling, extra_latency);

	return e.release ();
}

void
SummingProcessor::run (BufferSet& bufs, samplepos_t start_sample, samplepos_t /*end_sample*/, double speed, pframes_t nframes, bool)
{
	if (!check_active ()) {
		return;
	}

	PBD::Mutex::Lock lm (_engine_lock, PBD::Mutex::TryLock);
	if (!lm.locked () || !_engine) {
		return;
	}

	Engine&        e (*_engine);
	uint32_t const n_chn = std::min<uint32_t> (bufs.count ().n_audio (), e.stage.n_channels ());
	pframes_t const max_block = e.stage.max_block ();

	if (n_chn == 0 || max_block == 0) {
		return;
	}

	for (pframes_t done = 0; done < nframes;) {
		pframes_t const ns = std::min<pframes_t> (nframes - done, max_block);

		for (uint32_t c = 0; c < n_chn; ++c) {
			e.ptrs[c] = bufs.get_audio (c).data (done);
		}

		e.stage.pre (&e.ptrs[0], n_chn, ns);
		if (e.plugins.empty ()) {
			e.stage.process (&e.ptrs[0], n_chn, ns);
		} else {
			run_plugins (e, bufs, start_sample + done, speed, ns, done);
		}
		e.stage.post (&e.ptrs[0], n_chn, ns);

		done += ns;
	}
}

void
SummingProcessor::run_plugins (Engine& e, BufferSet& bufs, samplepos_t start, double speed, pframes_t ns, pframes_t offset)
{
	/* plugins use the session's scratch buffers internally, so
	 * the plugin's I/O is staged in buffers owned by the engine.
	 */
	BufferSet&     sb (e.plugin_bufs);
	uint32_t const n_chn = std::min<uint32_t> (bufs.count ().n_audio (), e.stage.n_channels ());

	if (e.replicated) {
		ChanMapping in_map;
		ChanMapping out_map;
		in_map.set (DataType::AUDIO, 0, 0);
		out_map.set (DataType::AUDIO, 0, 1);
		for (uint32_t c = 0; c < n_chn; ++c) {
			sb.get_audio (0).read_from (bufs.get_audio (c), ns, 0, offset);
			e.plugins[c]->connect_and_run (sb, start, start + ns, speed, in_map, out_map, ns, 0);
			bufs.get_audio (c).read_from (sb.get_audio (1), ns, offset, 0);
		}
		return;
	}

	uint32_t const pin  = e.plugin_in;
	uint32_t const pout = e.plugin_out;
	ChanMapping    in_map;
	ChanMapping    out_map;

	for (uint32_t p = 0; p < pin; ++p) {
		/* e.g. a mono track feeding a stereo plugin: feed both pins */
		sb.get_audio (p).read_from (bufs.get_audio (p % n_chn), ns, 0, offset);
		in_map.set (DataType::AUDIO, p, p);
	}
	for (uint32_t p = 0; p < pout; ++p) {
		out_map.set (DataType::AUDIO, p, pin + p);
	}

	e.plugins.front ()->connect_and_run (sb, start, start + ns, speed, in_map, out_map, ns, 0);

	/* channels beyond the plugin's outputs pass unprocessed */
	for (uint32_t c = 0; c < std::min (n_chn, pout); ++c) {
		bufs.get_audio (c).read_from (sb.get_audio (pin + c), ns, offset, 0);
	}
}

XMLNode&
SummingProcessor::state () const
{
	XMLNode& node (Processor::state ());
	node.set_property ("type", "summing");
	return node;
}
