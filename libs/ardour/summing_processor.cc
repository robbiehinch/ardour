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

#include "pbd/error.h"

#include "ardour/audio_buffer.h"
#include "ardour/buffer_set.h"
#include "ardour/session.h"
#include "ardour/summing_model.h"
#include "ardour/summing_processor.h"

#include "pbd/i18n.h"

using namespace ARDOUR;
using namespace PBD;

SummingProcessor::SummingProcessor (Session& s)
	: Processor (s, "Summing", Temporal::TimeDomainProvider (Temporal::AudioTime))
{
}

SummingProcessor::~SummingProcessor ()
{
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

	PBD::Mutex::Lock lm (_model_lock);
	_bufs.resize (in.n_audio ());
	if (_model) {
		_model->configure (in.n_audio (), _session.nominal_sample_rate ());
	}

	return Processor::configure_io (in, out);
}

void
SummingProcessor::set_model (std::string const& name)
{
	std::unique_ptr<SummingModel> model;

	if (!name.empty ()) {
		model.reset (SummingModel::create (name));
		if (!model) {
			warning << string_compose (_("Summing model '%1' is not available, using linear summing."), name) << endmsg;
		}
	}

	{
		/* configure under the lock, so that the channel count cannot
		 * change concurrently. The process thread only try-locks.
		 */
		PBD::Mutex::Lock lm (_model_lock);
		if (model) {
			model->configure (_bufs.size (), _session.nominal_sample_rate ());
		}
		_model.swap (model);
		_model_name = name;
	}
	/* previous model, if any, is destroyed here, outside the lock */
}

std::string
SummingProcessor::model_name () const
{
	PBD::Mutex::Lock lm (_model_lock);
	return _model_name;
}

void
SummingProcessor::run (BufferSet& bufs, samplepos_t /*start_sample*/, samplepos_t /*end_sample*/, double /*speed*/, pframes_t nframes, bool)
{
	if (!check_active ()) {
		return;
	}

	PBD::Mutex::Lock lm (_model_lock, PBD::Mutex::TryLock);
	if (!lm.locked () || !_model) {
		return;
	}

	uint32_t n_chn = 0;
	for (BufferSet::audio_iterator i = bufs.audio_begin (); i != bufs.audio_end () && n_chn < _bufs.size (); ++i, ++n_chn) {
		_bufs[n_chn] = i->data ();
	}

	if (n_chn > 0) {
		_model->run (&_bufs[0], n_chn, nframes);
	}
}

XMLNode&
SummingProcessor::state () const
{
	XMLNode& node (Processor::state ());
	node.set_property ("type", "summing");
	return node;
}
