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
#include <string>
#include <vector>

#include "pbd/mutex.h"

#include "ardour/processor.h"
#include "ardour/types.h"

namespace ARDOUR {

class SummingModel;

/** Internal bus processor that applies a SummingModel to the summed
 * input of a bus, before trim, plugins and fader.
 */
class LIBARDOUR_API SummingProcessor : public Processor
{
public:
	SummingProcessor (Session&);
	~SummingProcessor ();

	bool display_to_user () const { return false; }
	void run (BufferSet& bufs, samplepos_t start_sample, samplepos_t end_sample, double speed, pframes_t nframes, bool result_required);
	bool configure_io (ChanCount in, ChanCount out);
	bool can_support_io_configuration (const ChanCount& in, ChanCount& out);

	/** Select the model to use (not realtime safe).
	 * @param name a name from SummingModel::available_models(); empty for linear summing
	 */
	void set_model (std::string const& name);
	std::string model_name () const;

protected:
	XMLNode& state () const;

private:
	mutable PBD::Mutex            _model_lock;
	std::unique_ptr<SummingModel> _model;
	std::string                   _model_name;
	std::vector<float*>           _bufs;
};

} // namespace ARDOUR
