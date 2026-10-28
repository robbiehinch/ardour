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

#include <functional>
#include <string>
#include <vector>

#include "ardour/libardour_visibility.h"
#include "ardour/types.h"

namespace ARDOUR {

/** Interface for analog summing emulation.
 *
 * A SummingModel processes the signal of a bus (and the master bus) right
 * after all of its inputs (port connections and aux-sends) have been summed,
 * and before the bus' trim, plugins and fader. This is the point where a
 * console's summing amplifier sits, and where Mixbus applies its built-in
 * summing character.
 *
 * Every bus owns its own model instance, so models may keep per-bus state
 * (filter memory, envelope followers etc).
 *
 * To add a custom model, implement this interface and register it once at
 * startup (e.g. from a static initializer in the same source file):
 *
 * @code
 * class MySumming : public ARDOUR::SummingModel {
 *   void configure (uint32_t n_chan, samplecnt_t rate) { ... }
 *   void run (float* const* bufs, uint32_t n_chan, pframes_t n_samples) { ... }
 * };
 *
 * static bool registered = ARDOUR::SummingModel::register_model (
 *     "My Console", [] () { return new MySumming; });
 * @endcode
 *
 * The model can then be selected per session in
 * Session > Properties > Mixer ("summing-model" session configuration).
 */
class LIBARDOUR_API SummingModel
{
public:
	typedef std::function<SummingModel* ()> Factory;

	virtual ~SummingModel () {}

	/** Prepare for processing.
	 *
	 * Called from a non-realtime thread before the first call to run(), and
	 * whenever the bus' channel count changes. Allocate per-channel state here.
	 *
	 * @param n_channels number of audio channels of the bus
	 * @param sample_rate session sample rate
	 */
	virtual void configure (uint32_t n_channels, samplecnt_t sample_rate) = 0;

	/** Process the summed bus signal in-place.
	 *
	 * Called from the realtime process thread: must not allocate memory,
	 * take locks or otherwise block. Must not add latency.
	 *
	 * @param bufs one buffer per channel, \p n_channels entries
	 * @param n_channels number of channels, never more than given to configure()
	 * @param n_samples number of samples to process in each buffer
	 */
	virtual void run (float* const* bufs, uint32_t n_channels, pframes_t n_samples) = 0;

	/** Make a model available under the given (user-visible) name.
	 * @return false if a model of the same name is already registered
	 */
	static bool register_model (std::string const& name, Factory factory);

	/** @return a new instance of the named model, or 0 if no such model exists */
	static SummingModel* create (std::string const& name);

	/** @return names of all registered models, in registration order */
	static std::vector<std::string> available_models ();
};

} // namespace ARDOUR
