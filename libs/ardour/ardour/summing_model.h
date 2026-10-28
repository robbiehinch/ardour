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

/** Where in the signal flow a summing model is used */
enum SummingRole {
	/** channel stage: end of a track's (or sub-bus') processing, before the
	 * fader, i.e. a console channel's line amplifier driving the mix bus.
	 */
	SummingChannel = 0x1,
	/** bus stage: the summed input of a bus or the master bus,
	 * i.e. the console's summing amplifier.
	 */
	SummingBus     = 0x2,
	SummingAnyRole = 0x3
};

/** Interface for analog summing emulation.
 *
 * A SummingModel processes audio at one of two places (see SummingRole):
 * the channel stage of each track and sub-bus, and the bus stage, right
 * after all inputs of a bus (port connections and aux-sends) have been
 * summed. Using both mirrors an analog console, where each channel's line
 * amp colours its own signal, and the summing amp colours the sum.
 *
 * Every stage owns its own model instance, so models may keep per-stage
 * state (filter memory, envelope followers etc).
 *
 * Level handling (drive, auto-trim) and oversampling are done around the
 * model by ARDOUR::SummingStage, so models only implement the transfer
 * function at the sample rate they are configured with.
 *
 * To add a custom model, implement this interface and register it once at
 * startup, e.g. using a static object in the same source file:
 *
 * @code
 * class MySumming : public ARDOUR::SummingModel {
 *   void configure (uint32_t n_chan, samplecnt_t rate) { ... }
 *   void run (float* const* bufs, uint32_t n_chan, pframes_t n_samples) { ... }
 * };
 *
 * static struct MyRegistrar {
 *   MyRegistrar () {
 *     ARDOUR::SummingModel::register_model ("My Console", [] () -> ARDOUR::SummingModel* { return new MySumming; });
 *   }
 * } my_registrar;
 * @endcode
 *
 * See airwindows_console.cc for complete examples. The model can then be
 * selected per session in Session > Properties > Mixer.
 */
class LIBARDOUR_API SummingModel
{
public:
	typedef std::function<SummingModel* ()> Factory;

	virtual ~SummingModel () {}

	/** Prepare for processing.
	 *
	 * Called from a non-realtime thread before the first call to run(), and
	 * whenever the stage's channel count or oversampling factor changes.
	 * Allocate per-channel state here.
	 *
	 * @param n_channels number of audio channels
	 * @param sample_rate rate at which run() will be called, i.e. the
	 *        session sample rate multiplied by the oversampling factor
	 */
	virtual void configure (uint32_t n_channels, samplecnt_t sample_rate) = 0;

	/** Process audio in-place.
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
	 * @param roles stages this model is meant for (a bitwise or of SummingRole)
	 * @return false if a model of the same name is already registered
	 */
	static bool register_model (std::string const& name, Factory factory, int roles = SummingAnyRole);

	/** @return a new instance of the named model, or 0 if no such model exists */
	static SummingModel* create (std::string const& name);

	/** @return names of all models registered for the given role, in registration order */
	static std::vector<std::string> available_models (int role = SummingAnyRole);
};

} // namespace ARDOUR
