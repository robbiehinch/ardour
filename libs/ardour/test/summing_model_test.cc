#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "ardour/summing_model.h"

#include "summing_model_test.h"

CPPUNIT_TEST_SUITE_REGISTRATION (SummingModelTest);

using namespace ARDOUR;

namespace {

class NullSumming : public SummingModel
{
public:
	void configure (uint32_t, samplecnt_t) {}
	void run (float* const*, uint32_t, pframes_t) {}
};

static const samplecnt_t rate = 48000;

/* run a stereo sine of the given amplitude through the model,
 * return the max abs deviation from the input and the output peak
 * over the second half (after filters settled).
 */
static void
run_sine (SummingModel& m, float amplitude, float& max_err, float& peak)
{
	const pframes_t n = rate;
	std::vector<float> l (n), r (n), in (n);
	for (pframes_t i = 0; i < n; ++i) {
		in[i] = l[i] = r[i] = amplitude * sinf (2.f * M_PI * 1000.f * i / rate);
	}
	m.configure (2, rate);
	float* bufs[2] = { &l[0], &r[0] };
	/* process in chunks like the engine would */
	for (pframes_t s = 0; s < n; s += 512) {
		float* chunk[2] = { bufs[0] + s, bufs[1] + s };
		m.run (chunk, 2, std::min<pframes_t> (512, n - s));
	}
	max_err = peak = 0;
	for (pframes_t i = n / 2; i < n; ++i) {
		CPPUNIT_ASSERT (std::isfinite (l[i]) && std::isfinite (r[i]));
		max_err = std::max (max_err, fabsf (l[i] - in[i]));
		peak    = std::max (peak, fabsf (l[i]));
	}
}

}

void
SummingModelTest::registryTest ()
{
	std::vector<std::string> models = SummingModel::available_models ();
	CPPUNIT_ASSERT (std::find (models.begin (), models.end (), "Analog Console") != models.end ());

	std::unique_ptr<SummingModel> m (SummingModel::create ("Analog Console"));
	CPPUNIT_ASSERT (m);
	CPPUNIT_ASSERT (!SummingModel::create ("No Such Model"));
	CPPUNIT_ASSERT (!SummingModel::create (""));

	CPPUNIT_ASSERT (SummingModel::register_model ("Test Null", [] () -> SummingModel* { return new NullSumming; }));
	CPPUNIT_ASSERT (!SummingModel::register_model ("Test Null", [] () -> SummingModel* { return new NullSumming; }));
	CPPUNIT_ASSERT (!SummingModel::register_model ("", [] () -> SummingModel* { return new NullSumming; }));
	std::unique_ptr<SummingModel> n (SummingModel::create ("Test Null"));
	CPPUNIT_ASSERT (n);
}

void
SummingModelTest::transparencyTest ()
{
	/* at -40dBFS the model is transparent within -60dB of the signal */
	std::unique_ptr<SummingModel> m (SummingModel::create ("Analog Console"));
	float err, peak;
	run_sine (*m, 0.01f, err, peak);
	CPPUNIT_ASSERT (err < 0.01f * 0.001f * 10.f); // < -40dB relative error
	CPPUNIT_ASSERT (fabsf (peak - 0.01f) < 0.0001f);
}

void
SummingModelTest::saturationTest ()
{
	/* +12dBFS input is compressed by the summing amplifier */
	std::unique_ptr<SummingModel> m (SummingModel::create ("Analog Console"));
	float err, peak;
	run_sine (*m, 4.f, err, peak);
	CPPUNIT_ASSERT (peak < 4.f * 0.97f);
	CPPUNIT_ASSERT (peak > 3.f);
}

void
SummingModelTest::dcBlockTest ()
{
	std::unique_ptr<SummingModel> m (SummingModel::create ("Analog Console"));
	const pframes_t n = rate;
	std::vector<float> l (n, 0.5f), r (n, -0.5f);
	float* bufs[2] = { &l[0], &r[0] };
	m->configure (2, rate);
	m->run (bufs, 2, n);
	CPPUNIT_ASSERT (fabsf (l[n - 1]) < 1e-3f);
	CPPUNIT_ASSERT (fabsf (r[n - 1]) < 1e-3f);
}

void
SummingModelTest::oddChannelTest ()
{
	std::unique_ptr<SummingModel> m (SummingModel::create ("Analog Console"));
	const pframes_t n = 1024;
	std::vector<float> a (n, 0.f), b (n, 0.f), c (n, 0.f);
	a[0] = b[0] = c[0] = 1.f;
	float* bufs[3] = { &a[0], &b[0], &c[0] };
	m->configure (3, rate);
	m->run (bufs, 3, n);
	for (pframes_t i = 0; i < n; ++i) {
		CPPUNIT_ASSERT (std::isfinite (a[i]) && std::isfinite (b[i]) && std::isfinite (c[i]));
	}
	/* fewer channels than configured is allowed */
	m->run (bufs, 1, n);
}

void
SummingModelTest::rolesTest ()
{
	std::vector<std::string> ch = SummingModel::available_models (SummingChannel);
	std::vector<std::string> bus = SummingModel::available_models (SummingBus);

	/* the analog model applies to both stages */
	CPPUNIT_ASSERT (std::find (ch.begin (), ch.end (), "Analog Console") != ch.end ());
	CPPUNIT_ASSERT (std::find (bus.begin (), bus.end (), "Analog Console") != bus.end ());

	/* Airwindows encode goes on channels, decode on busses */
	CPPUNIT_ASSERT (std::find (ch.begin (), ch.end (), "Airwindows Console6 Channel") != ch.end ());
	CPPUNIT_ASSERT (std::find (ch.begin (), ch.end (), "Airwindows Console6 Buss") == ch.end ());
	CPPUNIT_ASSERT (std::find (bus.begin (), bus.end (), "Airwindows Console6 Buss") != bus.end ());
	CPPUNIT_ASSERT (std::find (bus.begin (), bus.end (), "Airwindows Console6 Channel") == bus.end ());

	CPPUNIT_ASSERT (!SummingModel::register_model ("No Role", [] () -> SummingModel* { return new NullSumming; }, 0));
}

/** Channel encode followed by Buss decode, for a single source */
static float
airwindows_round_trip_error (std::string const& variant, float amplitude, pframes_t& n_out)
{
	std::unique_ptr<SummingModel> ch (SummingModel::create ("Airwindows " + variant + " Channel"));
	std::unique_ptr<SummingModel> bus (SummingModel::create ("Airwindows " + variant + " Buss"));
	CPPUNIT_ASSERT (ch && bus);

	const pframes_t n = rate / 10;
	std::vector<float> l (n), in (n);
	for (pframes_t i = 0; i < n; ++i) {
		in[i] = l[i] = amplitude * sinf (2.f * M_PI * 100.f * i / rate);
	}
	float* bufs[1] = { &l[0] };
	ch->configure (1, rate);
	bus->configure (1, rate);
	ch->run (bufs, 1, n);
	bus->run (bufs, 1, n);

	float err = 0;
	for (pframes_t i = n / 2; i < n; ++i) {
		CPPUNIT_ASSERT (std::isfinite (l[i]));
		err = std::max (err, fabsf (l[i] - in[i]));
	}
	n_out = n;
	return err / amplitude;
}

void
SummingModelTest::airwindowsRoundTripTest ()
{
	/* decode is the exact inverse of encode for the stateless variants */
	pframes_t n;
	CPPUNIT_ASSERT (airwindows_round_trip_error ("PurestConsole", .5f, n) < 1e-5f);
	CPPUNIT_ASSERT (airwindows_round_trip_error ("Console6", .5f, n) < 1e-5f);
	/* Console7 adds 20kHz filters, its curves are not exact inverses */
	CPPUNIT_ASSERT (airwindows_round_trip_error ("Console7", .1f, n) < .05f);
}

/** max deviation of Console6 encode -> sum -> decode from a plain sum of two sines */
static float
console6_sum_deviation (float amplitude)
{
	std::unique_ptr<SummingModel> ch (SummingModel::create ("Airwindows Console6 Channel"));
	std::unique_ptr<SummingModel> bus (SummingModel::create ("Airwindows Console6 Buss"));
	ch->configure (2, rate);
	bus->configure (1, rate);

	const pframes_t n = 1024;
	std::vector<float> a (n), b (n), sum (n), plain (n);
	for (pframes_t i = 0; i < n; ++i) {
		a[i] = amplitude * sinf (2.f * M_PI * 100.f * i / rate);
		b[i] = amplitude * sinf (2.f * M_PI * 150.f * i / rate);
		plain[i] = a[i] + b[i];
	}
	float* chn[2] = { &a[0], &b[0] };
	ch->run (chn, 2, n);
	for (pframes_t i = 0; i < n; ++i) {
		sum[i] = a[i] + b[i];
	}
	float* bus_buf[1] = { &sum[0] };
	bus->run (bus_buf, 1, n);

	float diff = 0;
	for (pframes_t i = 0; i < n; ++i) {
		CPPUNIT_ASSERT (std::isfinite (sum[i]));
		diff = std::max (diff, fabsf (sum[i] - plain[i]));
	}
	return diff / (2.f * amplitude);
}

void
SummingModelTest::airwindowsSummingTest ()
{
	/* summing via encode/decode is nearly linear at low levels, and adds
	 * density (the console character) at higher levels */
	CPPUNIT_ASSERT (console6_sum_deviation (.001f) < .002f);
	CPPUNIT_ASSERT (console6_sum_deviation (.25f) > .05f);
}
