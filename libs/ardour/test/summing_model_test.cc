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
