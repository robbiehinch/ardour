#include <algorithm>
#include <cmath>
#include <vector>

#include "ardour/dB.h"
#include "ardour/summing_model.h"
#include "ardour/summing_stage.h"

#include "summing_stage_test.h"

CPPUNIT_TEST_SUITE_REGISTRATION (SummingStageTest);

using namespace ARDOUR;

namespace {

static const samplecnt_t rate  = 48000;
static const pframes_t   block = 256;

/** hard clip at +/-1, linear below; records the peak it saw */
class ClipModel : public SummingModel
{
public:
	ClipModel (float* peak) : _peak (peak) {}
	void configure (uint32_t, samplecnt_t) {}
	void
	run (float* const* bufs, uint32_t n_channels, pframes_t n_samples)
	{
		for (uint32_t c = 0; c < n_channels; ++c) {
			for (pframes_t s = 0; s < n_samples; ++s) {
				*_peak     = std::max (*_peak, fabsf (bufs[c][s]));
				bufs[c][s] = std::max (-1.f, std::min (1.f, bufs[c][s]));
			}
		}
	}

private:
	float* _peak;
};

static std::vector<float>
sine (float amplitude, float freq, pframes_t n)
{
	std::vector<float> v (n);
	for (pframes_t i = 0; i < n; ++i) {
		v[i] = amplitude * sinf (2.f * M_PI * freq * i / rate);
	}
	return v;
}

class IdentityModel : public SummingModel
{
public:
	void configure (uint32_t, samplecnt_t) {}
	void run (float* const*, uint32_t, pframes_t) {}
};

/* run a mono signal through up+downsampling */
static std::vector<float>
round_trip (Oversampler& os, std::vector<float> const& in)
{
	std::vector<float> out (in.size ()), hi (block * os.factor ());
	for (size_t s = 0; s < in.size (); s += block) {
		os.upsample (0, &in[s], &hi[0], block);
		os.downsample (0, &hi[0], &out[s], block);
	}
	return out;
}

/* run a stage over a mono signal, in blocks */
static std::vector<float>
run_stage (SummingStage& st, std::vector<float> in)
{
	for (size_t s = 0; s < in.size (); s += block) {
		float* b[1] = { &in[s] };
		st.pre (b, 1, block);
		st.process (b, 1, block);
		st.post (b, 1, block);
	}
	return in;
}

}

void
SummingStageTest::oversamplerLatencyTest ()
{
	for (uint32_t L : { 2u, 4u, 8u, 16u }) {
		Oversampler os;
		os.configure (L, 1);
		CPPUNIT_ASSERT_EQUAL (L, os.factor ());
		CPPUNIT_ASSERT_EQUAL ((samplecnt_t) Oversampler::taps_per_phase, os.latency ());

		std::vector<float> imp (4 * block, 0.f);
		imp[10] = 1.f;
		std::vector<float> out = round_trip (os, imp);
		size_t peak = std::max_element (out.begin (), out.end (), [] (float a, float b) { return fabsf (a) < fabsf (b); }) - out.begin ();
		CPPUNIT_ASSERT_EQUAL ((size_t) (10 + os.latency ()), peak);
	}

	Oversampler os;
	os.configure (3, 1); // unsupported: bypass
	CPPUNIT_ASSERT_EQUAL (1u, os.factor ());
	CPPUNIT_ASSERT_EQUAL ((samplecnt_t) 0, os.latency ());
}

void
SummingStageTest::oversamplerPassbandTest ()
{
	/* 1kHz and 15kHz pass a round trip unchanged, apart from the delay */
	for (float freq : { 1000.f, 15000.f }) {
		Oversampler os;
		os.configure (4, 1);
		std::vector<float> in  = sine (.5f, freq, 32 * block);
		std::vector<float> out = round_trip (os, in);
		float err = 0;
		for (size_t i = 4 * block; i < in.size (); ++i) {
			err = std::max (err, fabsf (out[i] - in[i - os.latency ()]));
		}
		CPPUNIT_ASSERT (err < .5f * 1e-3f); // < -60dB
	}
}

void
SummingStageTest::oversamplerImageTest ()
{
	/* the upsampled 18kHz tone has no image at 48 - 18 = 30kHz */
	Oversampler os;
	os.configure (2, 1);
	const pframes_t n = 64 * block;
	std::vector<float> in = sine (1.f, 18000.f, n);
	std::vector<float> hi (2 * n);
	for (pframes_t s = 0; s < n; s += block) {
		os.upsample (0, &in[s], &hi[2 * s], block);
	}
	double re18 = 0, im18 = 0, re30 = 0, im30 = 0;
	for (pframes_t i = n; i < 2 * n; ++i) { // skip filter warm-up
		double const t = (double) i / (2.0 * rate);
		re18 += hi[i] * cos (2 * M_PI * 18000 * t);
		im18 += hi[i] * sin (2 * M_PI * 18000 * t);
		re30 += hi[i] * cos (2 * M_PI * 30000 * t);
		im30 += hi[i] * sin (2 * M_PI * 30000 * t);
	}
	double const ratio = sqrt ((re30 * re30 + im30 * im30) / (re18 * re18 + im18 * im18));
	CPPUNIT_ASSERT (20 * log10 (ratio) < -80);
}

void
SummingStageTest::driveTest ()
{
	/* +12dB drive pushes a -6dBFS sine into the clipper, the output level
	 * is restored afterwards: peak stays at the input's level, but clipped */
	float        seen = 0;
	SummingStage st;
	st.set_levels (12.f, false, -6.f);
	st.configure (new ClipModel (&seen), 1, rate, block, 1, 0);
	std::vector<float> out = run_stage (st, sine (.5f, 100.f, 64 * block));

	CPPUNIT_ASSERT (seen > 1.9f); // .5 * 4
	float peak = 0;
	for (size_t i = 32 * block; i < out.size (); ++i) {
		peak = std::max (peak, fabsf (out[i]));
	}
	CPPUNIT_ASSERT (fabsf (peak - .25f) < .01f); // clipped at 1, divided by 4
}

void
SummingStageTest::autoTrimTest ()
{
	/* auto-trim normalizes a quiet and a hot sine to the target level before
	 * the model, and restores the level after it */
	for (float amplitude : { .05f, 2.f }) {
		float        seen = 0;
		SummingStage st;
		st.set_levels (0.f, true, -6.f);
		st.configure (new ClipModel (&seen), 1, rate, block, 1, 0);
		std::vector<float> in  = sine (amplitude, 100.f, 400 * block); // ~2 sec
		std::vector<float> out = run_stage (st, in);

		/* after settling, the model sees ~ -6dBFS peaks */
		float const g = st.current_gain ();
		CPPUNIT_ASSERT (fabsf (g * amplitude - .5f) < .05f);

		/* and since that is below the clip level, the output equals the input */
		float err = 0;
		for (size_t i = 300 * block; i < in.size (); ++i) {
			err = std::max (err, fabsf (out[i] - in[i]));
		}
		CPPUNIT_ASSERT (err < amplitude * 1e-3f);
	}

	/* the boost is limited */
	float        seen = 0;
	SummingStage st;
	st.set_levels (0.f, true, -6.f);
	st.configure (new ClipModel (&seen), 1, rate, block, 1, 0);
	run_stage (st, sine (.001f, 100.f, 100 * block));
	CPPUNIT_ASSERT (fabsf (st.current_gain () - dB_to_coefficient (SummingStage::max_boost_db)) < .01f);
}

void
SummingStageTest::latencyCompensatedGainTest ()
{
	/* with oversampling latency, the inverse gain is applied delayed, so the
	 * output is the delayed input, even after auto-trim changed the gain.
	 * (right at the level jump, the gain moves fast and the filters smear
	 * samples with different gains: allow a small error there) */
	SummingStage st;
	st.set_levels (0.f, true, -6.f);
	st.configure (new IdentityModel, 1, rate, block, 4, 0);
	CPPUNIT_ASSERT_EQUAL ((samplecnt_t) Oversampler::taps_per_phase, st.latency ());

	std::vector<float> in = sine (.05f, 200.f, 200 * block);
	for (size_t i = 100 * block; i < in.size (); ++i) {
		in[i] *= 4.f; // level jump, auto-trim adapts
	}
	std::vector<float> out = run_stage (st, in);

	samplecnt_t const lat = st.latency ();
	float err = 0, err_jump = 0;
	for (size_t i = 10 * block; i < in.size (); ++i) {
		float const e = fabsf (out[i] - in[i - lat]);
		if (i >= 100 * block && i < (size_t) (100 * block + 4 * lat)) {
			err_jump = std::max (err_jump, e);
		} else {
			err = std::max (err, e);
		}
	}
	CPPUNIT_ASSERT (err < .2f * 1e-4f);      // < -80dB
	CPPUNIT_ASSERT (err_jump < .2f * 1e-2f); // < -40dB
}
