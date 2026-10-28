#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

class SummingStageTest : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE (SummingStageTest);
	CPPUNIT_TEST (oversamplerLatencyTest);
	CPPUNIT_TEST (oversamplerPassbandTest);
	CPPUNIT_TEST (oversamplerImageTest);
	CPPUNIT_TEST (driveTest);
	CPPUNIT_TEST (autoTrimTest);
	CPPUNIT_TEST (latencyCompensatedGainTest);
	CPPUNIT_TEST_SUITE_END ();

public:
	void setUp () {}
	void tearDown () {}

	void oversamplerLatencyTest ();
	void oversamplerPassbandTest ();
	void oversamplerImageTest ();
	void driveTest ();
	void autoTrimTest ();
	void latencyCompensatedGainTest ();
};
