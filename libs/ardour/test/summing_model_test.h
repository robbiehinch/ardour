#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

class SummingModelTest : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE (SummingModelTest);
	CPPUNIT_TEST (registryTest);
	CPPUNIT_TEST (transparencyTest);
	CPPUNIT_TEST (saturationTest);
	CPPUNIT_TEST (dcBlockTest);
	CPPUNIT_TEST (oddChannelTest);
	CPPUNIT_TEST_SUITE_END ();

public:
	void setUp () {}
	void tearDown () {}

	void registryTest ();
	void transparencyTest ();
	void saturationTest ();
	void dcBlockTest ();
	void oddChannelTest ();
};
