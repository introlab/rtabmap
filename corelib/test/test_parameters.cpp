#include <gtest/gtest.h>
#include <rtabmap/core/Parameters.h>

using rtabmap::Parameters;
using rtabmap::ParametersMap;

TEST(Parameters, descriptions_are_shorter_than_1024_characters)
{
	// Long descriptions don't fit in the GUI's tooltips nor in a ROS parameter listing:
	// keep them short, the details belong in the documentation. 1024 was also the size
	// beyond which uFormat(), which formats most of them, aborted on Windows.
	const ParametersMap & parameters = Parameters::getDefaultParameters();
	ASSERT_FALSE(parameters.empty());
	for(ParametersMap::const_iterator iter = parameters.begin(); iter != parameters.end(); ++iter)
	{
		const std::string description = Parameters::getDescription(iter->first);
		EXPECT_LT(description.size(), 1024u) << iter->first << " has a description of "
				<< description.size() << " characters: shorten it";
	}
}
