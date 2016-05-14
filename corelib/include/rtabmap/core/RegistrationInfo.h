/*
 * RegistrationInfo.h
 *
 *  Created on: Jan 5, 2016
 *      Author: mathieu
 */

#ifndef REGISTRATIONINFO_H_
#define REGISTRATIONINFO_H_


namespace rtabmap {

class RegistrationInfo
{
public:
	RegistrationInfo() :
		variance(0),
		inliers(0),
		matches(0),
		icpInliersRatio(0)
	{
	}

	float variance;
	std::string rejectedMsg;

	// RegistrationVis
	int inliers;
	std::vector<int> inliersIDs;
	int matches;
	std::vector<int> matchesIDs;

	// RegistrationIcp
	float icpInliersRatio;
};

}

#endif /* REGISTRATIONINFO_H_ */
