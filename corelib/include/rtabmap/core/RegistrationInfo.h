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
		inliersRatio(0)
	{
	}

	float variance;
	int inliers;
	float inliersRatio;
	std::vector<int> inliersIndexes_;
	std::string rejectedMsg_;
};

}

#endif /* REGISTRATIONINFO_H_ */
