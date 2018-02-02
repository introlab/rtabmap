/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef REGISTRATIONINFO_H_
#define REGISTRATIONINFO_H_


namespace rtabmap {

class RegistrationInfo
{
public:
	RegistrationInfo() :
		totalTime(0.0),
		inliers(0),
		matches(0),
		icpInliersRatio(0),
		icpTranslation(0.0f),
		icpRotation(0.0f),
		icpStructuralComplexity(0.0f)

	{
	}

	RegistrationInfo copyWithoutData() const
	{
		RegistrationInfo output;
		output.totalTime = totalTime;
		output.covariance = covariance.clone();
		output.rejectedMsg = rejectedMsg;
		output.inliers = inliers;
		output.matches = matches;
		output.icpInliersRatio = icpInliersRatio;
		output.icpTranslation = icpTranslation;
		output.icpRotation = icpRotation;
		output.icpStructuralComplexity = icpStructuralComplexity;
		return output;
	}

	cv::Mat covariance;
	std::string rejectedMsg;
	double totalTime;

	// RegistrationVis
	int inliers;
	std::vector<int> inliersIDs;
	int matches;
	std::vector<int> matchesIDs;
	std::vector<int> projectedIDs; // "From" IDs

	// RegistrationIcp
	float icpInliersRatio;
	float icpTranslation;
	float icpRotation;
	float icpStructuralComplexity;
};

}

#endif /* REGISTRATIONINFO_H_ */
