/*
Copyright (c) 2010-2024, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap/core/LASWriter.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>

#include <fstream>
#include <liblas/liblas.hpp>

namespace rtabmap {

int saveLASFile(const std::string & filePath,
		const pcl::PointCloud<pcl::PointXYZ> & cloud,
		const std::vector<int> & cameraIds)
{
	UASSERT_MSG(cameraIds.empty() || cameraIds.size() == cloud.size(),
			uFormat("cameraIds=%d cloud=%d", (int)cameraIds.size(), (int)cloud.size()).c_str());

	std::ofstream ofs;
	ofs.open(filePath, std::ios::out | std::ios::binary);

	liblas::Header header;
	header.SetScale(0.001, 0.001, 0.001);

	header.SetCompressed(uToLowerCase(UFile::getExtension(filePath)) == "laz");

	try {
		liblas::Writer writer(ofs, header);

		for(size_t i=0; i<cloud.size(); ++i)
		{
			liblas::Point point(&header);
			point.SetCoordinates(cloud.at(i).x, cloud.at(i).y, cloud.at(i).z);
			if(!cameraIds.empty())
			{
				point.SetPointSourceID(cameraIds.at(i));
			}
			
			writer.WritePoint(point);
		}
	}
	catch(liblas::configuration_error & e)
	{
		UERROR("\"laz\" format not available, use \"las\" instead: %s", e.what());
		return 1;
	}

	return 0; //success
}

int saveLASFile(const std::string & filePath,
		const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
		const std::vector<int> & cameraIds,
		const std::vector<float> & intensities)
{
	UASSERT_MSG(cameraIds.empty() || cameraIds.size() == cloud.size(),
			uFormat("cameraIds=%d cloud=%d", (int)cameraIds.size(), (int)cloud.size()).c_str());
	UASSERT_MSG(intensities.empty() || intensities.size() == cloud.size(),
				uFormat("intensities=%d cloud=%d", (int)intensities.size(), (int)cloud.size()).c_str());

	UASSERT_MSG(cameraIds.empty() || cameraIds.size() == cloud.size(),
			uFormat("cameraIds=%d cloud=%d", (int)cameraIds.size(), (int)cloud.size()).c_str());

	std::ofstream ofs;
	ofs.open(filePath, std::ios::out | std::ios::binary);

	liblas::Header header;

	header.SetScale(0.001, 0.001, 0.001);
	header.SetCompressed(uToLowerCase(UFile::getExtension(filePath)) == "laz");

	try {
		liblas::Writer writer(ofs, header);

		for(size_t i=0; i<cloud.size(); ++i)
		{
			liblas::Point point(&header);
			point.SetCoordinates(cloud.at(i).x, cloud.at(i).y, cloud.at(i).z);
			liblas::Color color(cloud.at(i).r*65535/255, cloud.at(i).g*65535/255, cloud.at(i).b*65535/255);
			point.SetColor(color);
			if(!cameraIds.empty())
			{
				point.SetPointSourceID(cameraIds.at(i));
			}
			if(!intensities.empty())
			{
				point.SetIntensity(intensities.at(i));
			}
			
			writer.WritePoint(point);
		}
	}
	catch(liblas::configuration_error & e)
	{
		UERROR("\"laz\" format not available, use \"las\" instead: %s", e.what());
		return 1;
	}

	return 0; //success
}

int saveLASFile(const std::string & filePath,
		const pcl::PointCloud<pcl::PointXYZI> & cloud,
		const std::vector<int> & cameraIds)
{
	UASSERT_MSG(cameraIds.empty() || cameraIds.size() == cloud.size(),
			uFormat("cameraIds=%d cloud=%d", (int)cameraIds.size(), (int)cloud.size()).c_str());

	std::ofstream ofs;
	ofs.open(filePath, std::ios::out | std::ios::binary);

	liblas::Header header;
	header.SetScale(0.001, 0.001, 0.001);

	header.SetCompressed(uToLowerCase(UFile::getExtension(filePath)) == "laz");

	try {
		liblas::Writer writer(ofs, header);

		for(size_t i=0; i<cloud.size(); ++i)
		{
			liblas::Point point(&header);
			point.SetCoordinates(cloud.at(i).x, cloud.at(i).y, cloud.at(i).z);
			if(!cameraIds.empty())
			{
				point.SetPointSourceID(cameraIds.at(i));
			}
			
			writer.WritePoint(point);
		}
	}
	catch(liblas::configuration_error & e)
	{
		UERROR("\"laz\" format not available, use \"las\" instead: %s", e.what());
		return 1;
	}

	return 0; //success
}

}

