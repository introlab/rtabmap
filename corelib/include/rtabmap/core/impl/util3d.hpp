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

#ifndef UTIL3D_HPP_
#define UTIL3D_HPP_

#include <rtabmap/core/util3d_transforms.h>

namespace rtabmap{
namespace util3d{

template<typename PointCloud2T>
LaserScan laserScanFromPointCloud(const PointCloud2T & cloud, bool filterNaNs, bool is2D, const Transform & transform)
{
	if(cloud.data.empty())
	{
		return LaserScan();
	}
	//determine the output type
	int fieldStates[8] = {0}; // x,y,z,normal_x,normal_y,normal_z,rgb,intensity
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
	std::uint32_t fieldOffsets[8] = {0};
#else
	pcl::uint32_t fieldOffsets[8] = {0};
#endif
	for(unsigned int i=0; i<cloud.fields.size(); ++i)
	{
		if(cloud.fields[i].name.compare("x") == 0)
		{
			fieldStates[0] = 1;
			fieldOffsets[0] = cloud.fields[i].offset;
		}
		else if(cloud.fields[i].name.compare("y") == 0)
		{
			fieldStates[1] = 1;
			fieldOffsets[1] = cloud.fields[i].offset;
		}
		else if(cloud.fields[i].name.compare("z") == 0 && !is2D)
		{
			fieldStates[2] = 1;
			fieldOffsets[2] = cloud.fields[i].offset;
		}
		else if(cloud.fields[i].name.compare("normal_x") == 0)
		{
			fieldStates[3] = 1;
			fieldOffsets[3] = cloud.fields[i].offset;
		}
		else if(cloud.fields[i].name.compare("normal_y") == 0)
		{
			fieldStates[4] = 1;
			fieldOffsets[4] = cloud.fields[i].offset;
		}
		else if(cloud.fields[i].name.compare("normal_z") == 0)
		{
			fieldStates[5] = 1;
			fieldOffsets[5] = cloud.fields[i].offset;
		}
		else if(cloud.fields[i].name.compare("rgb") == 0 || cloud.fields[i].name.compare("rgba") == 0)
		{
			fieldStates[6] = 1;
			fieldOffsets[6] = cloud.fields[i].offset;
		}
		else if(cloud.fields[i].name.compare("intensity") == 0)
		{
			if(cloud.fields[i].datatype != pcl::PCLPointField::FLOAT32)
			{
				static bool warningShown = false;
				if(!warningShown)
				{
					UWARN("The input scan cloud has an \"intensity\" field "
							"but the datatype (%d) is not supported. Intensity will be ignored. "
							"This message is only shown once.", cloud.fields[i].datatype);
					warningShown = true;
				}
				continue;
			}

			fieldStates[7] = 1;
			fieldOffsets[7] = cloud.fields[i].offset;
		}
		else
		{
			UDEBUG("Ignoring \"%s\" field", cloud.fields[i].name.c_str());
		}
	}
	if(fieldStates[0]==0 || fieldStates[1]==0)
	{
		//should have at least x and y set
		UERROR("Cloud has not corresponding fields to laser scan!");
		return LaserScan();
	}

	bool hasNormals = fieldStates[3] || fieldStates[4] || fieldStates[5];
	bool hasIntensity = fieldStates[7];
	bool hasRGB = !hasIntensity&&fieldStates[6];
	bool is3D = fieldStates[0] && fieldStates[1] && fieldStates[2];

	LaserScan::Format format;
	int outputNormalOffset = 0;
	if(is3D)
	{
		if(hasNormals && hasIntensity)
		{
			format = LaserScan::kXYZINormal;
			outputNormalOffset = 4;
		}
		else if(hasNormals && !hasIntensity && !hasRGB)
		{
			format = LaserScan::kXYZNormal;
			outputNormalOffset = 3;
		}
		else if(hasNormals && hasRGB)
		{
			format = LaserScan::kXYZRGBNormal;
			outputNormalOffset = 4;
		}
		else if(!hasNormals && hasIntensity)
		{
			format = LaserScan::kXYZI;
		}
		else if(!hasNormals && hasRGB)
		{
			format = LaserScan::kXYZRGB;
		}
		else
		{
			format = LaserScan::kXYZ;
		}
	}
	else
	{
		if(hasNormals && hasIntensity)
		{
			format = LaserScan::kXYINormal;
			outputNormalOffset = 3;
		}
		else if(hasNormals && !hasIntensity)
		{
			format = LaserScan::kXYNormal;
			outputNormalOffset = 2;
		}
		else if(!hasNormals && hasIntensity)
		{
			format = LaserScan::kXYI;
		}
		else
		{
			format = LaserScan::kXY;
		}
	}

	UASSERT(cloud.data.size()/cloud.point_step == (uint32_t)cloud.height*cloud.width);
	cv::Mat laserScan = cv::Mat(1, (int)cloud.data.size()/cloud.point_step, CV_32FC(LaserScan::channels(format)));

	bool transformValid = !transform.isNull() && !transform.isIdentity();
	Transform transformRot;
	if(transformValid)
	{
		transformRot = transform.rotation();
	}
	int oi=0;
	for (uint32_t row = 0; row < (uint32_t)cloud.height; ++row)
	{
		const uint8_t* row_data = &cloud.data[row * cloud.row_step];
		for (uint32_t col = 0; col < (uint32_t)cloud.width; ++col)
		{
			const uint8_t* msg_data = row_data + col * cloud.point_step;

			float * ptr = laserScan.ptr<float>(0, oi);

			bool valid = true;
			if(laserScan.channels() == 2)
			{
				ptr[0] = *(float*)(msg_data + fieldOffsets[0]);
				ptr[1] = *(float*)(msg_data + fieldOffsets[1]);
				valid = uIsFinite(ptr[0]) && uIsFinite(ptr[1]);
			}
			else if(laserScan.channels() == 3)
			{
				ptr[0] = *(float*)(msg_data + fieldOffsets[0]);
				ptr[1] = *(float*)(msg_data + fieldOffsets[1]);
				if(format == LaserScan::kXYI)
				{
					ptr[2] = *(float*)(msg_data + fieldOffsets[7]);
				}
				else // XYZ
				{
					ptr[2] = *(float*)(msg_data + fieldOffsets[2]);
					valid = uIsFinite(ptr[0]) && uIsFinite(ptr[1]) && uIsFinite(ptr[2]);
				}
			}
			else if(laserScan.channels() == 4)
			{
				ptr[0] = *(float*)(msg_data + fieldOffsets[0]);
				ptr[1] = *(float*)(msg_data + fieldOffsets[1]);
				ptr[2] = *(float*)(msg_data + fieldOffsets[2]);
				if(format == LaserScan::kXYZI)
				{
					ptr[3] = *(float*)(msg_data + fieldOffsets[7]);
				}
				else // XYZRGB
				{
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
					std::uint8_t b=*(msg_data + fieldOffsets[6]);
					std::uint8_t g=*(msg_data + fieldOffsets[6]+1);
					std::uint8_t r=*(msg_data + fieldOffsets[6]+2);
#else
					pcl::uint8_t b=*(msg_data + fieldOffsets[6]);
					pcl::uint8_t g=*(msg_data + fieldOffsets[6]+1);
					pcl::uint8_t r=*(msg_data + fieldOffsets[6]+2);
#endif
					int * ptrInt = (int*)ptr;
					ptrInt[3] = int(b) | (int(g) << 8) | (int(r) << 16);
				}
				valid = uIsFinite(ptr[0]) && uIsFinite(ptr[1]) && uIsFinite(ptr[2]);
			}
			else if(laserScan.channels() == 5)
			{
				ptr[0] = *(float*)(msg_data + fieldOffsets[0]);
				ptr[1] = *(float*)(msg_data + fieldOffsets[1]);
				ptr[2] = *(float*)(msg_data + fieldOffsets[3]);
				ptr[3] = *(float*)(msg_data + fieldOffsets[4]);
				ptr[4] = *(float*)(msg_data + fieldOffsets[5]);
				valid = uIsFinite(ptr[0]) && uIsFinite(ptr[1]) && uIsFinite(ptr[2]) && uIsFinite(ptr[3]) && uIsFinite(ptr[4]);
			}
			else if(laserScan.channels() == 6)
			{
				ptr[0] = *(float*)(msg_data + fieldOffsets[0]);
				ptr[1] = *(float*)(msg_data + fieldOffsets[1]);
				if(format == LaserScan::kXYINormal)
				{
					ptr[2] = *(float*)(msg_data + fieldOffsets[7]);
				}
				else // XYZNormal
				{
					ptr[2] = *(float*)(msg_data + fieldOffsets[2]);
				}
				ptr[3] = *(float*)(msg_data + fieldOffsets[3]);
				ptr[4] = *(float*)(msg_data + fieldOffsets[4]);
				ptr[5] = *(float*)(msg_data + fieldOffsets[5]);
				valid = uIsFinite(ptr[0]) && uIsFinite(ptr[1]) && uIsFinite(ptr[2]) && uIsFinite(ptr[3]) && uIsFinite(ptr[4]) &&  uIsFinite(ptr[5]);
			}
			else if(laserScan.channels() == 7)
			{
				ptr[0] = *(float*)(msg_data + fieldOffsets[0]);
				ptr[1] = *(float*)(msg_data + fieldOffsets[1]);
				ptr[2] = *(float*)(msg_data + fieldOffsets[2]);
				if(format == LaserScan::kXYZINormal)
				{
					ptr[3] = *(float*)(msg_data + fieldOffsets[7]);
				}
				else // XYZRGBNormal
				{
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
					std::uint8_t b=*(msg_data + fieldOffsets[6]);
					std::uint8_t g=*(msg_data + fieldOffsets[6]+1);
					std::uint8_t r=*(msg_data + fieldOffsets[6]+2);
#else
					pcl::uint8_t b=*(msg_data + fieldOffsets[6]);
					pcl::uint8_t g=*(msg_data + fieldOffsets[6]+1);
					pcl::uint8_t r=*(msg_data + fieldOffsets[6]+2);
#endif
					int * ptrInt = (int*)ptr;
					ptrInt[3] = int(b) | (int(g) << 8) | (int(r) << 16);
				}
				ptr[4] = *(float*)(msg_data + fieldOffsets[3]);
				ptr[5] = *(float*)(msg_data + fieldOffsets[4]);
				ptr[6] = *(float*)(msg_data + fieldOffsets[5]);
				valid = uIsFinite(ptr[0]) && uIsFinite(ptr[1]) && uIsFinite(ptr[2]) && uIsFinite(ptr[4]) && uIsFinite(ptr[5]) &&  uIsFinite(ptr[6]);
			}
			else
			{
				UFATAL("Cannot handle as many channels (%d)!", laserScan.channels());
			}

			if(!filterNaNs || valid)
			{
				if(valid && transformValid)
				{
					cv::Point3f pt = util3d::transformPoint(cv::Point3f(ptr[0], ptr[1], is3D?ptr[3]:0), transform);
					ptr[0] = pt.x;
					ptr[1] = pt.y;
					if(is3D)
					{
						ptr[2] = pt.z;
					}
					if(hasNormals)
					{
						pt = util3d::transformPoint(cv::Point3f(ptr[outputNormalOffset], ptr[outputNormalOffset+1], ptr[outputNormalOffset+2]), transformRot);
						ptr[outputNormalOffset] = pt.x;
						ptr[outputNormalOffset+1] = pt.y;
						ptr[outputNormalOffset+2] = pt.z;
					}
				}

				++oi;
			}
		}
	}
	if(oi == 0)
	{
		return LaserScan();
	}
	return  LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0, format);
}

}
}

#endif /* UTIL3D_HPP_ */
