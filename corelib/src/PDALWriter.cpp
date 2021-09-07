/*
Copyright (c) 2010-2021, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap/core/PDALWriter.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <pdal/io/BufferReader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PluginManager.hpp>

#ifdef RTABMAP_PDAL_16
#include <pdal/pdal_defines.h>
#else
#include <pdal/pdal_features.hpp>
#endif

namespace rtabmap {

std::string getPDALSupportedWriters()
{
	std::string output;
	// Force plugin loading.
#if PDAL_VERSION_MAJOR>1 || (PDAL_VERSION_MAJOR == 1 && PDAL_VERSION_MINOR > 7) || (PDAL_VERSION_MAJOR == 1 && PDAL_VERSION_MINOR == 7 && PDAL_VERSION_MINOR>=1)
	pdal::StageFactory f;
	pdal::PluginManager<pdal::Stage>::loadAll();
	pdal::StringList stages = pdal::PluginManager<pdal::Stage>::names();
#else
	pdal::StageFactory f(false);
	pdal::StringList stages = pdal::PluginManager::names(PF_PluginType_Writer);
#endif
	for(pdal::StringList::iterator iter=stages.begin(); iter!=stages.end(); ++iter)
	{
		if(!uStrContains(*iter, "writers"))
		{
			continue;
		}
		if(iter->compare("writers.null") == 0)
		{
			continue;
		}
		if(iter!=stages.begin())
		{
			output += " ";
		}
		output += UFile::getExtension(*iter);
	}
	return output;
}

int savePDALFile(const std::string & filePath,
		const pcl::PointCloud<pcl::PointXYZ> & cloud,
		const std::vector<int> & cameraIds,
		bool binary)
{
	UASSERT_MSG(cameraIds.empty() || cameraIds.size() == cloud.size(),
			uFormat("cameraIds=%d cloud=%d", (int)cameraIds.size(), (int)cloud.size()).c_str());

	pdal::PointTable table;

	if(!cameraIds.empty())
	{
		table.layout()->registerDims({
			pdal::Dimension::Id::X,
			pdal::Dimension::Id::Y,
			pdal::Dimension::Id::Z,
			pdal::Dimension::Id::PointSourceId});
	}
	else
	{
		table.layout()->registerDims({
			pdal::Dimension::Id::X,
			pdal::Dimension::Id::Y,
			pdal::Dimension::Id::Z});
	}
	pdal::BufferReader bufferReader;

	pdal::PointViewPtr view(new pdal::PointView(table));
	for(size_t i=0; i<cloud.size(); ++i)
	{
		view->setField(pdal::Dimension::Id::X, i, cloud.at(i).x);
		view->setField(pdal::Dimension::Id::Y, i, cloud.at(i).y);
		view->setField(pdal::Dimension::Id::Z, i, cloud.at(i).z);
		if(!cameraIds.empty())
		{
			view->setField(pdal::Dimension::Id::PointSourceId, i, cameraIds.at(i));
		}
	}
	bufferReader.addView(view);

	pdal::StageFactory factory;
	std::string ext = UFile::getExtension(filePath);
	pdal::Stage *writer = factory.createStage("writers." + ext);
	if(writer)
	{
		pdal::Options writerOps;
		writerOps.add("filename", filePath);
		if(ext.compare("ply")==0) writerOps.add("storage_mode", binary?"little endian":"ascii"); // PLY
		if(ext.compare("pcd")==0) writerOps.add("compression", binary?"binary":"ascii"); // PCD

		writer->setOptions(writerOps);
		writer->setInput(bufferReader);

		writer->prepare(table);
		writer->execute(table);
	}
	else
	{
		UERROR("PDAL: cannot find writer for extension \"%s\". Available extensions: \"%s\"",
				UFile::getExtension(filePath).c_str(),
				getPDALSupportedWriters().c_str());
		return 1;
	}

	return 0; //success
}

int savePDALFile(const std::string & filePath,
		const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
		const std::vector<int> & cameraIds,
		bool binary,
		const std::vector<float> & intensities)
{
	UASSERT_MSG(cameraIds.empty() || cameraIds.size() == cloud.size(),
			uFormat("cameraIds=%d cloud=%d", (int)cameraIds.size(), (int)cloud.size()).c_str());
	UASSERT_MSG(intensities.empty() || intensities.size() == cloud.size(),
				uFormat("intensities=%d cloud=%d", (int)intensities.size(), (int)cloud.size()).c_str());

	pdal::PointTable table;

	if(!intensities.empty() && !cameraIds.empty())
	{
		table.layout()->registerDims({
			pdal::Dimension::Id::X,
			pdal::Dimension::Id::Y,
			pdal::Dimension::Id::Z,
			pdal::Dimension::Id::Red,
			pdal::Dimension::Id::Green,
			pdal::Dimension::Id::Blue,
			pdal::Dimension::Id::PointSourceId,
			pdal::Dimension::Id::Intensity});
	}
	else if(!intensities.empty())
	{
		table.layout()->registerDims({
			pdal::Dimension::Id::X,
			pdal::Dimension::Id::Y,
			pdal::Dimension::Id::Z,
			pdal::Dimension::Id::Red,
			pdal::Dimension::Id::Green,
			pdal::Dimension::Id::Blue,
			pdal::Dimension::Id::Intensity});
	}
	else if(!cameraIds.empty())
	{
		table.layout()->registerDims({
			pdal::Dimension::Id::X,
			pdal::Dimension::Id::Y,
			pdal::Dimension::Id::Z,
			pdal::Dimension::Id::Red,
			pdal::Dimension::Id::Green,
			pdal::Dimension::Id::Blue,
			pdal::Dimension::Id::PointSourceId});
	}
	else
	{
		table.layout()->registerDims({
			pdal::Dimension::Id::X,
			pdal::Dimension::Id::Y,
			pdal::Dimension::Id::Z,
			pdal::Dimension::Id::Red,
			pdal::Dimension::Id::Green,
			pdal::Dimension::Id::Blue});
	}
	pdal::BufferReader bufferReader;

	pdal::PointViewPtr view(new pdal::PointView(table));
	for(size_t i=0; i<cloud.size(); ++i)
	{
		view->setField(pdal::Dimension::Id::X, i, cloud.at(i).x);
		view->setField(pdal::Dimension::Id::Y, i, cloud.at(i).y);
		view->setField(pdal::Dimension::Id::Z, i, cloud.at(i).z);
		view->setField(pdal::Dimension::Id::Red, i, cloud.at(i).r);
		view->setField(pdal::Dimension::Id::Green, i, cloud.at(i).g);
		view->setField(pdal::Dimension::Id::Blue, i, cloud.at(i).b);
		if(!cameraIds.empty())
		{
			view->setField(pdal::Dimension::Id::PointSourceId, i, cameraIds.at(i));
		}
		if(!intensities.empty())
		{
			view->setField(pdal::Dimension::Id::Intensity, i, (unsigned short)intensities.at(i));
		}
	}
	bufferReader.addView(view);

	pdal::StageFactory factory;
	std::string ext = UFile::getExtension(filePath);
	pdal::Stage *writer = factory.createStage("writers." + ext);
	if(writer)
	{
		pdal::Options writerOps;
		writerOps.add("filename", filePath);
		if(ext.compare("ply")==0) writerOps.add("storage_mode", binary?"little endian":"ascii"); // PLY
		if(ext.compare("pcd")==0) writerOps.add("compression", binary?"binary":"ascii"); // PCD

		writer->setOptions(writerOps);
		writer->setInput(bufferReader);

		writer->prepare(table);
		writer->execute(table);
	}
	else
	{
		UERROR("PDAL: cannot find writer for extension \"%s\". Available extensions: \"%s\"",
				UFile::getExtension(filePath).c_str(),
				getPDALSupportedWriters().c_str());
		return 1;
	}

	return 0; //success
}

int savePDALFile(const std::string & filePath,
		const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud,
		const std::vector<int> & cameraIds,
		bool binary,
		const std::vector<float> & intensities)
{
	UASSERT_MSG(cameraIds.empty() || cameraIds.size() == cloud.size(),
			uFormat("cameraIds=%d cloud=%d", (int)cameraIds.size(), (int)cloud.size()).c_str());
	UASSERT_MSG(intensities.empty() || intensities.size() == cloud.size(),
			uFormat("intensities=%d cloud=%d", (int)intensities.size(), (int)cloud.size()).c_str());

	pdal::PointTable table;

	if(!intensities.empty() && !cameraIds.empty())
	{
		table.layout()->registerDims({
			pdal::Dimension::Id::X,
			pdal::Dimension::Id::Y,
			pdal::Dimension::Id::Z,
			pdal::Dimension::Id::Red,
			pdal::Dimension::Id::Green,
			pdal::Dimension::Id::Blue,
			pdal::Dimension::Id::NormalX,
			pdal::Dimension::Id::NormalY,
			pdal::Dimension::Id::NormalZ,
			pdal::Dimension::Id::PointSourceId,
			pdal::Dimension::Id::Intensity});
	}
	else if(!intensities.empty())
	{
		table.layout()->registerDims({
			pdal::Dimension::Id::X,
			pdal::Dimension::Id::Y,
			pdal::Dimension::Id::Z,
			pdal::Dimension::Id::Red,
			pdal::Dimension::Id::Green,
			pdal::Dimension::Id::Blue,
			pdal::Dimension::Id::NormalX,
			pdal::Dimension::Id::NormalY,
			pdal::Dimension::Id::NormalZ,
			pdal::Dimension::Id::Intensity});
	}
	else if(!cameraIds.empty())
	{
		table.layout()->registerDims({
			pdal::Dimension::Id::X,
			pdal::Dimension::Id::Y,
			pdal::Dimension::Id::Z,
			pdal::Dimension::Id::Red,
			pdal::Dimension::Id::Green,
			pdal::Dimension::Id::Blue,
			pdal::Dimension::Id::NormalX,
			pdal::Dimension::Id::NormalY,
			pdal::Dimension::Id::NormalZ,
			pdal::Dimension::Id::PointSourceId});
	}
	else
	{
		table.layout()->registerDims({
			pdal::Dimension::Id::X,
			pdal::Dimension::Id::Y,
			pdal::Dimension::Id::Z,
			pdal::Dimension::Id::Red,
			pdal::Dimension::Id::Green,
			pdal::Dimension::Id::Blue,
			pdal::Dimension::Id::NormalX,
			pdal::Dimension::Id::NormalY,
			pdal::Dimension::Id::NormalZ});
	}
	pdal::BufferReader bufferReader;

	pdal::PointViewPtr view(new pdal::PointView(table));
	for(size_t i=0; i<cloud.size(); ++i)
	{
		view->setField(pdal::Dimension::Id::X, i, cloud.at(i).x);
		view->setField(pdal::Dimension::Id::Y, i, cloud.at(i).y);
		view->setField(pdal::Dimension::Id::Z, i, cloud.at(i).z);
		view->setField(pdal::Dimension::Id::Red, i, cloud.at(i).r);
		view->setField(pdal::Dimension::Id::Green, i, cloud.at(i).g);
		view->setField(pdal::Dimension::Id::Blue, i, cloud.at(i).b);
		view->setField(pdal::Dimension::Id::NormalX, i, cloud.at(i).normal_x);
		view->setField(pdal::Dimension::Id::NormalY, i, cloud.at(i).normal_y);
		view->setField(pdal::Dimension::Id::NormalZ, i, cloud.at(i).normal_z);
		if(!cameraIds.empty())
		{
			view->setField(pdal::Dimension::Id::PointSourceId, i, cameraIds.at(i));
		}
		if(!intensities.empty())
		{
			view->setField(pdal::Dimension::Id::Intensity, i, (unsigned short)intensities.at(i));
		}
	}
	bufferReader.addView(view);

	pdal::StageFactory factory;
	std::string ext = UFile::getExtension(filePath);
	pdal::Stage *writer = factory.createStage("writers." + ext);
	if(writer)
	{
		pdal::Options writerOps;
		writerOps.add("filename", filePath);
		if(ext.compare("ply")==0) writerOps.add("storage_mode", binary?"little endian":"ascii"); // PLY
		if(ext.compare("pcd")==0) writerOps.add("compression", binary?"binary":"ascii"); // PCD

		writer->setOptions(writerOps);
		writer->setInput(bufferReader);

		writer->prepare(table);
		writer->execute(table);
	}
	else
	{
		UERROR("PDAL: cannot find writer for extension \"%s\". Available extensions: \"%s\"",
				UFile::getExtension(filePath).c_str(),
				getPDALSupportedWriters().c_str());
		return 1;
	}

	return 0; //success
}

int savePDALFile(const std::string & filePath,
		const pcl::PointCloud<pcl::PointXYZI> & cloud,
		const std::vector<int> & cameraIds,
		bool binary)
{
	UASSERT_MSG(cameraIds.empty() || cameraIds.size() == cloud.size(),
			uFormat("cameraIds=%d cloud=%d", (int)cameraIds.size(), (int)cloud.size()).c_str());

	pdal::PointTable table;

	if(!cameraIds.empty())
	{
		table.layout()->registerDims({
			pdal::Dimension::Id::X,
			pdal::Dimension::Id::Y,
			pdal::Dimension::Id::Z,
			pdal::Dimension::Id::Intensity,
			pdal::Dimension::Id::PointSourceId});
	}
	else
	{
		table.layout()->registerDims({
			pdal::Dimension::Id::X,
			pdal::Dimension::Id::Y,
			pdal::Dimension::Id::Z,
			pdal::Dimension::Id::Intensity});
	}
	pdal::BufferReader bufferReader;

	pdal::PointViewPtr view(new pdal::PointView(table));
	for(size_t i=0; i<cloud.size(); ++i)
	{
		view->setField(pdal::Dimension::Id::X, i, cloud.at(i).x);
		view->setField(pdal::Dimension::Id::Y, i, cloud.at(i).y);
		view->setField(pdal::Dimension::Id::Z, i, cloud.at(i).z);
		view->setField(pdal::Dimension::Id::Intensity, i, (unsigned short)cloud.at(i).intensity);
		if(!cameraIds.empty())
		{
			view->setField(pdal::Dimension::Id::PointSourceId, i, cameraIds.at(i));
		}
	}
	bufferReader.addView(view);

	pdal::StageFactory factory;
	std::string ext = UFile::getExtension(filePath);
	pdal::Stage *writer = factory.createStage("writers." + ext);
	if(writer)
	{
		pdal::Options writerOps;
		writerOps.add("filename", filePath);
		if(ext.compare("ply")==0) writerOps.add("storage_mode", binary?"little endian":"ascii"); // PLY
		if(ext.compare("pcd")==0) writerOps.add("compression", binary?"binary":"ascii"); // PCD

		writer->setOptions(writerOps);
		writer->setInput(bufferReader);

		writer->prepare(table);
		writer->execute(table);
	}
	else
	{
		UERROR("PDAL: cannot find writer for extension \"%s\". Available extensions: \"%s\"",
				UFile::getExtension(filePath).c_str(),
				getPDALSupportedWriters().c_str());
		return 1;
	}

	return 0; //success
}

int savePDALFile(const std::string & filePath,
		const pcl::PointCloud<pcl::PointXYZINormal> & cloud,
		const std::vector<int> & cameraIds,
		bool binary)
{
	UASSERT_MSG(cameraIds.empty() || cameraIds.size() == cloud.size(),
			uFormat("cameraIds=%d cloud=%d", (int)cameraIds.size(), (int)cloud.size()).c_str());

	pdal::PointTable table;

	if(!cameraIds.empty())
	{
		table.layout()->registerDims({
			pdal::Dimension::Id::X,
			pdal::Dimension::Id::Y,
			pdal::Dimension::Id::Z,
			pdal::Dimension::Id::Intensity,
			pdal::Dimension::Id::NormalX,
			pdal::Dimension::Id::NormalY,
			pdal::Dimension::Id::NormalZ,
			pdal::Dimension::Id::PointSourceId});
	}
	else
	{
		table.layout()->registerDims({
			pdal::Dimension::Id::X,
			pdal::Dimension::Id::Y,
			pdal::Dimension::Id::Z,
			pdal::Dimension::Id::Intensity,
			pdal::Dimension::Id::NormalX,
			pdal::Dimension::Id::NormalY,
			pdal::Dimension::Id::NormalZ});
	}
	pdal::BufferReader bufferReader;

	pdal::PointViewPtr view(new pdal::PointView(table));
	for(size_t i=0; i<cloud.size(); ++i)
	{
		view->setField(pdal::Dimension::Id::X, i, cloud.at(i).x);
		view->setField(pdal::Dimension::Id::Y, i, cloud.at(i).y);
		view->setField(pdal::Dimension::Id::Z, i, cloud.at(i).z);
		view->setField(pdal::Dimension::Id::Intensity, i, (unsigned short)cloud.at(i).intensity);
		view->setField(pdal::Dimension::Id::NormalX, i, cloud.at(i).normal_x);
		view->setField(pdal::Dimension::Id::NormalY, i, cloud.at(i).normal_y);
		view->setField(pdal::Dimension::Id::NormalZ, i, cloud.at(i).normal_z);
		if(!cameraIds.empty())
		{
			view->setField(pdal::Dimension::Id::PointSourceId, i, cameraIds.at(i));
		}
	}
	bufferReader.addView(view);

	pdal::StageFactory factory;
	std::string ext = UFile::getExtension(filePath);
	pdal::Stage *writer = factory.createStage("writers." + ext);
	if(writer)
	{
		pdal::Options writerOps;
		writerOps.add("filename", filePath);
		if(ext.compare("ply")==0) writerOps.add("storage_mode", binary?"little endian":"ascii"); // PLY
		if(ext.compare("pcd")==0) writerOps.add("compression", binary?"binary":"ascii"); // PCD

		writer->setOptions(writerOps);
		writer->setInput(bufferReader);

		writer->prepare(table);
		writer->execute(table);
	}
	else
	{
		UERROR("PDAL: cannot find writer for extension \"%s\". Available extensions: \"%s\"",
				UFile::getExtension(filePath).c_str(),
				getPDALSupportedWriters().c_str());
		return 1;
	}

	return 0; //success
}

}

