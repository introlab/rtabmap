/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
// Authors:
//  * Anatoly Baksheev, Itseez Inc.  myname.mysurname <> mycompany.com
//
//M*/

#include "vtkImageMatSource.h"
#include <vtkImageData.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkObjectFactory.h>
#include <vtkVersionMacros.h>

namespace rtabmap {
	vtkStandardNewMacro(vtkImageMatSource);
}

rtabmap::vtkImageMatSource::vtkImageMatSource()
{
    this->SetNumberOfInputPorts(0);
    this->ImageData = vtkSmartPointer<vtkImageData>::New();
}

int rtabmap::vtkImageMatSource::RequestInformation(vtkInformation *, vtkInformationVector**, vtkInformationVector *outputVector)
{
    vtkInformation* outInfo = outputVector->GetInformationObject(0);

    outInfo->Set(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(), this->ImageData->GetExtent(), 6);
    outInfo->Set(vtkDataObject::SPACING(), 1.0, 1.0, 1.0);
    outInfo->Set(vtkDataObject::ORIGIN(),  0.0, 0.0, 0.0);

    vtkDataObject::SetPointDataActiveScalarInfo(outInfo, this->ImageData->GetScalarType(), this->ImageData->GetNumberOfScalarComponents());
    return 1;
}

int rtabmap::vtkImageMatSource::RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector *outputVector)
{
     vtkInformation *outInfo = outputVector->GetInformationObject(0);

     vtkImageData *output = vtkImageData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()) );
     output->ShallowCopy(this->ImageData);
     return 1;
}

void rtabmap::vtkImageMatSource::SetImage(cv::InputArray _image)
{
    CV_Assert(_image.depth() == CV_8U && (_image.channels() == 1 || _image.channels() == 3 || _image.channels() == 4));

	cv::Mat image = _image.getMat();

    this->ImageData->SetDimensions(image.cols, image.rows, 1);
#if VTK_MAJOR_VERSION <= 5
    this->ImageData->SetNumberOfScalarComponents(image.channels());
    this->ImageData->SetScalarTypeToUnsignedChar();
    this->ImageData->AllocateScalars();
#else
    this->ImageData->AllocateScalars(VTK_UNSIGNED_CHAR, image.channels());
#endif

    switch(image.channels())
    {
    case 1: copyGrayImage(image, this->ImageData); break;
    case 3: copyRGBImage (image, this->ImageData); break;
    case 4: copyRGBAImage(image, this->ImageData); break;
    }
    this->ImageData->Modified();
}

void rtabmap::vtkImageMatSource::copyGrayImage(const cv::Mat &source, vtkSmartPointer<vtkImageData> output)
{
    unsigned char* dptr = reinterpret_cast<unsigned char*>(output->GetScalarPointer());
    size_t elem_step = output->GetIncrements()[1]/sizeof(unsigned char);

    for (int y = 0; y < source.rows; ++y)
    {
        unsigned char* drow = dptr + elem_step * y;
        const unsigned char *srow = source.ptr<unsigned char>(source.rows-(y+1)); // vertical flip for texturing
        for (int x = 0; x < source.cols; ++x)
            drow[x] = *srow++;
    }
}

void rtabmap::vtkImageMatSource::copyRGBImage(const cv::Mat &source, vtkSmartPointer<vtkImageData> output)
{
	cv::Vec3b* dptr = reinterpret_cast<cv::Vec3b*>(output->GetScalarPointer());
    size_t elem_step = output->GetIncrements()[1]/sizeof(cv::Vec3b);

    for (int y = 0; y < source.rows; ++y)
    {
		cv::Vec3b* drow = dptr + elem_step * y;
        const unsigned char *srow = source.ptr<unsigned char>(source.rows - (y + 1)); // vertical flip for texturing
        for (int x = 0; x < source.cols; ++x, srow += source.channels())
            drow[x] = cv::Vec3b(srow[2], srow[1], srow[0]);
    }
}

void rtabmap::vtkImageMatSource::copyRGBAImage(const cv::Mat &source, vtkSmartPointer<vtkImageData> output)
{
    cv::Vec4b* dptr = reinterpret_cast<cv::Vec4b*>(output->GetScalarPointer());
    size_t elem_step = output->GetIncrements()[1]/sizeof(cv::Vec4b);

    for (int y = 0; y < source.rows; ++y)
    {
		cv::Vec4b* drow = dptr + elem_step * y;
        const unsigned char *srow = source.ptr<unsigned char>(source.rows - (y + 1)); // vertical flip for texturing
        for (int x = 0; x < source.cols; ++x, srow += source.channels())
            drow[x] = cv::Vec4b(srow[2], srow[1], srow[0], srow[3]);
    }
}
