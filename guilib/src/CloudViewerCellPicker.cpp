/*
 * CloudViewerCellPicker.cpp
 *
 *  Created on: Aug 21, 2018
 *      Author: mathieu
 */

#include "rtabmap/gui/CloudViewerCellPicker.h"

#include <vtkImageData.h>
#include <vtkRenderer.h>
#include <vtkAbstractPicker.h>
#include <vtkPicker.h>
#include <vtkAbstractCellLocator.h>
#include <vtkIdList.h>
#include <vtkCellPicker.h>
#include <vtkLODProp3D.h>
#include <vtkMapper.h>
#include <vtkGenericCell.h>
#include <vtkMath.h>
#include <vtkTexture.h>
#include <vtkObjectFactory.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkProperty.h>

namespace rtabmap {

// Standard VTK macro for *New ()
vtkStandardNewMacro (CloudViewerCellPicker);

CloudViewerCellPicker::CloudViewerCellPicker()
{
	cell_ = vtkGenericCell::New();
	pointIds_ = vtkIdList::New();
}

CloudViewerCellPicker::~CloudViewerCellPicker()
{
	cell_->Delete();
	pointIds_->Delete();
}

double CloudViewerCellPicker::IntersectActorWithLine(const double p1[3],
		const double p2[3],
		double t1, double t2,
		double tol,
		vtkProp3D *prop,
		vtkMapper *mapper)
{
	// This code was taken from the original CellPicker with almost no
	// modification except for the locator and texture additions.

	// Intersect each cell with ray.  Keep track of one closest to
	// the eye (within the tolerance tol) and within the clipping range).
	// Note that we fudge the "closest to" (tMin+this->Tolerance) a little and
	// keep track of the cell with the best pick based on parametric
	// coordinate (pick the minimum, maximum parametric distance). This
	// breaks ties in a reasonable way when cells are the same distance
	// from the eye (like cells laying on a 2D plane).

	vtkDataSet *data = mapper->GetInput();
	double tMin = VTK_DOUBLE_MAX;
	double minPCoords[3];
	double pDistMin = VTK_DOUBLE_MAX;
	vtkIdType minCellId = -1;
	int minSubId = -1;
	double minXYZ[3];
	minXYZ[0] = minXYZ[1] = minXYZ[2] = 0.0;
	double ray[3] = {p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]};
	vtkMath::Normalize(ray);
	vtkActor * actor = vtkActor::SafeDownCast(prop);

	// Polydata has no 3D cells
	int isPolyData = data->IsA("vtkPolyData");

	vtkCollectionSimpleIterator iter;
	vtkAbstractCellLocator *locator = 0;
	this->Locators->InitTraversal(iter);
	while ( (locator = static_cast<vtkAbstractCellLocator *>(
			this->Locators->GetNextItemAsObject(iter))) )
	{
		if (locator->GetDataSet() == data)
		{
			break;
		}
	}

	// Make a new p1 and p2 using the clipped t1 and t2
	double q1[3], q2[3];
	q1[0] = p1[0]; q1[1] = p1[1]; q1[2] = p1[2];
	q2[0] = p2[0]; q2[1] = p2[1]; q2[2] = p2[2];
	if (t1 != 0.0 || t2 != 1.0)
	{
		for (int j = 0; j < 3; j++)
		{
			q1[j] = p1[j]*(1.0 - t1) + p2[j]*t1;
			q2[j] = p1[j]*(1.0 - t2) + p2[j]*t2;
		}
	}

	// Use the locator if one exists for this data
	if (locator)
	{
		vtkSmartPointer<vtkPoints> intersectPoints = vtkSmartPointer<vtkPoints>::New();
		vtkSmartPointer<vtkIdList> intersectCells = vtkSmartPointer<vtkIdList>::New();

		locator->IntersectWithLine(q1, q2, intersectPoints, intersectCells);
		for(int i = 0; i < intersectPoints->GetNumberOfPoints(); i++ )
		{
			double intersection[3];
			intersectPoints->GetPoint(i, intersection);
		}

		if (!locator->IntersectWithLine(q1, q2, tol, tMin, minXYZ,
				minPCoords, minSubId, minCellId,
				this->cell_))
		{
			return VTK_DOUBLE_MAX;
		}

		// Stretch tMin out to the original range
		if (t1 != 0.0 || t2 != 1.0)
		{
			tMin = t1*(1.0 - tMin) + t2*tMin;
		}

		// If cell is a strip, then replace cell with a sub-cell
		this->SubCellFromCell(this->cell_, minSubId);
	}
	else
	{
		vtkIdList *pointIds = this->pointIds_;
		vtkIdType numCells = data->GetNumberOfCells();

		for (vtkIdType cellId = 0; cellId < numCells; cellId++)
		{
			double t;
			double x[3];
			double pcoords[3];
			pcoords[0] = pcoords[1] = pcoords[2] = 0;
			int newSubId = -1;
			int numSubIds = 1;

			// If it is a strip, we need to iterate over the subIds
			int cellType = data->GetCellType(cellId);
			int useSubCells = this->HasSubCells(cellType);
			if (useSubCells)
			{
				// Get the pointIds for the strip and the length of the strip
				data->GetCellPoints(cellId, pointIds);
				numSubIds = this->GetNumberOfSubCells(pointIds, cellType);
			}

			// This will only loop once unless we need to deal with a strip
			for (int subId = 0; subId < numSubIds; subId++)
			{
				if (useSubCells)
				{
					// Get a sub-cell from a the strip
					this->GetSubCell(data, pointIds, subId, cellType, this->cell_);
				}
				else
				{
					data->GetCell(cellId, this->cell_);
				}

				int cellPicked = 0;
				if (isPolyData)
				{
					// Polydata can always be picked with original endpoints
					cellPicked = this->cell_->IntersectWithLine(
							const_cast<double *>(p1), const_cast<double *>(p2),
							tol, t, x, pcoords, newSubId);
				}
				else
				{
					// Any 3D cells need to be intersected with a line segment that
					// has been clipped with the clipping planes, in case one end is
					// actually inside the cell.
					cellPicked = this->cell_->IntersectWithLine(
							q1, q2, tol, t, x, pcoords, newSubId);

					// Stretch t out to the original range
					if (t1 != 0.0 || t2 != 1.0)
					{
						t = t1*(1.0 - t) + t2*t;
					}
				}

				if (cellPicked && t <= (tMin + this->Tolerance) && t >= t1 && t <= t2)
				{
					double pDist = this->cell_->GetParametricDistance(pcoords);
					if (pDist < pDistMin || (pDist == pDistMin && t < tMin))
					{
////////////////////////////////////////////////////////////////////////////////////
// BEGIN: Modifications from VTK 6.2
////////////////////////////////////////////////////////////////////////////////////
						bool visible = true;
						if(actor->GetProperty()->GetBackfaceCulling() ||
						   actor->GetProperty()->GetFrontfaceCulling())
						{
							// Get the cell weights
							vtkIdType numPoints = this->cell_->GetNumberOfPoints();
							double *weights = new double[numPoints];
							for (vtkIdType i = 0; i < numPoints; i++)
							{
								weights[i] = 0;
							}

							// Get the interpolation weights (point is thrown away)
							double point[3] = {0.0,0.0,0.0};
							this->cell_->EvaluateLocation(minSubId, minPCoords, point, weights);

							double normal[3] = {0.0,0.0,0.0};

							if (this->ComputeSurfaceNormal(data, this->cell_, weights, normal))
							{
								if(actor->GetProperty()->GetBackfaceCulling())
								{
									visible = ray[0]*normal[0] + ray[1]*normal[1] + ray[2]*normal[2] <= 0;
								}
								else
								{
									visible = ray[0]*normal[0] + ray[1]*normal[1] + ray[2]*normal[2] >= 0;
								}
							}
							delete [] weights;
						}
						if(visible)
						{
							tMin = t;
							pDistMin = pDist;
							// save all of these
							minCellId = cellId;
							minSubId = newSubId;
							if (useSubCells)
							{
								minSubId = subId;
							}
							for (int k = 0; k < 3; k++)
							{
								minXYZ[k] = x[k];
								minPCoords[k] = pcoords[k];
							}
						}
////////////////////////////////////////////////////////////////////////////////////
// END: Modifications from VTK 6.2
////////////////////////////////////////////////////////////////////////////////////
					} // for all subIds
				} // if minimum, maximum
			} // if a close cell
		} // for all cells
	}

	// Do this if a cell was intersected
	if (minCellId >= 0 && tMin < this->GlobalTMin)
	{
		this->ResetPickInfo();

		// Get the cell, convert to triangle if it is a strip
		vtkGenericCell *cell = this->cell_;

		// If we used a locator, we already have the picked cell
		if (!locator)
		{
			int cellType = data->GetCellType(minCellId);

			if (this->HasSubCells(cellType))
			{
				data->GetCellPoints(minCellId, this->pointIds_);
				this->GetSubCell(data, this->pointIds_, minSubId, cellType, cell);
			}
			else
			{
				data->GetCell(minCellId, cell);
			}
		}

		// Get the cell weights
		vtkIdType numPoints = cell->GetNumberOfPoints();
		double *weights = new double[numPoints];
		for (vtkIdType i = 0; i < numPoints; i++)
		{
			weights[i] = 0;
		}

		// Get the interpolation weights (point is thrown away)
		double point[3];
		cell->EvaluateLocation(minSubId, minPCoords, point, weights);

		this->Mapper = mapper;

		// Get the texture from the actor or the LOD
		vtkActor *actor = 0;
		vtkLODProp3D *lodActor = 0;
		if ( (actor = vtkActor::SafeDownCast(prop)) )
		{
			this->Texture = actor->GetTexture();
		}
		else if ( (lodActor = vtkLODProp3D::SafeDownCast(prop)) )
		{
			int lodId = lodActor->GetPickLODID();
			lodActor->GetLODTexture(lodId, &this->Texture);
		}

		if (this->PickTextureData && this->Texture)
		{
			// Return the texture's image data to the user
			vtkImageData *image = this->Texture->GetInput();
			this->DataSet = image;

			// Get and check the image dimensions
			int extent[6];
			image->GetExtent(extent);
			int dimensionsAreValid = 1;
			int dimensions[3];
			for (int i = 0; i < 3; i++)
			{
				dimensions[i] = extent[2*i + 1] - extent[2*i] + 1;
				dimensionsAreValid = (dimensionsAreValid && dimensions[i] > 0);
			}

			// Use the texture coord to set the information
			double tcoord[3];
			if (dimensionsAreValid &&
					this->ComputeSurfaceTCoord(data, cell, weights, tcoord))
			{
				// Take the border into account when computing coordinates
				double x[3];
				x[0] = extent[0] + tcoord[0]*dimensions[0] - 0.5;
				x[1] = extent[2] + tcoord[1]*dimensions[1] - 0.5;
				x[2] = extent[4] + tcoord[2]*dimensions[2] - 0.5;

				this->SetImageDataPickInfo(x, extent);
			}
		}
		else
		{
			// Return the polydata to the user
			this->DataSet = data;
			this->CellId = minCellId;
			this->SubId = minSubId;
			this->PCoords[0] = minPCoords[0];
			this->PCoords[1] = minPCoords[1];
			this->PCoords[2] = minPCoords[2];

			// Find the point with the maximum weight
			double maxWeight = 0;
			vtkIdType iMaxWeight = -1;
			for (vtkIdType i = 0; i < numPoints; i++)
			{
				if (weights[i] > maxWeight)
				{
					iMaxWeight = i;
				}
			}

			// If maximum weight is found, use it to get the PointId
			if (iMaxWeight != -1)
			{
				this->PointId = cell->PointIds->GetId(iMaxWeight);
			}
		}

		// Set the mapper position
		this->MapperPosition[0] = minXYZ[0];
		this->MapperPosition[1] = minXYZ[1];
		this->MapperPosition[2] = minXYZ[2];

		// Compute the normal
		if (!this->ComputeSurfaceNormal(data, cell, weights, this->MapperNormal))
		{
			// By default, the normal points back along view ray
			this->MapperNormal[0] = p1[0] - p2[0];
			this->MapperNormal[1] = p1[1] - p2[1];
			this->MapperNormal[2] = p1[2] - p2[2];
			vtkMath::Normalize(this->MapperNormal);
		}

		delete [] weights;
	}

	return tMin;
}

} /* namespace rtabmap */
