/*
 * CloudViewerCellPicker.h
 *
 *  Created on: Aug 21, 2018
 *      Author: mathieu
 */

#ifndef GUILIB_SRC_CLOUDVIEWERCELLPICKER_H_
#define GUILIB_SRC_CLOUDVIEWERCELLPICKER_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <vtkCellPicker.h>

namespace rtabmap {

class RTABMAP_GUI_EXPORT CloudViewerCellPicker : public vtkCellPicker {
public:
public:
    static CloudViewerCellPicker *New ();
    vtkTypeMacro(CloudViewerCellPicker, vtkCellPicker);
	CloudViewerCellPicker();
	virtual ~CloudViewerCellPicker();

protected:
	// overrided to ignore back faces
    virtual double IntersectActorWithLine(const double p1[3],
    		const double p2[3],
			double t1, double t2,
			double tol,
			vtkProp3D *prop,
			vtkMapper *mapper);

private:
	vtkGenericCell * cell_; //used to accelerate picking
	vtkIdList * pointIds_; // used to accelerate picking
};

} /* namespace rtabmap */

#endif /* GUILIB_SRC_CLOUDVIEWERCELLPICKER_H_ */
