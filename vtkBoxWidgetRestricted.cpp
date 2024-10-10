#include <vtkBoxWidgetRestricted.h>

#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkTransform.h>

vtkStandardNewMacro(vtkBoxWidgetRestricted);

void vtkBoxWidgetRestricted::Rotate(int X, int Y, double *p1, double *p2, double *vpn){
    double* pts = static_cast<vtkDoubleArray*>(this->Points->GetData())->GetPointer(0);
    double v[3] = { 0, 0, 0 };

    switch (this->CurrentHexFace) {
    case 0:
    case 1:
        v[0] = p2[0] - p1[0];
        break;
    case 2:
    case 3:
        v[1] = p2[1] - p1[1];
        break;
    case 4:
    case 5:
        v[2] = p2[2] - p1[2];
        break;
    default:
        break;
    }

    for (int i = 0; i < 8; i++)
    {
        *pts++ += v[0];
        *pts++ += v[1];
        *pts++ += v[2];
    }

    this->PositionHandles();
    return;
}
