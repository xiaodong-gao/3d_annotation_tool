#ifndef vtkBoxWidgetRestricted_h
#define vtkBoxWidgetRestricted_h

#include <vtkBoxWidget.h>

/**
 * @brief The vtkBoxWidgetRestricted class
 * change rotation to move toward a specific direction
 */

class vtkBoxWidgetRestricted: public vtkBoxWidget{
public:
    static vtkBoxWidgetRestricted *New();

    vtkTypeMacro(vtkBoxWidgetRestricted, vtkBoxWidget);

    virtual void Rotate(int X, int Y, double *p1, double *p2, double *vpn) override;
};
#endif
