#include <vtkBoxWidgetCallback.h>
#include <Annotaion.h>
#include <vtkBoxWidget.h>
#include <vtkTransform.h>

vtkBoxWidgetCallback0 *vtkBoxWidgetCallback0::New()
{
    return new vtkBoxWidgetCallback0;
}

void vtkBoxWidgetCallback0::Execute(vtkObject *caller, unsigned long, void *)
{
    vtkSmartPointer<vtkTransform> t =
            vtkSmartPointer<vtkTransform>::New();
    vtkBoxWidget *widget = reinterpret_cast<vtkBoxWidget*>(caller);

    widget->GetTransform( t );

    if (anno){
        anno->applyTransform(t);
    }
}

void vtkBoxWidgetCallback0::setAnno(Annotation *value){
    anno = value;
}

vtkBoxWidgetCallback1 *vtkBoxWidgetCallback1::New()
{
    return new vtkBoxWidgetCallback1;
}

void vtkBoxWidgetCallback1::Execute(vtkObject *caller, unsigned long, void *)
{
    if (anno) {
        anno->adjustToAnchor();
    }
}

void vtkBoxWidgetCallback1::setAnno(Annotation *value)
{
    anno = value;
}
