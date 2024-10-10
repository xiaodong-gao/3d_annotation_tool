#ifndef vtkAnnotationBoxSource_h
#define vtkAnnotationBoxSource_h

#include "vtkPolyDataAlgorithm.h"

class  vtkAnnotationBoxSource : public vtkPolyDataAlgorithm
{
public:
  static vtkAnnotationBoxSource *New();
  vtkTypeMacro(vtkAnnotationBoxSource,vtkPolyDataAlgorithm);

protected:
  vtkAnnotationBoxSource();
  ~vtkAnnotationBoxSource() override {}
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *) override;
private:
  vtkAnnotationBoxSource(const vtkAnnotationBoxSource&) = delete;
  void operator=(const vtkAnnotationBoxSource&) = delete;
};

#endif
