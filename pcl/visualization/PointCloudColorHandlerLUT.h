#ifndef PointCloudColorHandlerLUT_H
#define PointCloudColorHandlerLUT_H

#include <QVector>
#include <QDebug>
#include <algorithm> // std::clamp

#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/MyCloudLUT.h>
#include <vtkDataArray.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using pcl::visualization::PointCloudColorHandler;

template <typename PointT>
class PointCloudColorHandlerLUT : public PointCloudColorHandler<PointT> {

public:
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    typedef std::shared_ptr<PointCloudColorHandlerLUT<PointT> > Ptr;
    typedef std::shared_ptr<const PointCloudColorHandlerLUT<PointT> > ConstPtr;

    /** \brief Constructor. */
    PointCloudColorHandlerLUT () :
        PointCloudColorHandler<PointT>()
    {
         capable_ = false;
    }

    /** \brief Constructor. */
    PointCloudColorHandlerLUT (const PointCloudConstPtr &cloud):
        PointCloudColorHandler<PointT>(cloud)
    {
         setInputCloud (cloud);
    }

    /** \brief Destructor. */
    virtual ~PointCloudColorHandlerLUT () {}

    /** \brief Check if this handler is capable of handling the input data or not. */
    inline bool
    isCapable () const {
        return (capable_);
    }

    /** \brief Abstract getName method. */
    virtual std::string
    getName () const {
        return "";
    };

    /** \brief Abstract getFieldName method. */
    virtual std::string
    getFieldName () const{
        return "";
    };

    /** \brief Obtain the actual color for the input dataset as vtk scalars.
    * \param[out] scalars the output scalars containing the color for the dataset
    * \return true if the operation was successful (the handler is capable and
    * the input cloud was given as a valid pointer), false otherwise
    */
    virtual bool
    getColor (vtkSmartPointer<vtkDataArray> &scalars) const {
        if (!capable_ || !cloud_)
            return (false);

        if (!scalars)
            scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
        scalars->SetNumberOfComponents (4);
        vtkIdType nr_points = cloud_->points.size ();
        reinterpret_cast<vtkUnsignedCharArray*> (&(*scalars))->SetNumberOfTuples (nr_points);
        unsigned char* colors = reinterpret_cast<vtkUnsignedCharArray*> (&(*scalars))->GetPointer (0);
        int j = 0;

        if (!showMode) {
            for (vtkIdType cp = 0; cp < nr_points; ++cp)
            {
                if (pcl::isFinite (cloud_->points[cp]) && cp < label.size())
                {
                    const pcl::RGB& color = MyCloudLUT::at (label[cp] % MyCloudLUT::size ());
                    colors[j    ] = color.r;
                    colors[j + 1] = color.g;
                    colors[j + 2] = color.b;
                    colors[j + 3] = cloud_->points[cp].intensity;
                    j += 4;
                }
            }
        } else {
            auto a = m_maxIntensity - m_minIntensity + 1;
            for (vtkIdType cp = 0; cp < nr_points; ++cp)
            {
                if (pcl::isFinite (cloud_->points[cp]) && cp < label.size())
                {
                    auto intensity = cloud_->points[cp].intensity;
                    int idx = intensity * 1.0 / a * 10;
                    idx = std::clamp(idx, 0, 9);
                    auto color = colorLut[idx];
                    if(label[cp]){
                        colors[j    ] = 0;
                        colors[j + 1] = 255;
                        colors[j + 2] = 0;
                        colors[j + 3] = 255;
                    }else{
                        colors[j    ] = color[0];
                        colors[j + 1] = color[1];
                        colors[j + 2] = color[2];
                        colors[j + 3] = 255;
                    }

                    j += 4;
                }
            }
        }


        return (true);
    };

    virtual vtkSmartPointer<vtkDataArray> getColor() const {
        vtkSmartPointer<vtkDataArray> scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
        bool succeed = getColor(scalars);
        return succeed ? scalars : 0;
    }

    /** \brief Set the input cloud to be used.
    * \param[in] cloud the input cloud to be used by the handler
    */
    virtual void
    setInputCloud (const PointCloudConstPtr &cloud)
    {
        cloud_ = cloud;
    }

    void setLabel(const QVector<int>& value){
        label = value;
        capable_ = true;
    };

    void setShowMode(int mode){
        showMode = (bool) mode;
    }

    void setIntensityBetween(uint8_t min, uint8_t max){
        m_minIntensity = min;
        m_maxIntensity = max;
    }
private:
    /**
     * @brief array of cloud label
     */
    QVector<int> label;
    int showMode = 0;
    uint8_t m_minIntensity = 0;
    uint8_t m_maxIntensity = 255;
    // Members derived from the base class
    using PointCloudColorHandler<PointT>::cloud_;
    using PointCloudColorHandler<PointT>::capable_;
    using PointCloudColorHandler<PointT>::field_idx_;
    using PointCloudColorHandler<PointT>::fields_;
};

#endif
