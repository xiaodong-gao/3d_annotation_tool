#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "common.h"
#include <QMainWindow>
#include <QString>
#include <QMap>
#include <QPushButton>
#include <string>
#include <stdio.h>
#include <iostream>
#include <pcl/visualization/PointCloudColorHandlerLUT.h>
#include <Annotaion.h>

namespace Ui {
class MainWindow;
}

class Visualizer:public QMainWindow
{
public:
    explicit Visualizer(QWidget *parent = 0);
    ~Visualizer();

    bool GetBinFileData(const std::string &file_path, PointCloudTPtr &bin_cloud_data);
private:
    /**
     * @brief initialize ui slot
     */
    void initialize();
    /**
     * @brief openPath 打开文件夹
     */
    void openPath();
    /**
     * @brief openFile 打开点云文件
     * @param path
     */
    void reopenFile();
    /**
     * @brief clear state before load new cloud and annotation
     */
    void clear();
    /**
     * @brief loadBinFile 读取.bin文件
     * @param filename_
     * @param cloud_
     */
    void loadBinFile(const QString& filename_);
    /**
     * @brief show loaded cloud and annotations
     */
    void refresh();
    /**
     * @brief save 保存文件
     */
    void save();
    /**
     * @brief onfileListChanged 选择的文件变更
     */
    void onfileListChanged(class QListWidgetItem *item);
    /**
     * @brief addAnnotationType 添加缺陷类型
     */
    void addAnnotationType();
    /**
     * @brief addAnnotationTypeFromType
     */
    void addAnnotationTypeFromType(const QString&);
    /**
     * @brief delAnnotationType
     */
    void delAnnotationType();
    /**
     * @brief updateAnnotationButton
     */
    void updateAnnotationButton();
    /**
     * @brief showColor 是否显示颜色
     */
    void showColor(bool, bool = false);
    /**
     * @brief setPointSize 设置点大小
     * @param size
     */
    void setPointSize(int size);
    /**
     * @brief showAnnotation 显示某个标注
     * @param anno
     */
    void showAnnotation(const Annotation* anno);
    /**
     * @brief showAnnotation 显示所有标注
     */
    void showAnnotation();
    /**
     * @brief removeAnnotation 移除某个标注
     * @param anno
     */
    void removeAnnotation(Annotation* anno);
    /**
     * @brief removeAnnotation 移除所有标注
     */
    void removeAnnotation();
    /**
     * @brief pickAnnotation 选择一个标注
     * @param x
     * @param y
     */
    void pickAnnotation(double x,double y);
    /**
     * @brief defaultColorPoint 设置为默认颜色
     * @param slice
     */
    void defaultColorPoint(std::vector<int>& slice);
    /**
     * @brief highlightPoint 高亮显示点云
     * @param slice
     */
    void highlightPoint(std::vector<int>& slice);
    /**
     * @brief updateCloud 更新点云颜色
     */
    void updateCloud();
    /**
     * @brief virtualKeyClicked 模拟键盘点击事件
     */
    void virtualKeyClicked();
    /**
     * @brief KeyboardEventProcess 键盘事件
     * @param event
     */
    void KeyboardEventProcess(const pcl::visualization::KeyboardEvent& event);
    /**
     * @brief MouseEventProcess 鼠标事件
     * @param event
     */
    void MouseEventProcess (const pcl::visualization::MouseEvent& event);
    /**
     * @brief AreaPickingEventProcess 区域选点
     * @param event
     */
    void AreaPickingEventProcess(const pcl::visualization::AreaPickingEvent& event);
    /**
     * @brief createAnnotationFromSelectPoints 从选择的点上创建标注框
     * @param type
     */
    void createAnnotationFromSelectPoints(std::string type="unknown");
    /**
     * @brief typeButtonClickedProcess 创建某个类型的标注
     * @param type
     */
    void typeButtonClickedProcess(std::string type);
    /**
     * @brief printMessage 显示消息
     */
    void printMessage(const QString&);
    /**
     * @brief printWarning 显示警告
     */
    void printWarning(const QString&);

private:
    Ui::MainWindow* ui;
    //
    class FlowLayout *m_Annolayout = nullptr;
    // pcl viewer
    PCLViewerPtr m_viewer;
    // filename
    QString pointcloudFileName;
    QString annotationFileName;
    // cloud data
    PointCloudTPtr cloud;
    // manage annotations
    std::shared_ptr<Annotaions> annoManager;
    // for pick
    Annotation *currPickedAnnotation = nullptr;
    // axes
    vtkSmartPointer<vtkOrientationMarkerWidget> axesWidget;
    // show color
    bool m_showColor = false;
    //
    bool m_changeMode = false;
    // intensity between
    uint8_t m_minIntensity = 255;
    uint8_t m_maxIntensity = 0;
    // 颜色label
    QVector<int> cloudLabel;
    // 颜色管理器
    PointCloudColorHandlerLUT<PointT> colorHandler;
    // 选的点
    std::vector<int> last_selected_slice;
    // 标注按钮
    QMap<std::string, QPushButton*> m_annotationButtons;


};

#endif // VISUALIZER_H
