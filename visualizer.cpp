#include "visualizer.h"
#include "ui_mainwindow.h"

#include <iostream>

#include <QDebug>
#include <QString>
#include <QPushButton>
#include <QMessageBox>
#include <QFileDialog>
#include <QKeyEvent>
#include <QInputDialog>
#include <QFile>
#include <QVTKOpenGLStereoWidget.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkPropPicker.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkTransform.h>
#include <vtkOutputWindow.h>

#include <boost/unordered/unordered_map_fwd.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/common/actor_map.h>
#include <pcl/visualization/PointCloudColorHandlerLUT.h>
#include <pcl/visualization/mouse_event.h>

#include "Annotaion.h"
#include "pcl/visualization/pcl_visualizer_extented.h"
#include "view/flowlayout.h"
#include "project/SystemManager.h"

using namespace pcl::visualization;

#define DEFAULT_POINT  0
#define SELECTED_POINT 1
#define GROUND_POINT   2


std::vector<int> intersectionVector(std::vector<int> &a,std::vector<int> &b){
    std::vector<int> c;
    sort(a.begin(), a.end());
    sort(b.begin(), b.end());
    set_intersection(a.begin(), a.end(), b.begin(), b.end(), back_inserter(c));
    return c;
}

std::vector<int> unionVector(std::vector<int> &a,std::vector<int> &b){
    std::vector<int> c;
    set_union(a.begin(), a.end(), b.begin(), b.end(), back_inserter(c));
    return c;
}

std::vector<int> diffVector(std::vector<int> a,std::vector<int> b){
    std::vector<int> c;
    sort(a.begin(), a.end());
    sort(b.begin(), b.end());
    set_difference(a.begin(), a.end(), b.begin(), b.end(), back_inserter(c));
    return c;
}


Visualizer::Visualizer(QWidget *parent)
    :QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("Micro-i 3D Annotation Tool");

    // init ui
    initialize();
}

Visualizer::~Visualizer(){
    delete ui;
}

void Visualizer::initialize()
{
    // init viewer
    m_viewer.reset(new PCLViewer("", false));
    m_viewer->setBackgroundColor(0.3, 0.3, 0.33);
    m_viewer->setShowFPS(true);
    // init vtk
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    ui->qvtkWidget->renderWindow()->AddRenderer(m_viewer->getRendererCollection()->GetFirstRenderer());
    //
    m_viewer->setupInteractor(ui->qvtkWidget->interactor(), ui->qvtkWidget->renderWindow());
    // add orientation marker widget
    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New ();
    axesWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New ();
    axesWidget->SetOutlineColor (0.9300, 0.5700, 0.1300);
    axesWidget->SetOrientationMarker (axes);
    axesWidget->SetInteractor (ui->qvtkWidget->interactor());
    axesWidget->SetViewport (0.0, 0.0, 0.2, 0.2);
    axesWidget->SetEnabled (true);
    axesWidget->InteractiveOn ();
    // add different types of annotation
    m_Annolayout = new FlowLayout();
    Annotation::getTypes()->clear();
    auto typeList = SystemManager::instance()->getAnnotations();

    for(auto type : typeList){
        Annotation::getTypes()->push_back(type.toStdString());
        addAnnotationTypeFromType(type);
    }

    ui->groupBox_Types->setLayout(m_Annolayout);
    ui->groupBox_Types->setMaximumWidth(200);
    ui->groupBox_Types->setMaximumHeight(200);
    // register call back
    m_viewer->registerKeyboardCallback(std::bind(&Visualizer::KeyboardEventProcess, this, std::placeholders::_1));
    m_viewer->registerAreaPickingCallback(std::bind(&Visualizer::AreaPickingEventProcess, this, std::placeholders::_1));
    m_viewer->registerMouseCallback(std::bind(&Visualizer::MouseEventProcess, this, std::placeholders::_1));
    // signal
    connect(ui->action_Open, &QAction::triggered, this, &Visualizer::openPath);
    connect(ui->action_Save, &QAction::triggered, this, &Visualizer::save);
    connect(ui->slider_pointSize, &QSlider::valueChanged, this, &Visualizer::setPointSize, Qt::QueuedConnection);
    connect(ui->pushButton_modechange, &QPushButton::clicked, this, &Visualizer::virtualKeyClicked);
    connect(ui->listWidget, &QListWidget::itemDoubleClicked, this, &Visualizer::onfileListChanged);
    connect(ui->action_addAnnoType, &QAction::triggered, this, &Visualizer::addAnnotationType);
    connect(ui->action_delAnnoType, &QAction::triggered, this, &Visualizer::delAnnotationType);
    //
    connect(ui->action_visiualize, &QAction::triggered, this, [&]{
        this->showColor(!m_showColor);
        ui->qvtkWidget->renderWindow()->Render();
    });
    //init annotation
    annoManager.reset(new Annotaions());
    // init cloud
    cloud.reset(new PointCloudT);
}

void Visualizer::openPath()
{
    QString openPath = QFileDialog::getExistingDirectory(this, tr("Open OCT bin Path"), SystemManager::instance()->getLastSelectPath());
    if(openPath.isEmpty()){
        return;
    }
    auto manager = SystemManager::instance();
    manager->setProjectPath(openPath);
    ui->listWidget->addItems(manager->getBaseNameList());
    SystemManager::instance()->setLastSelectPath(openPath);
}

void Visualizer::reopenFile()
{
    this->clear();
    //
    loadBinFile(pointcloudFileName);
    //
    this->printMessage(QString("%1 loaded, cloud point number : %2").arg(pointcloudFileName, QString::number(cloud->width * cloud->height)));
    //
    QFileInfo info(pointcloudFileName);
    annotationFileName = QString("%1/%2.json").arg(info.absolutePath(), info.baseName());
    if (QFile::exists(annotationFileName)){
        annoManager->loadAnnotationsFromJson(annotationFileName);
    }
    this->refresh();
}

void Visualizer::clear()
{
    // intensity between
    m_minIntensity = 255;
    m_maxIntensity = 0;
    // annotation
    removeAnnotation();
    annoManager->clear();
    currPickedAnnotation=NULL;
    // cloud
    m_viewer->removeAllPointClouds();

    last_selected_slice.clear();
    cloudLabel.clear();
}

void Visualizer::loadBinFile(const QString& filename_)
{
    if(!cloud){
        qWarning() << "点云文件不存在";
        return;
    }
    std::ifstream input(filename_.toStdString().c_str(), std::ios_base::binary);
    if(!input.good()){
        qWarning() << "Cannot open file : " << filename_;
        return;
    }
    cloud->clear();
    cloud->height = 1;

//    GetBinFileData(filename_.toStdString(),cloud);

    while(input.good() && !input.eof()){
        PointT point;
        uint16_t temp[4] = {0, 0, 0, 0};
        input.read((char *)temp, sizeof(uint16_t) * 4);
        point.x = temp[0];
        point.y = temp[1];
        point.z = temp[2];
        point.intensity = temp[3];
        m_minIntensity = std::min((uint8_t)point.intensity, m_minIntensity);
        m_maxIntensity = std::max((uint8_t)point.intensity, m_maxIntensity);
        cloud->push_back(point);
    }
    input.close();
}


bool Visualizer::GetBinFileData(const std::string &file_path, PointCloudTPtr &bin_cloud_data)
{
    if(file_path.empty()) {
        std::cout << "File path doesn't exist" << std::endl;
        return false;
    }
    FILE* file_open;
    file_open = fopen(file_path.c_str(), "rb");
    if (file_open == nullptr) {
        std::cout << "Open bin file failed" << std::endl;
        return false;
    }
    uint16_t buffer[3]={0};
    uint8_t intensity;
    int fileOffset=0;
    PointT  tempData;
    while(1)
    {
        memset(buffer, 0, sizeof(buffer));
        fread(buffer, sizeof(uint16_t), 3, file_open);
        fread(&intensity, sizeof(uint8_t), 1, file_open);
        fileOffset = (int) ftell(file_open);
        if (feof(file_open)) {
            break;
        }
        tempData.x = float(buffer[0]);
        tempData.y = float(buffer[1]);
        tempData.z = float(buffer[2]);
        tempData.intensity = float(intensity);
        bin_cloud_data->push_back(tempData);
    }
    return true;
}

void Visualizer::refresh()
{
    ui->label_filename->setText(pointcloudFileName);
    cloudLabel = QVector<int>(cloud->size(), 0);

    this->showColor(m_showColor, true);

    showAnnotation();

    m_viewer->resetCamera();
    ui->qvtkWidget->renderWindow()->Render();
}

void Visualizer::save()
{
    annoManager->saveAnnotationsToJson(annotationFileName, QFileInfo(pointcloudFileName).baseName());
}

void Visualizer::onfileListChanged(QListWidgetItem *item)
{
    auto baseName = item->text();
    this->save();
    pointcloudFileName = SystemManager::instance()->getProjectPath() + "/" + baseName + ".bin";
    this->reopenFile();
    this->updateAnnotationButton();
}

void Visualizer::addAnnotationType()
{
    QString name = QInputDialog::getText(this, "新增标注", "名称：");
    if(name.isEmpty()){
        return;
    }
    if(m_annotationButtons.count(name.toStdString())){
        printWarning("缺陷已存在");
        return;
    }
    addAnnotationTypeFromType(name);
    Annotation::getTypes()->push_back(name.toStdString());
}

void Visualizer::addAnnotationTypeFromType(const QString &type)
{
    SystemManager::instance()->addAnnotation(type);
    QPushButton* button = new QPushButton(type);
    connect(button, &QPushButton::clicked, this, [=]() { this->typeButtonClickedProcess(button->text().toStdString()); });
    pcl::RGB c = Annotation::getColor(type.toStdString());
    QPalette pal = button->palette();
    pal.setColor(QPalette::Button, QColor(c.r, c.g, c.b, c.a));
    button->setAutoFillBackground(true);
    button->setPalette(pal);
    button->update();
    m_annotationButtons[type.toStdString()] = button;
    m_Annolayout->addWidget(button);
}

void Visualizer::delAnnotationType()
{
    QString name = QInputDialog::getText(this, "删除标注", "名称：");
    if(name.isEmpty()){
        return;
    }
    if(!m_annotationButtons.contains(name.toStdString())){
        printWarning("输入的标注类型不存在");
    }
    auto annos = annoManager->getAnnotations();

    for(auto anno : annos){
        if(anno->getType() == name.toStdString()){
            printWarning("当前模型中存在此标注，无法删除");
            return;
        }
    }

    SystemManager::instance()->delAnnotation(name);
    auto bt = m_annotationButtons[name.toStdString()];
    m_annotationButtons.remove(name.toStdString());
    m_Annolayout->removeWidget(bt);
    this->update();
}

void Visualizer::updateAnnotationButton()
{
    auto annos = annoManager->getAnnotations();
    for(auto anno : annos){
        if(!m_annotationButtons.count(anno->getType())){
            addAnnotationTypeFromType(QString::fromStdString(anno->getType()));
        }
    }
}

void Visualizer::showColor(bool show, bool add)
{
    if(!cloud){
        return;
    }
    if(cloudLabel.size() != cloud->size()){
        cloudLabel = QVector<int>(cloud->size(), 0);
    }
    colorHandler.setInputCloud(cloud);
    colorHandler.setLabel(cloudLabel);
    colorHandler.setShowMode(show);
    colorHandler.setIntensityBetween(m_minIntensity, m_maxIntensity);
    if (add) {
        m_viewer->addPointCloud<PointT>(cloud, colorHandler, "cloud");
    } else {
        m_viewer->updatePointCloud<PointT>(cloud, colorHandler, "cloud");
    }
    m_showColor = show;
}

void Visualizer::setPointSize(int size)
{
    size = std::clamp(size, 1, 100);
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "cloud");
    ui->qvtkWidget->renderWindow()->Render();
}

void Visualizer::showAnnotation(const Annotation* anno){
    if(!anno) return;
    m_viewer->addActorToRenderer(anno->getActor());
}

void Visualizer::showAnnotation()
{
    for (auto anno : annoManager->getAnnotations()){
        showAnnotation(anno);
    }
}

void Visualizer::removeAnnotation(Annotation *anno)
{
    if(!anno){
        return;
    }
    if (currPickedAnnotation){
        currPickedAnnotation->unpicked();
        currPickedAnnotation=NULL;
    }
    m_viewer->removeActorFromRenderer(anno->getActor());
}

void Visualizer::removeAnnotation()
{
    for (auto anno:annoManager->getAnnotations()){
        removeAnnotation(anno);
    }
}

void Visualizer::pickAnnotation(double x, double y){
    vtkSmartPointer<vtkPropPicker>  picker =
        vtkSmartPointer<vtkPropPicker>::New();
    picker->Pick(x, y, 0, m_viewer->getRendererCollection()->GetFirstRenderer());
    vtkActor* pickedActor = picker->GetActor();

    if (currPickedAnnotation){
        currPickedAnnotation->unpicked();
        currPickedAnnotation=NULL;
    }
    // get the correspond annotation
    currPickedAnnotation = annoManager->getAnnotation(pickedActor);
    if (currPickedAnnotation){
        currPickedAnnotation->picked(ui->qvtkWidget->renderWindow()->GetInteractor());
    }
}

void Visualizer::defaultColorPoint(std::vector<int>& slice)
{
    if (!slice.size()){
        cloudLabel = QVector<int>(cloud->size(), 0);
    }
    for (auto it = slice.begin(); it != slice.end(); it++) {
        if(cloudLabel.size() < *it){
            cloudLabel.resize(*it + 1);
        }
        cloudLabel[*it] = DEFAULT_POINT;
    }
}

void Visualizer::highlightPoint(std::vector<int>& slice)
{
    if (slice.size()<1) return;
    for (std::vector<int>::iterator it = slice.begin(); it != slice.end(); it++) {
        if(cloudLabel.size() < *it){
            cloudLabel.resize(*it + 1);
        }
        cloudLabel[*it] = SELECTED_POINT;
    }
}

void Visualizer::updateCloud()
{
    this->showColor(m_showColor);
    ui->qvtkWidget->renderWindow()->Render();
}

void Visualizer::virtualKeyClicked()
{
    INPUT inputs[2] = {};
    ZeroMemory(inputs, sizeof(inputs));
    ui->qvtkWidget->setFocus();

    inputs[0].type = INPUT_KEYBOARD;
    inputs[0].ki.wVk = 'X';

    inputs[1].type = INPUT_KEYBOARD;
    inputs[1].ki.wVk = 'X';
    inputs[1].ki.dwFlags = KEYEVENTF_KEYUP;

    UINT uSent = SendInput(ARRAYSIZE(inputs), inputs, sizeof(INPUT));
    if (uSent != ARRAYSIZE(inputs))
    {
        qDebug() << "SendInput failed: 0x%x\n" << HRESULT_FROM_WIN32(GetLastError());
    }
}

void Visualizer::KeyboardEventProcess(const KeyboardEvent& event)
{
    // delete annotation
    if (event.getKeySym() == "Delete" && currPickedAnnotation){
        removeAnnotation(currPickedAnnotation);
    }
    if ((event.getKeySym() == "x" || event.getKeySym() == "X") && event.keyUp()){
        m_changeMode = !m_changeMode;
        if (m_changeMode) {
            ui->pushButton_modechange->setText("当前:标注模式");
        } else {
            ui->pushButton_modechange->setText("当前:旋转模式");
        }
    }
}

void Visualizer::MouseEventProcess (const pcl::visualization::MouseEvent& event)
{
    if (event.getButton() == pcl::visualization::MouseEvent::LeftButton
        && event.getType() == pcl::visualization::MouseEvent::MouseButtonPress){
        pickAnnotation(event.getX(), event.getY());
    }
}

void Visualizer::AreaPickingEventProcess (const pcl::visualization::AreaPickingEvent& event)
{
    std::vector<int> new_selected_slice;
    event.getPointsIndices(new_selected_slice);
    if (new_selected_slice.empty()) return;
    if (!m_viewer) return;
    auto winInteractor = m_viewer->getRenderWindowInteractor();
    if (!winInteractor) return;
    int s = winInteractor->GetShiftKey();
    int a = winInteractor->GetControlKey();

    if (!last_selected_slice.empty()){
        defaultColorPoint(last_selected_slice);
    }

    if ( s && a ){ // intersection
        last_selected_slice = intersectionVector(last_selected_slice, new_selected_slice);
    }else if ( s ){ // union
        last_selected_slice = unionVector(last_selected_slice, new_selected_slice);
    }else if ( a ){ // remove
        last_selected_slice = diffVector(last_selected_slice, new_selected_slice);
    }else{ // new
        last_selected_slice = new_selected_slice;
    }

    highlightPoint(last_selected_slice);
    updateCloud();
}

void Visualizer::createAnnotationFromSelectPoints(std::string type)
{
    if (last_selected_slice.size()>3){
        Annotation* anno = new Annotation(cloud, last_selected_slice, type);
        annoManager->push_back(anno);
        showAnnotation(anno);
        ui->qvtkWidget->renderWindow()->Render();
    }else{
        printMessage("no points selected");
    }
}

void Visualizer::typeButtonClickedProcess(std::string type)
{
    if (last_selected_slice.size() > 3){
        createAnnotationFromSelectPoints(type);
        defaultColorPoint(last_selected_slice);
        last_selected_slice.clear();
    } else if (currPickedAnnotation){
        currPickedAnnotation->setType(type);
        ui->qvtkWidget->renderWindow()->Render();
        return;
    }
}

void Visualizer::printMessage(const QString &msg)
{
    ui->statusBar->showMessage(msg);
    qDebug() << msg;
}

void Visualizer::printWarning(const QString &msg)
{
    ui->statusBar->showMessage(msg);
    qWarning() << msg;
}
