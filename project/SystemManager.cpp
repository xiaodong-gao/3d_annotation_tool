#include "SystemManager.h"
#include <QDir>

#include <QDebug>

SystemManager *SystemManager::instance()
{
    static SystemManager manager;
    return &manager;
}

void SystemManager::setProjectPath(const QString &path)
{
    QDir dir(path);
    if(!dir.exists()){
        return;
    }
    m_baseNameList.clear();
    auto entryList = dir.entryInfoList(QDir::Files | QDir::NoDotAndDotDot);
    for(const auto& entry : entryList){
        if(entry.suffix() == MODEL_SUFFIX){
            m_baseNameList.append(entry.completeBaseName());
        }
    }
    m_projectPath = path;
}

void SystemManager::addAnnotation(const QString& type)
{
    QSettings settings("./config.ini", QSettings::IniFormat);
    settings.beginGroup("AnnoType");
    QStringList typeList = settings.value("type").toStringList();
    if(!typeList.contains(type)){
        typeList.push_back(type);
    }
    settings.setValue("type", typeList);
    settings.endGroup();
    settings.sync();
}

void SystemManager::delAnnotation(const QString& type)
{
    QSettings settings("./config.ini", QSettings::IniFormat);
    settings.beginGroup("AnnoType");
    QStringList typeList = settings.value("type").toStringList();
    typeList.removeAll(type);
    settings.setValue("type", typeList);
    settings.endGroup();
    settings.sync();
}

QStringList SystemManager::getAnnotations()
{
    QSettings settings("./config.ini", QSettings::IniFormat);
    settings.beginGroup("AnnoType");
    QStringList typeList = settings.value("type").toStringList();
    settings.endGroup();
    return typeList;
}

void SystemManager::setLastSelectPath(const QString& path)
{
    QSettings settings("./config.ini", QSettings::IniFormat);
    settings.beginGroup("Path");
    settings.setValue("path", path);
    settings.endGroup();
    settings.sync();
    m_lastSelectPath = path;
}

SystemManager::SystemManager()
{
    QSettings settings("./config.ini", QSettings::IniFormat);
    settings.beginGroup("Path");
    m_lastSelectPath = settings.value("path", "./").toString();
    settings.endGroup();
}
