#ifndef SystemManager_H
#define SystemManager_H

#include <QSettings>
#include <QString>
#include <QList>

#define MODEL_SUFFIX "bin"

class SystemManager{
public:
    static SystemManager* instance();
    // projectPath
    void setProjectPath(const QString& path);
    //
    const QString& getProjectPath() { return m_projectPath; }
    QList<QString> getBaseNameList() { return m_baseNameList; }
    //
    void addAnnotation(const QString& type);
    void delAnnotation(const QString& type);
    QStringList getAnnotations();
    //
    void setLastSelectPath(const QString& path);
    QString getLastSelectPath() { return m_lastSelectPath; };
private:
    SystemManager();
private:
    QSettings m_settings;
    QString m_projectPath = "./";
    QString m_lastSelectPath = "./";
    QList<QString> m_baseNameList;
};

#endif
