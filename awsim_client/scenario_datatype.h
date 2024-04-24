#ifndef SCENARIODATATYPE_H
#define SCENARIODATATYPE_H
#include <QVector3D>
#include <QQuaternion>

struct ScenarioDataType
{
    int a = 1234;
};

struct WayPoint
{
    int laneid;
    double s;
    double t;
};

class Vehicle
{
public:
    QString name;
    QVector3D localPosition;
    QVector3D rotation;
    QVector3D size;
    QVector3D forward;

    Vehicle() {}
    Vehicle(const QString &name, const QVector3D &localPosition, const QVector3D &rotation, const QVector3D &size,const QVector3D &forward)
        : name(name), localPosition(localPosition), rotation(rotation), size(size), forward(forward) {}
};

struct LidarInfo {
    QString name;
    QVector3D position;
    QVector3D rotation;
};




#endif // SCENARIODATATYPE_H
