#ifndef POINTCLOUDWIDGET_H
#define POINTCLOUDWIDGET_H


#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QMouseEvent>
#include <QFile>
#include <QVector>
#include <QPainter>
#include <QString>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <QPainter>
//#include <ft2build.h>
//#include FT_FREETYPE_H

//#include "opengllib_global.h"

class  PointCloudOpenGLWidget: public QOpenGLWidget, public QOpenGLFunctions_3_3_Core
{
    Q_OBJECT
public:
    PointCloudOpenGLWidget(QWidget *parent = 0);
    ~PointCloudOpenGLWidget();
    void updatePoints(const QVector<QVector3D> &points);
    void loadCsvFile(const QString &path);
    void loadBinFile(const QString &path);
    void loadFromRos(const QVector<QVector3D> &points);
    void clearCube();
    QVector<GLfloat> tempLineVertices = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    QVector<GLfloat> tempCubeVertices = {
        -0.0f, -0.0f, -0.0f,
        0.0f, -0.0f, -0.0f,
        0.0f,  0.0f, -0.0f,
        -0.0f,  0.0f, -0.0f,
        -0.0f, -0.0f,  0.0f,
        0.0f, -0.0f,  0.0f,
        0.0f,  0.0f,  0.0f,
        -0.0f,  0.0f,  0.0f,
    };

    struct CubeData {
        QVector3D position;
        QVector3D scale;
        QQuaternion rotation;
        QVector<GLfloat> vertices;
        QVector<GLuint> indices;
        GLuint VAO;
        GLuint VBO;
        GLuint EBO;
        QString name = "car";
    };

    //QVector<GLuint> tempCubeIndices = {
    //    0, 0, 0, 0, 0, 0, 0, 0,
    //    0, 0, 0, 0, 0, 0, 0, 0,
    //    0, 0, 0, 0, 0, 0, 0, 0
    //};


    QVector<GLfloat> cubeVertices = {
        -5.5f, -5.5f, -5.5f,
        5.5f, -5.5f, -5.5f,
        5.5f,  5.5f, -5.5f,
        -5.5f,  5.5f, -5.5f,
        -5.5f, -5.5f,  5.5f,
        5.5f, -5.5f,  5.5f,
        5.5f,  5.5f,  10.5f,
        -5.5f,  5.5f,  5.5f
    };

    QVector<GLuint> cubeIndices = {
        0, 1, 1, 2, 2, 3, 3, 0,    // bottom face
        4, 5, 5, 6, 6, 7, 7, 4,    // top face
        0, 4, 1, 5, 2, 6, 3, 7     // vertical edges
    };




public slots:
    void paintLine(QVector<GLfloat>,QVector<GLuint>);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event);

    virtual unsigned int drawMeshline(float size, int count);
    virtual void drawCooraxis(float length);
    virtual unsigned int drawPointdata(std::vector<float> &pointVertexs);

protected:
    QOpenGLShaderProgram m_shaderProgramMesh;
    QOpenGLShaderProgram m_shaderProgramAxis;
    QOpenGLShaderProgram m_shaderProgramPoint;
    QOpenGLShaderProgram m_shaderProgramLine;

    unsigned int m_VBO_Cube, m_VAO_Cube, m_EBO_Cube;
    //QVector<GLfloat> cubeVertices = tempCubeVertices;
    //QVector<GLuint> cubeIndices = tempCubeIndices;
    std::vector<float> axis_vertexs;


    unsigned int m_VBO_MeshLine;
    unsigned int m_VAO_MeshLine;

    unsigned int m_VBO_Axis;
    unsigned int m_VAO_Axis;

    unsigned int m_VBO_Point;
    unsigned int m_VAO_Point;

    std::vector<float> m_pointData;
    unsigned int m_pointCount;

    unsigned int m_vertexCount;

    float m_xRotate;
    float m_zRotate;
    float m_xTrans;
    float m_yTrans;
    float m_zoom;

    QPoint   lastPos;

    QVector<CubeData> m_cubes;
    QVector<GLfloat> computeCubeVerticesOpenGL(const QVector3D& centerUnity, const QVector3D& sizeUnity, const QVector3D& rotation, const QVector3D& forward);
    QVector<GLfloat> computeCubeVerticesOpenGL_mid70(const QVector3D& centerUnity, const QVector3D& sizeUnity, const QVector3D& rotation, const QVector3D& forward);
    //FT_Library ft;
    //FT_Face face;
    //GLuint createCharTexture(FT_Face face, char c);
    //QPainter painter;
    QImage image;
    QOpenGLTexture *texture = nullptr;
    void drawString(char *string);
    int temp_a=0;
    int init;
public:
    void addCube(const QVector3D& position, const QVector3D& scale, const QVector3D& rotation, const QVector3D& forward);
    void drawText();
};

#endif // POINTCLOUDWIDGET_H
