#include "point_cloud_widget.h"
#include <QDebug>

PointCloudOpenGLWidget::PointCloudOpenGLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
    m_xRotate = -30.0;
    m_zRotate = 100.0;
    m_xTrans = 0.0;
    m_yTrans = 0.0;
    m_zoom = 45.0;
}

PointCloudOpenGLWidget::~PointCloudOpenGLWidget()
{
    makeCurrent();
    glDeleteBuffers(1, &m_VBO_MeshLine);
    glDeleteVertexArrays(1, &m_VAO_MeshLine);

    glDeleteBuffers(1, &m_VBO_Axis);
    glDeleteVertexArrays(1, &m_VAO_Axis);

    glDeleteBuffers(1, &m_VBO_Point);
    glDeleteVertexArrays(1, &m_VAO_Point);

    m_shaderProgramMesh.release();
    m_shaderProgramAxis.release();
    m_shaderProgramPoint.release();

    glDeleteVertexArrays(1, &m_VAO_Cube);
    glDeleteBuffers(1, &m_VBO_Cube);
    glDeleteBuffers(1, &m_EBO_Cube);

    for (const CubeData& cube : m_cubes) {
        glDeleteVertexArrays(1, &cube.VAO);
        glDeleteBuffers(1, &cube.VBO);
        glDeleteBuffers(1, &cube.EBO);
    }

    doneCurrent();
    qDebug() << __FUNCTION__;
}

void PointCloudOpenGLWidget::updatePoints(const QVector<QVector3D> &points)
{
    m_pointData.clear();
    for(auto vector3D : points)
    {
        m_pointData.push_back(vector3D.x());
        m_pointData.push_back(vector3D.y());
        m_pointData.push_back(vector3D.z());
        m_pointData.push_back(1);
    }
}

void PointCloudOpenGLWidget::loadCsvFile(const QString &path)
{
    m_pointData.clear();
    QFile inFile(path);
    if (inFile.open(QIODevice::ReadOnly))
    {
        QTextStream stream_text(&inFile);
        while (!stream_text.atEnd())
        {
            QString line = stream_text.readLine();
            QStringList strSplit = line.split(",");

            double x = strSplit.value(0).toDouble();
            double y = strSplit.value(1).toDouble();
            double z = strSplit.value(2).toDouble();
            m_pointData.push_back(x);
            m_pointData.push_back(y);
            m_pointData.push_back(z);
            m_pointData.push_back(1);
        }
        inFile.close();
    }
}

void PointCloudOpenGLWidget::loadBinFile(const QString &path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        qWarning() << "Cannot open file" << path;
        return;
    }

    // 获取文件大小
    qint64 fileSize = file.size();

    // 检查文件大小是否是 16 的倍数（每个点的数据是 4 个浮点数，每个浮点数是 4 字节）
    if (fileSize % 16 != 0)
    {
        qWarning() << "Invalid file size" << path;
        return;
    }

    // 读取文件数据
    QByteArray data = file.readAll();

    // 关闭文件
    file.close();

    // 检查数据大小
    if (data.size() != fileSize)
    {
        qWarning() << "Failed to read data from" << path;
        return;
    }

    // 获取点的数量
    unsigned int pointCount = fileSize / 16;

    // 创建点云数据数组
    QVector<QVector3D> points(pointCount);

    // 将数据复制到点云数据数组
    float *dataPtr = reinterpret_cast<float *>(data.data());
    for (unsigned int i = 0; i < pointCount; ++i)
    {
        float x = dataPtr[i * 4];
        float y = dataPtr[i * 4 + 1];
        float z = dataPtr[i * 4 + 2];
        points[i] = QVector3D(x, y, z);
    }

    // 更新点云数据
    updatePoints(points);
}

void PointCloudOpenGLWidget::loadFromRos(const QVector<QVector3D> &points)
{

    updatePoints(points);
}

void PointCloudOpenGLWidget::initializeGL()
{

    initializeOpenGLFunctions();

    // enable depth_test
    glEnable(GL_DEPTH_TEST);

    // link meshline shaders   vs文件为顶点着色器  fs为片段着色器
    m_shaderProgramMesh.addShaderFromSourceFile(QOpenGLShader::Vertex,
                                                "/home/zzhhaa1/Desktop/awsim_client/shader/shader_mesh.vs");
    m_shaderProgramMesh.addShaderFromSourceFile(QOpenGLShader::Fragment,
                                                "/home/zzhhaa1/Desktop/awsim_client/shader/shader_mesh.fs");
    m_shaderProgramMesh.link();

    // link coordinate axis shaders
    m_shaderProgramAxis.addShaderFromSourceFile(QOpenGLShader::Vertex,
                                                "/home/zzhhaa1/Desktop/awsim_client/shader/shader_axis.vs");
    m_shaderProgramAxis.addShaderFromSourceFile(QOpenGLShader::Fragment,
                                                "/home/zzhhaa1/Desktop/awsim_client/shader/shader_axis.fs");
    m_shaderProgramAxis.link();

    // link pointcloud shaders
    m_shaderProgramPoint.addShaderFromSourceFile(QOpenGLShader::Vertex,
                                                 "/home/zzhhaa1/Desktop/awsim_client/shader/shader_point.vs");
    m_shaderProgramPoint.addShaderFromSourceFile(QOpenGLShader::Fragment,
                                                 "/home/zzhhaa1/Desktop/awsim_client/shader/shader_point.fs");
    m_shaderProgramPoint.link();




    // line
    m_shaderProgramLine.addShaderFromSourceFile(QOpenGLShader::Vertex, "/home/zzhhaa1/Desktop/awsim_client/shader/shader_point.vs");
    m_shaderProgramLine.addShaderFromSourceFile(QOpenGLShader::Fragment, "/home/zzhhaa1/Desktop/awsim_client/shader/shader_point.fs");
    m_shaderProgramLine.link();

    glGenVertexArrays(1, &m_VAO_Cube);
    glGenBuffers(1, &m_VBO_Cube);
    glGenBuffers(1, &m_EBO_Cube);

    // Bind VAO
    //glBindVertexArray(m_VAO_Cube);

    // Bind and set vertex data
    //glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Cube);
    //glBufferData(GL_ARRAY_BUFFER, cubeVertices.size() * sizeof(GLfloat), cubeVertices.constData(), GL_STATIC_DRAW);

    // Bind and set indices
    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_EBO_Cube);
    //glBufferData(GL_ELEMENT_ARRAY_BUFFER, cubeIndices.size() * sizeof(GLuint), cubeIndices.constData(), GL_STATIC_DRAW);

    // Vertex attributes
    //glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    //glEnableVertexAttribArray(0);

    // Unbind VAO and VBO
    //glBindBuffer(GL_ARRAY_BUFFER, 0);
    //glBindVertexArray(0);

    glGenVertexArrays(1, &m_VAO_Point);
    glGenBuffers(1, &m_VBO_Point);

    m_vertexCount = drawMeshline(2.0, 16);
    m_pointCount = drawPointdata(m_pointData);
    //qDebug() << "point_count" << m_pointCount;
    drawCooraxis(4.0);
    int argc = 1;
    char *argv[1] = {(char*)"Something"};
    glutInit(&argc, argv);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

}

void PointCloudOpenGLWidget::paintGL()
{

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /*
       为了将坐标从一个坐标系转换到另一个坐标系，需要用到几个转换矩阵，
       分别是模型(Model)、视图(View)、投影(Projection)三个矩阵。
    */
    QMatrix4x4 projection, view, model;
    //透视矩阵变换
    projection.perspective(m_zoom, (float)width() / (float)height(), 1.0f, 100.0f);

    // eye：摄像机位置  center：摄像机看的点位 up：摄像机上方的朝向
    view.lookAt(QVector3D(0.0, 0.0, 50.0), QVector3D(0.0, 0.0, 1.0), QVector3D(0.0, 1.0, 0.0));

    model.translate(m_xTrans, m_yTrans, 0.0);
    model.rotate(m_xRotate, 1.0, 0.0, 0.0);
    model.rotate(m_zRotate, 0.0, 0.0, 1.0);

    m_shaderProgramMesh.bind();
    m_shaderProgramMesh.setUniformValue("projection", projection);
    m_shaderProgramMesh.setUniformValue("view", view);
    m_shaderProgramMesh.setUniformValue("model", model);

    m_shaderProgramAxis.bind();
    m_shaderProgramAxis.setUniformValue("projection", projection);
    m_shaderProgramAxis.setUniformValue("view", view);
    m_shaderProgramAxis.setUniformValue("model", model);

    m_shaderProgramPoint.bind();
    m_shaderProgramPoint.setUniformValue("projection", projection);
    m_shaderProgramPoint.setUniformValue("view", view);
    m_shaderProgramPoint.setUniformValue("model", model);

    m_shaderProgramLine.bind();
    m_shaderProgramLine.setUniformValue("projection", projection);
    m_shaderProgramLine.setUniformValue("view", view);
    m_shaderProgramLine.setUniformValue("model", model);

    //画网格
    m_shaderProgramMesh.bind();
    glBindVertexArray(m_VAO_MeshLine);
    glLineWidth(1.0f);
    glDrawArrays(GL_LINES, 0, m_vertexCount);

    //画坐标轴
    m_shaderProgramAxis.bind();
    glBindVertexArray(m_VAO_Axis);
    glLineWidth(5.0f);
    glDrawArrays(GL_LINES, 0, 6);

    //画点云
    m_shaderProgramPoint.bind();
    glBindVertexArray(m_VAO_Point);
    glPointSize(1.0f);
    glDrawArrays(GL_POINTS, 0, m_pointCount);
    //glDeleteBuffers(1, &m_VBO    QMatrix4x4 projection, view, model;
    //透视矩阵变换
    projection.perspective(m_zoom, (float)width() / (float)height(), 1.0f, 100.0f);

    // eye：摄像机位置  center：摄像机看的点位 up：摄像机上方的朝向
    view.lookAt(QVector3D(0.0, 0.0, 50.0), QVector3D(0.0, 0.0, 1.0), QVector3D(0.0, 1.0, 0.0));

    model.translate(m_xTrans, m_yTrans, 0.0);
    model.rotate(m_xRotate, 1.0, 0.0, 0.0);
    model.rotate(m_zRotate, 0.0, 0.0, 1.0);
    //glDeleteVertexArrays(1, &m_VAO_Point);
    m_pointCount = drawPointdata(m_pointData);

    // Drawing the cube
    //m_shaderProgramPoint.bind();  // 假设 m_shaderProgramCube 是您的长方体线框的着色器程序
    //glBindVertexArray(m_VAO_Cube);
    //glDrawElements(GL_LINES, cubeIndices.size(), GL_UNSIGNED_INT, 0);
    //glBindVertexArray(0);


    m_shaderProgramPoint.bind();
    for (const CubeData& cube : m_cubes)
    {
        cubeVertices = cube.vertices;
        // Bind VAO
        glBindVertexArray(m_VAO_Cube);

        // Bind and set vertex data
        glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Cube);
        glBufferData(GL_ARRAY_BUFFER, cubeVertices.size() * sizeof(GLfloat), cubeVertices.constData(), GL_STATIC_DRAW);

        // Bind and set indices
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_EBO_Cube);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, cubeIndices.size() * sizeof(GLuint), cubeIndices.constData(), GL_STATIC_DRAW);

        // Vertex attributes
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        //glBindVertexArray(cube.VAO);
        //glLineWidth(2.0f);

        glDrawElements(GL_LINES, cube.indices.size(), GL_UNSIGNED_INT, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    for (const CubeData& cube : m_cubes)
    {
        drawText();
    }
}

void PointCloudOpenGLWidget::drawText()
{
    //const char* str = "ABCDEF";
    //glPushMatrix();
    //glTranslatef(0.0f, 0.0f, 0.0f); // 调整文本位置，使其出现在正方体的一个面上
    //glScalef(0.0001f, 0.0001f, 0.0001f); // 缩放文本
    //for (const char* p = str; *p; p++)
    //{
    //    glutStrokeCharacter(GLUT_STROKE_ROMAN, *p);
    //}
    //glPopMatrix();
}

void PointCloudOpenGLWidget::drawString(char *string)
    {
        GLuint base;
        glPushAttrib(GL_LIST_BIT);
        base = glGenLists(0x80);
        glListBase(base);
        glCallLists(strlen(string), GL_UNSIGNED_BYTE, (GLubyte *)string);
        glPopAttrib();

    }


/*
GLuint PointCloudOpenGLWidget::createCharTexture(FT_Face face, char c)
{
    //if (FT_Load_Char(face, c, FT_LOAD_RENDER)) {
        // Handle error
    }

    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RED,
        face->glyph->bitmap.width,
        face->glyph->bitmap.rows,
        0,
        GL_RED,
        GL_UNSIGNED_BYTE,
        face->glyph->bitmap.buffer
        );
    // Set texture options
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    return texture;
}
*/
void PointCloudOpenGLWidget::resizeGL(int width, int height)
{
    glViewport(0, 0, width, height);
}

void PointCloudOpenGLWidget::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}

void PointCloudOpenGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->pos().x() - lastPos.x();
    int dy = event->pos().y() - lastPos.y();
    if (event->buttons() & Qt::LeftButton)
    {
        m_xRotate = m_xRotate + 0.3 * dy;
        m_zRotate = m_zRotate + 0.3 * dx;

        if (m_xRotate > 30.0f)
        {
            m_xRotate = 30.0f;
        }
        if (m_xRotate < -120.0f)
        {
            m_xRotate = -120.0f;
        }
        update();
    }
    else if (event->buttons() & Qt::MiddleButton)
    {
        m_xTrans = m_xTrans + 0.1 * dx;
        m_yTrans = m_yTrans - 0.1 * dy;
        update();
    }
    lastPos = event->pos();
}

void PointCloudOpenGLWidget::wheelEvent(QWheelEvent *event)
{
    auto scroll_offest = event->angleDelta().y() / 120;
    m_zoom = m_zoom - (float)scroll_offest;

    if (m_zoom < 1.0f)    /* 放大限制 */
    {
        m_zoom = 1.0f;
    }

    if (m_zoom > 80.0f)
    {
        m_zoom = 80.0f;
    }

    update();
}

unsigned int PointCloudOpenGLWidget::drawMeshline(float size, int count)
{
    std::vector<float> mesh_vertexs;
    unsigned int vertex_count = 0;

    float start = count * (size / 2);
    float posX = start, posZ = start;

    for (int i = 0; i <= count; ++i)
    {
        mesh_vertexs.push_back(posX);
        mesh_vertexs.push_back(start);
        mesh_vertexs.push_back(0);

        mesh_vertexs.push_back(posX);
        mesh_vertexs.push_back(-start);
        mesh_vertexs.push_back(0);

        mesh_vertexs.push_back(start);
        mesh_vertexs.push_back(posZ);
        mesh_vertexs.push_back(0);

        mesh_vertexs.push_back(-start);
        mesh_vertexs.push_back(posZ);
        mesh_vertexs.push_back(0);

        posX = posX - size;
        posZ = posZ - size;
    }

    glGenVertexArrays(1, &m_VAO_MeshLine);
    glGenBuffers(1, &m_VBO_MeshLine);

    glBindVertexArray(m_VAO_MeshLine);

    glBindBuffer(GL_ARRAY_BUFFER, m_VBO_MeshLine);

    glBufferData(GL_ARRAY_BUFFER, mesh_vertexs.size() * sizeof(float), &mesh_vertexs[0], GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindVertexArray(0);

    vertex_count = (int)mesh_vertexs.size() / 3;

    return vertex_count;
}

void PointCloudOpenGLWidget::drawCooraxis(float length)
{
    axis_vertexs =
        {
            //x,y ,z ,r, g, b
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            length, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, length, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            0.0, 0.0, length, 0.0, 0.0, 1.0,
        };

    glGenVertexArrays(1, &m_VAO_Axis);
    glGenBuffers(1, &m_VBO_Axis);

    glBindVertexArray(m_VAO_Axis);

    glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Axis);
    glBufferData(GL_ARRAY_BUFFER, axis_vertexs.size() * sizeof(float), &axis_vertexs[0], GL_STATIC_DRAW);

    // 位置属性
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    // 颜色属性
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindVertexArray(0);
}

unsigned int PointCloudOpenGLWidget::drawPointdata(std::vector<float> &pointVertexs)
{
    unsigned int point_count = 0;



    glBindVertexArray(m_VAO_Point);

    glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Point);
    glBufferData(GL_ARRAY_BUFFER, pointVertexs.size() * sizeof(float), &pointVertexs[0], GL_STATIC_DRAW);

    // 位置属性
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    // 颜色属性
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindVertexArray(0);

    point_count = (unsigned int)pointVertexs.size() / 4;

    return point_count;
}

void PointCloudOpenGLWidget::paintLine(QVector<GLfloat> cubeVertices,QVector<GLuint> cubeIndices)
{
    //tempCubeVertices = cubeVertices;
    //tempCubeIndices =cubeIndices;

}

void PointCloudOpenGLWidget::addCube(const QVector3D& position, const QVector3D& scale, const QVector3D& rotation, const QVector3D& forward)
{
    CubeData newCube;

    newCube.position = position;
    newCube.scale = scale;
    newCube.rotation.setX(rotation.x());
    newCube.rotation.setY(rotation.y());
    newCube.rotation.setZ(rotation.z());

    newCube.vertices = computeCubeVerticesOpenGL(position, scale, rotation, forward);
    //newCube.vertices = computeCubeVerticesOpenGL_mid70(position, scale, rotation, forward);
    //newCube.indices = {0, 1, 2, 3, 4, 5, 6, 7, 0, 2, 1, 3, 4, 6, 5, 7}; // Assuming a line loop for the cube
    newCube.indices = {0, 1, 1, 2, 2, 3, 3, 0,    // bottom face
                       4, 5, 5, 6, 6, 7, 7, 4,    // top face
                       0, 4, 1, 5, 2, 6, 3, 7};

    glGenVertexArrays(1, &newCube.VAO);
    glGenBuffers(1, &newCube.VBO);
    glGenBuffers(1, &newCube.EBO);

    // Bind VAO
    //glBindVertexArray(newCube.VAO);

    // Bind and set vertex data
    //glBindBuffer(GL_ARRAY_BUFFER, newCube.VBO);
    //glBufferData(GL_ARRAY_BUFFER, newCube.vertices.size() * sizeof(GLfloat), newCube.vertices.constData(), GL_STATIC_DRAW);

    // Bind and set indices
    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, newCube.EBO);
    //glBufferData(GL_ELEMENT_ARRAY_BUFFER, newCube.indices.size() * sizeof(GLuint), newCube.indices.constData(), GL_STATIC_DRAW);

    // Vertex attributes
    //glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    //glEnableVertexAttribArray(0);

    // Unbind VAO and VBO
    //glBindBuffer(GL_ARRAY_BUFFER, 0);
    //glBindVertexArray(0);

    // Store the new cube in the cubes list
    m_cubes.append(newCube);
}

QVector<GLfloat> PointCloudOpenGLWidget::computeCubeVerticesOpenGL_mid70(const QVector3D& centerUnity, const QVector3D& sizeUnity, const QVector3D& rotation, const QVector3D& forward)
{

    QVector<GLfloat> vertices;

    // Convert Unity center to OpenGL by swapping Y and Z axes
    //QVector3D origin(209.1321,0,-260.204);

    QVector3D origin(223.83,0,-275.8);
    QVector3D center(abs(centerUnity.z()-origin.z()), abs(centerUnity.x()-origin.x()), centerUnity.y()-origin.y());
    if(centerUnity.x()>origin.x() && centerUnity.z()<origin.z())
    {
        //4
        //center.setX(-center.x());
        center.setY(-center.y());
    }
    else if(centerUnity.x()<origin.x() && centerUnity.z()>origin.z())
    {
        //2
        center.setX(-center.x());
    }
    else if(centerUnity.x()>origin.x() && centerUnity.z()>origin.z())
    {
        //3
        center.setX(-center.x());
        center.setY(-center.y());
    }


    QVector3D size(sizeUnity.x(), sizeUnity.z(), sizeUnity.y());



    // Calculate half sizes for convenience
    QVector3D half_sizes = size * 0.5f;

    // Define the 8 vertices of the cube in local coordinates
    QVector3D vertexPositions[8] = {
        QVector3D(-half_sizes.x(), -half_sizes.y(), -half_sizes.z()),  // Vertex 0
        QVector3D(half_sizes.x(), -half_sizes.y(), -half_sizes.z()),   // Vertex 1
        QVector3D(half_sizes.x(), half_sizes.y(), -half_sizes.z()),    // Vertex 2
        QVector3D(-half_sizes.x(), half_sizes.y(), -half_sizes.z()),   // Vertex 3
        QVector3D(-half_sizes.x(), -half_sizes.y(), half_sizes.z()),   // Vertex 4
        QVector3D(half_sizes.x(), -half_sizes.y(), half_sizes.z()),    // Vertex 5
        QVector3D(half_sizes.x(), half_sizes.y(), half_sizes.z()),     // Vertex 6
        QVector3D(-half_sizes.x(), half_sizes.y(), half_sizes.z())     // Vertex 7
    };

    // Apply the rotation and translation using the transformation matrix
    QMatrix4x4 transformationMatrix;
    transformationMatrix.setToIdentity();
    transformationMatrix.translate(center);

    transformationMatrix.rotate(rotation.y()-90, QVector3D(0, 0, 1));

    //transformationMatrix.rotate(rotation.scalar()+90.f,rotation.z(), rotation.x(),rotation.y() );
/*
    if( rotation.y() <360.0 && rotation.y()>270.0)
    {
        transformationMatrix.rotate(270.0-rotation.y(), QVector3D(0, 0, 1)); // Z轴旋转 rotation.setY(rotation.y()-270.0)
    }
    else if( rotation.y() <270.0 && rotation.y()>180.0)
    {
        transformationMatrix.rotate(270.0-rotation.y(), QVector3D(0, 0, 1)); // Z轴旋转 rotation.setY(rotation.y()-270.0)
    }
    else if( rotation.y() <180.0 && rotation.y()>90.0)
    {
        transformationMatrix.rotate(90.0-rotation.y(), QVector3D(0, 0, 1)); // Z轴旋转
    }
    else
    {
        transformationMatrix.rotate(rotation.y()+90.0, QVector3D(0, 0, 1)); // Z轴旋转
    }
*/

    transformationMatrix.rotate(rotation.x(), QVector3D(0, 1, 0)); // Y轴旋转

    transformationMatrix.rotate(rotation.z(), QVector3D(1, 0, 0)); // X轴旋转

    //GLuint texture = createCharTexture(face, 'A');
    //glBindTexture(GL_TEXTURE_2D, texture);

    //qDebug()<<rotation.x()<<" "<<rotation.y()<<" "<<rotation.z()<<" "<<rotation.scalar();
    for (int i = 0; i < 8; ++i) {
        QVector3D transformedVertex = transformationMatrix * vertexPositions[i];
        vertices << transformedVertex.x() << transformedVertex.y() << transformedVertex.z();
    }

    return vertices;
}


QVector<GLfloat> PointCloudOpenGLWidget::computeCubeVerticesOpenGL(const QVector3D& centerUnity, const QVector3D& sizeUnity, const QVector3D& rotation, const QVector3D& forward)
{
    QVector<GLfloat> vertices;

    // Convert Unity center to OpenGL by swapping Y and Z axes
    QVector3D origin(209.1321,2,-260.204);
    QVector3D center(abs(centerUnity.z()-origin.z()), abs(centerUnity.x()-origin.x()), centerUnity.y()-origin.y());

    if(centerUnity.x()<origin.x() && centerUnity.z()>origin.z())
    {
        center.setX(-center.x());
        center.setY(-center.y());
    }
    else if(centerUnity.x()<origin.x() && centerUnity.z()<origin.z())
    {
        //center.setX(-center.x());
        center.setY(-center.y());
    }
    else if(centerUnity.x()>origin.x() && centerUnity.z()>origin.z())
    {
        center.setX(-center.x());
    }

    QVector3D size(sizeUnity.x(), sizeUnity.z(), sizeUnity.y());



    // Calculate half sizes for convenience
    QVector3D half_sizes = size * 0.5f;

    // Define the 8 vertices of the cube in local coordinates
    QVector3D vertexPositions[8] = {
        QVector3D(-half_sizes.x(), -half_sizes.y(), -half_sizes.z()),  // Vertex 0
        QVector3D(half_sizes.x(), -half_sizes.y(), -half_sizes.z()),   // Vertex 1
        QVector3D(half_sizes.x(), half_sizes.y(), -half_sizes.z()),    // Vertex 2
        QVector3D(-half_sizes.x(), half_sizes.y(), -half_sizes.z()),   // Vertex 3
        QVector3D(-half_sizes.x(), -half_sizes.y(), half_sizes.z()),   // Vertex 4
        QVector3D(half_sizes.x(), -half_sizes.y(), half_sizes.z()),    // Vertex 5
        QVector3D(half_sizes.x(), half_sizes.y(), half_sizes.z()),     // Vertex 6
        QVector3D(-half_sizes.x(), half_sizes.y(), half_sizes.z())     // Vertex 7
    };

    // Apply the rotation and translation using the transformation matrix
    QMatrix4x4 transformationMatrix;
    transformationMatrix.setToIdentity();
    transformationMatrix.translate(center);

    //transformationMatrix.rotate(rotation.scalar()+90.f,rotation.z(), rotation.x(),rotation.y() );
    if( rotation.y() <360.0 && rotation.y()>270.0)
    {
        transformationMatrix.rotate(270.0-rotation.y(), QVector3D(0, 0, 1)); // Z轴旋转 rotation.setY(rotation.y()-270.0)
    }
    else if( rotation.y() <270.0 && rotation.y()>180.0)
    {
        transformationMatrix.rotate(270.0-rotation.y(), QVector3D(0, 0, 1)); // Z轴旋转 rotation.setY(rotation.y()-270.0)
    }
    else if( rotation.y() <180.0 && rotation.y()>90.0)
    {
        transformationMatrix.rotate(90.0-rotation.y(), QVector3D(0, 0, 1)); // Z轴旋转
    }
    else
    {
        transformationMatrix.rotate(rotation.y()+90.0, QVector3D(0, 0, 1)); // Z轴旋转
    }

    transformationMatrix.rotate(rotation.x(), QVector3D(0, 1, 0)); // Y轴旋转

    transformationMatrix.rotate(rotation.z(), QVector3D(1, 0, 0)); // X轴旋转

    //GLuint texture = createCharTexture(face, 'A');
    //glBindTexture(GL_TEXTURE_2D, texture);

    //qDebug()<<rotation.x()<<" "<<rotation.y()<<" "<<rotation.z()<<" "<<rotation.scalar();
    for (int i = 0; i < 8; ++i) {
        QVector3D transformedVertex = transformationMatrix * vertexPositions[i];
        vertices << transformedVertex.x() << transformedVertex.y() << transformedVertex.z();
    }

    return vertices;
}


void PointCloudOpenGLWidget::clearCube()
{
    m_cubes.clear();
}
