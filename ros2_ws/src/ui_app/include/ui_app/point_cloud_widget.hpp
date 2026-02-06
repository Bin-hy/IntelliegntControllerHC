#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <mutex>

struct PointVertex {
    float x, y, z;
    float r, g, b;
};

class PointCloudWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit PointCloudWidget(QWidget *parent = nullptr);
    ~PointCloudWidget();

    void updatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    void processPointCloud();

    // OpenGL resources
    QOpenGLShaderProgram *m_program;
    QOpenGLBuffer m_vbo;
    QOpenGLVertexArrayObject m_vao;
    int m_vertexCount;

    // Camera
    QMatrix4x4 m_projection;
    QMatrix4x4 m_view;
    QMatrix4x4 m_model;
    
    float m_dist;
    float m_rotX;
    float m_rotY;
    QPoint m_lastPos;

    // Data
    std::mutex m_mutex;
    sensor_msgs::msg::PointCloud2::SharedPtr m_pendingMsg;
    std::vector<PointVertex> m_points;
    bool m_newCloudAvailable;
};
