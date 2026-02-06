#include "ui_app/point_cloud_widget.hpp"
#include <QDebug>
#include <cmath>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <QPainter>

// Vertex Shader
static const char *vertexShaderSource =
    "#version 130\n"
    "in vec3 posAttr;\n"
    "in vec3 colAttr;\n"
    "out vec3 col;\n"
    "uniform mat4 mvp;\n"
    "void main() {\n"
    "   col = colAttr;\n"
    "   gl_Position = mvp * vec4(posAttr, 1.0);\n"
    "   gl_PointSize = 2.0;\n"
    "}\n";

// Fragment Shader
static const char *fragmentShaderSource =
    "#version 130\n"
    "in vec3 col;\n"
    "void main() {\n"
    "   gl_FragColor = vec4(col, 1.0);\n"
    "}\n";

PointCloudWidget::PointCloudWidget(QWidget *parent)
    : QOpenGLWidget(parent),
      m_program(nullptr),
      m_vertexCount(0),
      m_dist(-3.0f),
      m_rotX(0.0f),
      m_rotY(0.0f),
      m_newCloudAvailable(false)
{
    m_view.setToIdentity();
    m_model.setToIdentity();
    setFocusPolicy(Qt::StrongFocus);
}

PointCloudWidget::~PointCloudWidget()
{
    makeCurrent();
    m_vbo.destroy();
    m_vao.destroy();
    delete m_program;
    doneCurrent();
}

void PointCloudWidget::updatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_pendingMsg = msg;
    m_newCloudAvailable = true;
    // Request update on UI thread
    QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
}

void PointCloudWidget::processPointCloud()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_newCloudAvailable || !m_pendingMsg) return;

    m_points.clear();
    m_points.reserve(m_pendingMsg->width * m_pendingMsg->height);

    // Use manual parsing to avoid dependency on pcl_conversions if possible, 
    // or use sensor_msgs::PointCloud2ConstIterator if available (it is in sensor_msgs).
    
    // We'll iterate manually for maximum compatibility if iterators are tricky to link,
    // but standard iterators are header-only usually. Let's try manual.
    
    // Find offsets
    int x_off = -1, y_off = -1, z_off = -1, rgb_off = -1;
    for (const auto& field : m_pendingMsg->fields) {
        if (field.name == "x") x_off = field.offset;
        else if (field.name == "y") y_off = field.offset;
        else if (field.name == "z") z_off = field.offset;
        else if (field.name == "rgb" || field.name == "packed_rgb") rgb_off = field.offset;
    }

    if (x_off == -1 || y_off == -1 || z_off == -1) {
        m_newCloudAvailable = false;
        return;
    }

    const uint8_t* ptr = m_pendingMsg->data.data();
    int point_step = m_pendingMsg->point_step;
    size_t num_points = m_pendingMsg->width * m_pendingMsg->height;

    for (size_t i = 0; i < num_points; ++i) {
        float x, y, z;
        memcpy(&x, ptr + i * point_step + x_off, sizeof(float));
        memcpy(&y, ptr + i * point_step + y_off, sizeof(float));
        memcpy(&z, ptr + i * point_step + z_off, sizeof(float));

        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;

        PointVertex v;
        v.x = x; v.y = y; v.z = z;
        
        if (rgb_off != -1) {
            // RGB is usually float or uint32
            // Packed RGB is often represented as float in ROS messages (legacy)
            // or uint32. 
            // We assume it's 4 bytes.
            uint32_t rgb_val;
            memcpy(&rgb_val, ptr + i * point_step + rgb_off, sizeof(uint32_t));
            // BGRA or RGB? Usually ROS is RGB packed in int
            // layout: [r][g][b][a] or similar.
            // PCL default: 0x00RRGGBB
            uint8_t r = (rgb_val >> 16) & 0xFF;
            uint8_t g = (rgb_val >> 8) & 0xFF;
            uint8_t b = (rgb_val) & 0xFF;
            
            v.r = r / 255.0f;
            v.g = g / 255.0f;
            v.b = b / 255.0f;
        } else {
            // Pseudo color by Z
            v.r = 0.0f; v.g = 1.0f; v.b = 1.0f;
        }
        m_points.push_back(v);
    }
    
    m_vertexCount = m_points.size();
    m_newCloudAvailable = false;
}

void PointCloudWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.05f, 0.05f, 0.05f, 1.0f); // Dark background

    m_program = new QOpenGLShaderProgram(this);
    m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    m_program->link();

    m_vao.create();
    m_vao.bind();

    m_vbo.create();
    m_vbo.bind();
    
    // Allocate some initial space (e.g. 1MB)
    m_vbo.allocate(1024 * 1024 * sizeof(PointVertex));

    // Setup attributes
    m_program->bind();
    m_program->enableAttributeArray("posAttr");
    m_program->setAttributeBuffer("posAttr", GL_FLOAT, offsetof(PointVertex, x), 3, sizeof(PointVertex));
    
    m_program->enableAttributeArray("colAttr");
    m_program->setAttributeBuffer("colAttr", GL_FLOAT, offsetof(PointVertex, r), 3, sizeof(PointVertex));
    
    m_vao.release();
    m_vbo.release();
    m_program->release();
}

void PointCloudWidget::resizeGL(int w, int h)
{
    m_projection.setToIdentity();
    m_projection.perspective(45.0f, GLfloat(w) / h, 0.01f, 100.0f);
}

void PointCloudWidget::paintGL()
{
    // Check for new data and upload
    {
        // Ideally process on separate thread, but here we do it in paintGL or before
        // Since updatePointCloud calls 'update', paintGL is triggered.
        // We can do the heavy processing here or check if already done.
        // For simplicity, process here if needed (could block UI for large clouds)
        if (m_newCloudAvailable) {
            processPointCloud();
            
            if (m_vertexCount > 0) {
                m_vbo.bind();
                if (m_vbo.size() < (int)(m_points.size() * sizeof(PointVertex))) {
                     m_vbo.allocate(m_points.data(), m_points.size() * sizeof(PointVertex));
                } else {
                     m_vbo.write(0, m_points.data(), m_points.size() * sizeof(PointVertex));
                }
                m_vbo.release();
            }
        }
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    m_program->bind();
    m_vao.bind();

    QMatrix4x4 matrix;
    matrix.translate(0, 0, m_dist);
    matrix.rotate(m_rotX, 1, 0, 0);
    matrix.rotate(m_rotY, 0, 1, 0);
    
    // Correct for camera coordinate system (Z forward, Y down vs OpenGL Z backward, Y up)
    // ROS optical frame: X Right, Y Down, Z Forward
    // OpenGL: X Right, Y Up, Z Backward
    // Rotation -90 around X converts Y Down to Z Backward?
    // Actually, usually we rotate -90 X and then maybe 90 Z.
    // Let's just do a basic reorientation so it looks upright.
    matrix.rotate(-90, 1, 0, 0); // Z up
    matrix.rotate(90, 0, 0, 1);  // X forward

    m_program->setUniformValue("mvp", m_projection * matrix);

    glDrawArrays(GL_POINTS, 0, m_vertexCount);

    // Draw Grid / Axis
    // We'll use immediate mode for simple axis since it's just debug lines
    // Or just overlay text using QPainter
    
    m_vao.release();
    m_program->release();

    QPainter painter(this);
    painter.setPen(Qt::white);
    painter.drawText(10, 20, QString("Points: %1").arg(m_vertexCount));
    painter.drawText(10, 40, QString("Dist: %1").arg(m_dist));
    painter.end();
}

void PointCloudWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastPos = event->pos();
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - m_lastPos.x();
    int dy = event->y() - m_lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        m_rotX += dy;
        m_rotY += dx;
        update();
    }
    m_lastPos = event->pos();
}

void PointCloudWidget::wheelEvent(QWheelEvent *event)
{
    if(event->angleDelta().y() > 0) m_dist += 0.2f;
    else m_dist -= 0.2f;
    update();
}
