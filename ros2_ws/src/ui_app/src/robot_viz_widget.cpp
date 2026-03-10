#include "ui_app/robot_viz_widget.hpp"
#include "ui_app/ros_node.hpp"
#include <QFile>
#include <QByteArray>
#include <QVBoxLayout>
#include <QColor>
#include <QMessageBox>
#include <urdf/model.h>
#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>
#include <limits>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>

std::string resolvePath(const std::string& path) {
    std::string prefix = "package://";
    if (path.find(prefix) == 0) {
        std::string rest = path.substr(prefix.length());
        size_t slash_pos = rest.find('/');
        if (slash_pos != std::string::npos) {
            std::string package_name = rest.substr(0, slash_pos);
            std::string file_path = rest.substr(slash_pos);
            try {
                std::string share_dir = ament_index_cpp::get_package_share_directory(package_name);
                return share_dir + file_path;
            } catch (...) {
                std::cerr << "Could not resolve package: " << package_name << std::endl;
            }
        }
    }
    return path;
}

RobotVizWidget::RobotVizWidget(std::shared_ptr<RosNode> node, QWidget *parent)
    : QWidget(parent), node_(node)
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0,0,0,0);

    view_ = new Qt3DExtras::Qt3DWindow();
    view_->defaultFrameGraph()->setClearColor(QColor(40, 40, 40));

    container_ = QWidget::createWindowContainer(view_);
    container_->setMouseTracking(true);
    container_->installEventFilter(this);
    layout->addWidget(container_);

    setupScene();

    // Start update timer (30 Hz)
    timer_id_ = startTimer(33);
}

RobotVizWidget::~RobotVizWidget() {
    if (timer_id_ != 0) killTimer(timer_id_);
    // view_ is owned by container/Qt
}

void RobotVizWidget::setRenderingEnabled(bool enabled) {
    if (rendering_enabled_ == enabled) return;
    rendering_enabled_ = enabled;

    if (enabled) {
        container_->show();
        if (timer_id_ == 0) {
            timer_id_ = startTimer(33);
        }
    } else {
        if (timer_id_ != 0) {
            killTimer(timer_id_);
            timer_id_ = 0;
        }
        container_->hide();
    }
}

void RobotVizWidget::setupScene() {
    root_entity_ = new Qt3DCore::QEntity();

    // Camera
    Qt3DRender::QCamera *camera = view_->camera();
    camera->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    camera->setPosition(QVector3D(1.6f, 1.0f, 1.2f));
    camera->setViewCenter(QVector3D(0.0f, 0.4f, 0.0f));

    // No QOrbitCameraController — we use custom mouse-based orbit control

    // Light
    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(root_entity_);
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor("white");
    light->setIntensity(2.0f);
    lightEntity->addComponent(light);
    Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);
    lightTransform->setTranslation(QVector3D(5.0f, 5.0f, 10.0f));
    lightEntity->addComponent(lightTransform);

    // Additional Fill Light
    Qt3DCore::QEntity *lightEntity2 = new Qt3DCore::QEntity(root_entity_);
    Qt3DRender::QPointLight *light2 = new Qt3DRender::QPointLight(lightEntity2);
    light2->setColor("white");
    light2->setIntensity(1.5f);
    lightEntity2->addComponent(light2);
    Qt3DCore::QTransform *lightTransform2 = new Qt3DCore::QTransform(lightEntity2);
    lightTransform2->setTranslation(QVector3D(-5.0f, -5.0f, 5.0f));
    lightEntity2->addComponent(lightTransform2);

    // Apply rotation to root entity to align ROS (Z-up) with Qt3D (Y-up)
    Qt3DCore::QTransform *rootTransform = new Qt3DCore::QTransform();
    rootTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), -90.0f));
    root_entity_->addComponent(rootTransform);

    view_->setRootEntity(root_entity_);
}

void RobotVizWidget::loadRobotModel(const std::string& urdf_path, const std::string& tf_root) {
    urdf::Model model;
    if (!model.initFile(urdf_path)) {
        std::cerr << "Failed to parse URDF: " << urdf_path << std::endl;
        QMessageBox::critical(this, "Model Load Error",
            QString("Failed to load URDF file:\n%1\n\nPlease check if the file exists.").arg(QString::fromStdString(urdf_path)));
        return;
    }

    std::cout << "Loaded robot: " << model.getName() << std::endl;

    // Debug info collector
    QString debug_msg = QString("Attempting to load URDF: %1\n").arg(QString::fromStdString(urdf_path));
    debug_msg += QString("Robot Name: %1\n").arg(QString::fromStdString(model.getName()));
    int link_count = 0;
    int visual_count = 0;

    // Start recursive processing from root link
    std::shared_ptr<const urdf::Link> root = model.getRoot();
    if (root) {
        std::string effective_root = tf_root.empty() ? root->name : tf_root;
        if (tf_root.empty()) {
            root_link_name_ = root->name;
        }
        QMatrix4x4 identity;
        processLinkRecursive(root, identity, effective_root);

        // Count links and visuals for debugging
        std::vector<std::shared_ptr<const urdf::Link>> stack;
        stack.push_back(root);
        while(!stack.empty()){
            auto link = stack.back();
            stack.pop_back();
            link_count++;
            if(link->visual) visual_count++;
            for(auto c : link->child_links) stack.push_back(c);
        }
    }

    debug_msg += QString("Total Links: %1\n").arg(link_count);
    debug_msg += QString("Total Visuals: %1\n").arg(visual_count);

    // Sample first mesh path resolution
    if (visual_count > 0) {
        std::shared_ptr<const urdf::Link> link = root;
        while(link && (!link->visual || link->visual->geometry->type != urdf::Geometry::MESH)) {
            if (!link->child_links.empty()) link = link->child_links[0];
            else break;
        }
        if (link && link->visual && link->visual->geometry->type == urdf::Geometry::MESH) {
            auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
            std::string resolved = resolvePath(mesh->filename);
            debug_msg += QString("\nSample Mesh Path:\n%1").arg(QString::fromStdString(resolved));
        }
    }
}

void RobotVizWidget::setCameraPosition(const QVector3D& pos, const QVector3D& view_center) {
    if (!view_ || !view_->camera()) return;
    Qt3DRender::QCamera *camera = view_->camera();
    camera->setPosition(pos);
    camera->setViewCenter(view_center);
}

// --- Custom Orbit Camera Control ---

bool RobotVizWidget::eventFilter(QObject *obj, QEvent *event) {
    if (obj != container_) return QWidget::eventFilter(obj, event);

    switch (event->type()) {
    case QEvent::MouseButtonPress: {
        auto *me = static_cast<QMouseEvent*>(event);
        if (me->button() == Qt::LeftButton) {
            mouse_pressed_ = true;
            last_mouse_pos_ = me->pos();
            return true;
        } else if (me->button() == Qt::MiddleButton) {
            middle_pressed_ = true;
            last_mouse_pos_ = me->pos();
            return true;
        }
        break;
    }
    case QEvent::MouseButtonRelease: {
        auto *me = static_cast<QMouseEvent*>(event);
        if (me->button() == Qt::LeftButton) {
            mouse_pressed_ = false;
            return true;
        } else if (me->button() == Qt::MiddleButton) {
            middle_pressed_ = false;
            return true;
        }
        break;
    }
    case QEvent::MouseMove: {
        auto *me = static_cast<QMouseEvent*>(event);
        QPoint delta = me->pos() - last_mouse_pos_;
        last_mouse_pos_ = me->pos();

        if (mouse_pressed_ && (me->modifiers() & Qt::ShiftModifier)) {
            // Shift + Left drag = Pan
            panCamera(delta.x(), delta.y());
            return true;
        } else if (mouse_pressed_) {
            // Left drag = Orbit
            orbitCamera(delta.x(), delta.y());
            return true;
        } else if (middle_pressed_) {
            // Middle drag = Pan
            panCamera(delta.x(), delta.y());
            return true;
        }
        break;
    }
    case QEvent::Wheel: {
        auto *we = static_cast<QWheelEvent*>(event);
        float delta = we->angleDelta().y() / 120.0f;
        zoomCamera(delta);
        return true;
    }
    default:
        break;
    }
    return QWidget::eventFilter(obj, event);
}

void RobotVizWidget::orbitCamera(float deltaX, float deltaY) {
    Qt3DRender::QCamera *camera = view_->camera();
    if (!camera) return;

    QVector3D viewCenter = camera->viewCenter();
    QVector3D position = camera->position();

    // Compute the vector from viewCenter to camera position
    QVector3D offset = position - viewCenter;
    float radius = offset.length();
    if (radius < 0.001f) return;

    // Spherical coordinates
    // theta: azimuth angle around Y axis (horizontal rotation)
    // phi: elevation angle from XZ plane (vertical rotation)
    float theta = std::atan2(offset.x(), offset.z());
    float phi = std::asin(qBound(-1.0f, offset.y() / radius, 1.0f));

    // Apply deltas (invert X for natural feeling)
    theta -= deltaX * orbit_speed_ * 0.01f;
    phi += deltaY * orbit_speed_ * 0.01f;

    // Clamp phi to avoid flipping (stay away from poles)
    const float maxPhi = static_cast<float>(M_PI) / 2.0f - 0.05f;
    phi = qBound(-maxPhi, phi, maxPhi);

    // Convert back to Cartesian
    QVector3D newOffset;
    newOffset.setX(radius * std::cos(phi) * std::sin(theta));
    newOffset.setY(radius * std::sin(phi));
    newOffset.setZ(radius * std::cos(phi) * std::cos(theta));

    camera->setPosition(viewCenter + newOffset);
    camera->setUpVector(QVector3D(0, 1, 0));
    camera->setViewCenter(viewCenter);
}

void RobotVizWidget::panCamera(float deltaX, float deltaY) {
    Qt3DRender::QCamera *camera = view_->camera();
    if (!camera) return;

    QVector3D viewCenter = camera->viewCenter();
    QVector3D position = camera->position();
    float distance = (position - viewCenter).length();

    // Scale pan speed by distance so it feels consistent at any zoom level
    float scaledPan = pan_speed_ * distance;

    // Camera's local right and up vectors
    QVector3D forward = (viewCenter - position).normalized();
    QVector3D right = QVector3D::crossProduct(forward, camera->upVector()).normalized();
    QVector3D up = QVector3D::crossProduct(right, forward).normalized();

    QVector3D panOffset = -right * deltaX * scaledPan + up * deltaY * scaledPan;

    camera->setPosition(position + panOffset);
    camera->setViewCenter(viewCenter + panOffset);
}

void RobotVizWidget::zoomCamera(float delta) {
    Qt3DRender::QCamera *camera = view_->camera();
    if (!camera) return;

    QVector3D viewCenter = camera->viewCenter();
    QVector3D position = camera->position();
    QVector3D direction = viewCenter - position;
    float distance = direction.length();

    if (distance < 0.001f) return;

    // Zoom factor scales with distance for consistent feel
    float zoomAmount = distance * zoom_speed_ * delta;

    // Don't zoom past the center or too far out
    float newDistance = distance - zoomAmount;
    newDistance = qBound(0.05f, newDistance, 50.0f);

    QVector3D newPos = viewCenter - direction.normalized() * newDistance;
    camera->setPosition(newPos);
}

// --- End Custom Orbit Camera ---

void RobotVizWidget::processLinkRecursive(std::shared_ptr<const urdf::Link> link, const QMatrix4x4& parent_transform, const std::string& root_name) {
    if (!link) return;

    QMatrix4x4 current_transform = parent_transform;

    if (link->parent_joint) {
        auto origin = link->parent_joint->parent_to_joint_origin_transform;

        QMatrix4x4 joint_mat;
        QQuaternion rot(origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z);
        QVector3D trans(origin.position.x, origin.position.y, origin.position.z);

        joint_mat.rotate(rot);
        joint_mat.setColumn(3, QVector4D(trans, 1.0f));

        current_transform = parent_transform * joint_mat;
    }

    if (link->visual && link->visual->geometry && link->visual->geometry->type == urdf::Geometry::MESH) {
        auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
        if (mesh) {
            std::string mesh_path = resolvePath(mesh->filename);
            if (!mesh_path.empty()) {
                createLinkNode(link->name, link, current_transform, root_name);
            }
        }
    }

    for (auto child : link->child_links) {
        processLinkRecursive(child, current_transform, root_name);
    }
}

Qt3DCore::QEntity* RobotVizWidget::createLinkNode(const std::string& name, std::shared_ptr<const urdf::Link> link, const QMatrix4x4& initial_transform, const std::string& root_name) {
    Qt3DCore::QEntity *link_entity = new Qt3DCore::QEntity(root_entity_);

    Qt3DCore::QTransform *link_transform = new Qt3DCore::QTransform();
    link_transform->setMatrix(initial_transform);
    link_entity->addComponent(link_transform);

    if (link->visual && link->visual->geometry && link->visual->geometry->type == urdf::Geometry::MESH) {
        auto urdf_mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
        if (urdf_mesh) {
            std::string mesh_path = resolvePath(urdf_mesh->filename);
            if (!mesh_path.empty()) {
                Qt3DCore::QEntity *visual_entity = new Qt3DCore::QEntity(link_entity);

                Qt3DRender::QGeometryRenderer *mesh = nullptr;
                if (mesh_path.find(".stl") != std::string::npos || mesh_path.find(".STL") != std::string::npos) {
                    mesh = loadSTLGeometry(QString::fromStdString(mesh_path), visual_entity);
                }
                if (!mesh) {
                    Qt3DExtras::QCuboidMesh *box = new Qt3DExtras::QCuboidMesh();
                    box->setXExtent(0.05f);
                    box->setYExtent(0.05f);
                    box->setZExtent(0.05f);
                    visual_entity->addComponent(box);
                }

                if (mesh) {
                    visual_entity->addComponent(mesh);
                }

                Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial();
                material->setDiffuse(QColor(200, 200, 200));
                visual_entity->addComponent(material);

                Qt3DCore::QTransform *visual_transform = new Qt3DCore::QTransform();
                auto v_origin = link->visual->origin;

                QMatrix4x4 v_mat;
                v_mat.translate(v_origin.position.x, v_origin.position.y, v_origin.position.z);
                QQuaternion v_rot(v_origin.rotation.w, v_origin.rotation.x, v_origin.rotation.y, v_origin.rotation.z);
                v_mat.rotate(v_rot);

                v_mat.scale(urdf_mesh->scale.x, urdf_mesh->scale.y, urdf_mesh->scale.z);

                visual_transform->setMatrix(v_mat);
                visual_entity->addComponent(visual_transform);

                std::cout << "Created Node: " << name << " | Mesh: " << mesh_path << std::endl;
                std::cout << "  Init Pose: " << initial_transform.column(3).x() << ", "
                          << initial_transform.column(3).y() << ", " << initial_transform.column(3).z() << std::endl;
            }
        }
    }

    LinkInfo info;
    info.entity = link_entity;
    info.transform = link_transform;
    info.name = name;
    info.root_name = root_name;
    links_[name] = info;

    return link_entity;
}

Qt3DRender::QGeometryRenderer* RobotVizWidget::loadSTLGeometry(const QString& path, Qt3DCore::QEntity* parent) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        std::cerr << "Cannot open STL file: " << path.toStdString() << std::endl;
        return nullptr;
    }

    QByteArray data = file.readAll();
    const char* ptr = data.constData();

    if (data.size() < 84) return nullptr;

    quint32 triangleCount = 0;
    std::memcpy(&triangleCount, ptr + 80, sizeof(quint32));
    if (triangleCount == 0) return nullptr;
    quint64 expected_size = 84ull + static_cast<quint64>(triangleCount) * 50ull;
    if (expected_size != static_cast<quint64>(data.size())) {
        return nullptr;
    }
    if (triangleCount > 2000000) return nullptr;

    QByteArray bufferData;
    quint64 buffer_bytes = static_cast<quint64>(triangleCount) * 3ull * 2ull * 3ull * sizeof(float);
    if (buffer_bytes > static_cast<quint64>(std::numeric_limits<int>::max())) return nullptr;
    bufferData.resize(static_cast<int>(buffer_bytes));
    float* fptr = reinterpret_cast<float*>(bufferData.data());

    const char* dataPtr = ptr + 84;
    quint32 actualTriangles = 0;
    for (quint32 i = 0; i < triangleCount; ++i) {
        if (dataPtr + 50 > ptr + data.size()) break;

        float nx, ny, nz;
        float v1x, v1y, v1z;
        float v2x, v2y, v2z;
        float v3x, v3y, v3z;
        std::memcpy(&nx, dataPtr + 0, 4);
        std::memcpy(&ny, dataPtr + 4, 4);
        std::memcpy(&nz, dataPtr + 8, 4);
        std::memcpy(&v1x, dataPtr + 12, 4);
        std::memcpy(&v1y, dataPtr + 16, 4);
        std::memcpy(&v1z, dataPtr + 20, 4);
        std::memcpy(&v2x, dataPtr + 24, 4);
        std::memcpy(&v2y, dataPtr + 28, 4);
        std::memcpy(&v2z, dataPtr + 32, 4);
        std::memcpy(&v3x, dataPtr + 36, 4);
        std::memcpy(&v3y, dataPtr + 40, 4);
        std::memcpy(&v3z, dataPtr + 44, 4);

        *fptr++ = v1x; *fptr++ = v1y; *fptr++ = v1z;
        *fptr++ = nx;  *fptr++ = ny;  *fptr++ = nz;
        *fptr++ = v2x; *fptr++ = v2y; *fptr++ = v2z;
        *fptr++ = nx;  *fptr++ = ny;  *fptr++ = nz;
        *fptr++ = v3x; *fptr++ = v3y; *fptr++ = v3z;
        *fptr++ = nx;  *fptr++ = ny;  *fptr++ = nz;

        dataPtr += 50;
        actualTriangles++;
    }
    if (actualTriangles == 0) return nullptr;
    if (actualTriangles != triangleCount) {
        bufferData.resize(actualTriangles * 3 * 2 * 3 * sizeof(float));
    }

    Qt3DRender::QGeometry *geometry = new Qt3DRender::QGeometry(parent);

    Qt3DRender::QBuffer *vertexBuffer = new Qt3DRender::QBuffer(geometry);
    vertexBuffer->setData(bufferData);

    Qt3DRender::QAttribute *posAttr = new Qt3DRender::QAttribute();
    posAttr->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
    posAttr->setVertexBaseType(Qt3DRender::QAttribute::Float);
    posAttr->setVertexSize(3);
    posAttr->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    posAttr->setBuffer(vertexBuffer);
    posAttr->setByteStride(6 * sizeof(float));
    posAttr->setByteOffset(0);
    posAttr->setCount(actualTriangles * 3);
    geometry->addAttribute(posAttr);

    Qt3DRender::QAttribute *normAttr = new Qt3DRender::QAttribute();
    normAttr->setName(Qt3DRender::QAttribute::defaultNormalAttributeName());
    normAttr->setVertexBaseType(Qt3DRender::QAttribute::Float);
    normAttr->setVertexSize(3);
    normAttr->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    normAttr->setBuffer(vertexBuffer);
    normAttr->setByteStride(6 * sizeof(float));
    normAttr->setByteOffset(3 * sizeof(float));
    normAttr->setCount(actualTriangles * 3);
    geometry->addAttribute(normAttr);

    Qt3DRender::QGeometryRenderer *renderer = new Qt3DRender::QGeometryRenderer(parent);
    renderer->setGeometry(geometry);
    renderer->setPrimitiveType(Qt3DRender::QGeometryRenderer::Triangles);
    renderer->setVertexCount(actualTriangles * 3);

    return renderer;
}

void RobotVizWidget::timerEvent(QTimerEvent *event) {
    if (event->timerId() == timer_id_) {
        updateTransforms();
    }
}

void RobotVizWidget::updateTransforms() {
    auto tf_buffer = node_->get_tf_buffer();
    if (!tf_buffer) return;

    for (auto& [name, info] : links_) {
        try {
            geometry_msgs::msg::TransformStamped t;
            t = tf_buffer->lookupTransform(info.root_name, name, tf2::TimePointZero);

            QVector3D pos(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
            QQuaternion rot(t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z);

            info.transform->setTranslation(pos);
            info.transform->setRotation(rot);

        } catch (tf2::TransformException &ex) {
            // Transform not available yet
        }
    }
}
