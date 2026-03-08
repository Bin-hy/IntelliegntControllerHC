#ifndef ROBOT_VIZ_WIDGET_HPP
#define ROBOT_VIZ_WIDGET_HPP

#include <QWidget>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DCore/QEntity>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QCameraLens>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DRender/QGeometryRenderer>
#include <Qt3DRender/QBuffer>
#include <Qt3DRender/QAttribute>
#include <Qt3DRender/QGeometry>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DRender/QMesh>
#include <Qt3DRender/QPointLight>
#include <Qt3DExtras/QForwardRenderer>
#include <memory>
#include <map>
#include <string>

// Forward declaration for urdf
namespace urdf { class Link; }

class RosNode;

class RobotVizWidget : public QWidget {
    Q_OBJECT
public:
    explicit RobotVizWidget(std::shared_ptr<RosNode> node, QWidget *parent = nullptr);
    ~RobotVizWidget();

    // Load URDF and create entities
    // tf_root: if non-empty, use this frame as the TF root for all links
    //          (used to attach hand models to the arm's base frame)
    void loadRobotModel(const std::string& urdf_path, const std::string& tf_root = "");

    // Get the root link name of the first loaded model
    std::string getRootLinkName() const { return root_link_name_; }

    // Set camera position
    void setCameraPosition(const QVector3D& pos, const QVector3D& view_center);

protected:
    void timerEvent(QTimerEvent *event) override;

private:
    void setupScene();
    void updateTransforms();
    Qt3DCore::QEntity* createLinkNode(const std::string& name, std::shared_ptr<const urdf::Link> link, const QMatrix4x4& initial_transform, const std::string& root_name);
    void processLinkRecursive(std::shared_ptr<const urdf::Link> link, const QMatrix4x4& parent_transform, const std::string& root_name);

    // Custom STL Loader helper
    Qt3DRender::QGeometryRenderer* loadSTLGeometry(const QString& path, Qt3DCore::QEntity* parent);

    std::shared_ptr<RosNode> node_;
    QWidget *container_;
    Qt3DExtras::Qt3DWindow *view_;
    Qt3DCore::QEntity *root_entity_;
    Qt3DExtras::QOrbitCameraController *cam_controller_;
    
    struct LinkInfo {
        Qt3DCore::QEntity* entity;
        Qt3DCore::QTransform* transform;
        std::string name;
        std::string root_name;
    };
    
    std::map<std::string, LinkInfo> links_;
    int timer_id_;
    
    // Base path for resolving package://
    std::string package_path_;
    
    // Root link name for TF lookup
    std::string root_link_name_ = "base_link";
};

#endif // ROBOT_VIZ_WIDGET_HPP
