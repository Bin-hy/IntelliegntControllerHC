#include "ui_app/app_window.hpp"
#include "ui_app/point_cloud_widget.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QGroupBox>
#include <QTabWidget>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QTimer>
#include <QScrollArea>
#include <QMessageBox>
#include <QTableWidget>
#include <QHeaderView>
#include <QLineEdit>
#include <QDateTimeEdit>
#include <QInputDialog>

AppWindow::AppWindow(std::shared_ptr<RosNode> node,
                     std::shared_ptr<AuthManager> auth_manager,
                     std::shared_ptr<AuthLogManager> log_manager,
                     PermissionManager* permission_manager,
                     UserRole current_role,
                     QWidget *parent) 
    : QWidget(parent),
      node_(std::move(node)),
      auth_manager_(std::move(auth_manager)),
      auth_log_manager_(std::move(log_manager)),
      permission_manager_(permission_manager),
      current_role_(current_role),
      tabs_(nullptr),
      admin_tab_(nullptr),
      admin_user_table_(nullptr),
      admin_log_table_(nullptr),
      admin_log_user_filter_(nullptr),
      admin_log_from_(nullptr),
      admin_log_to_(nullptr),
      admin_log_success_only_(nullptr),
      admin_log_failure_only_(nullptr)
{
    auto * main_layout = new QVBoxLayout();
    
    label_count_ = new QLabel("Heartbeat: 0");
    main_layout->addWidget(label_count_);

    tabs_ = new QTabWidget();
    tabs_->addTab(createControlTab(), "Power & Status");
    tabs_->addTab(createMoveTab(), "Motion Control");
    tabs_->addTab(createCameraTab(), "Vision System");
    
    robot_viz_ = new RobotVizWidget(node_);
    tabs_->addTab(robot_viz_, "3D Simulation");
    std::string urdf_path = node_->get_robot_urdf_path();
    robot_viz_->loadRobotModel(urdf_path);

    std::string lhand_path = node_->get_left_hand_urdf_path();
    bool fused_hand = urdf_path.find("with_dh116_dualhand") != std::string::npos
        || urdf_path.find("with_dh116_lhand") != std::string::npos;
    if (!fused_hand && !lhand_path.empty()) robot_viz_->loadRobotModel(lhand_path);

    std::string rhand_path = node_->get_right_hand_urdf_path();
    if (!fused_hand && !rhand_path.empty()) robot_viz_->loadRobotModel(rhand_path);

    tabs_->addTab(createLHandTab(), "LHand Control");
    tabs_->addTab(createTaskTab(), "Task Module");

    rebuildAdminTab();

    main_layout->addWidget(tabs_);
    
    setLayout(main_layout);
    
    applyPermissionToControls();

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this](){
      updateUI();
    });
    timer_->start(100);
}

void AppWindow::setCurrentRole(UserRole role) {
    current_role_ = role;
    rebuildAdminTab();
    applyPermissionToControls();
}

QWidget* AppWindow::createControlTab() {
      auto * widget = new QWidget();
      auto * layout = new QVBoxLayout();

      // Power Control
      auto * group_ctrl = new QGroupBox("Power Management");
      auto * layout_ctrl = new QHBoxLayout();
      btn_power_on_ = new QPushButton("Power ON");
      btn_enable_ = new QPushButton("Enable");
      btn_disable_ = new QPushButton("Disable");
      btn_power_off_ = new QPushButton("Power OFF");
      
      btn_power_on_->setObjectName("btn_power_on");
      btn_power_off_->setObjectName("btn_power_off");

      layout_ctrl->addWidget(btn_power_on_);
      layout_ctrl->addWidget(btn_enable_);
      layout_ctrl->addWidget(btn_disable_);
      layout_ctrl->addWidget(btn_power_off_);
      group_ctrl->setLayout(layout_ctrl);
      layout->addWidget(group_ctrl);

      // Status
      auto * group_status = new QGroupBox("Robot Status");
      auto * layout_status = new QVBoxLayout();
      text_robot_state_ = new QTextEdit();
      text_robot_state_->setReadOnly(true);
      layout_status->addWidget(text_robot_state_);
      group_status->setLayout(layout_status);
      layout->addWidget(group_status);

      widget->setLayout(layout);

      connect(btn_power_on_, &QPushButton::clicked, this, [this](){
          if (permission_manager_ &&
              !permission_manager_->hasPermission(current_role_, ActionType::PowerOn)) {
              QMessageBox::warning(this, "权限不足", "当前用户无权执行 Power ON 操作");
              return;
          }
          node_->call_robot_control("poweron");
      });

      connect(btn_enable_, &QPushButton::clicked, this, [this](){
          if (permission_manager_ &&
              !permission_manager_->hasPermission(current_role_, ActionType::ModifyParam)) {
              QMessageBox::warning(this, "权限不足", "当前用户无权执行 Enable 操作");
              return;
          }
          node_->call_robot_control("enable");
      });

      connect(btn_disable_, &QPushButton::clicked, this, [this](){
          if (permission_manager_ &&
              !permission_manager_->hasPermission(current_role_, ActionType::ModifyParam)) {
              QMessageBox::warning(this, "权限不足", "当前用户无权执行 Disable 操作");
              return;
          }
          node_->call_robot_control("disable");
      });

      connect(btn_power_off_, &QPushButton::clicked, this, [this](){
          if (permission_manager_ &&
              !permission_manager_->hasPermission(current_role_, ActionType::PowerOff)) {
              QMessageBox::warning(this, "权限不足", "当前用户无权执行 Power OFF 操作");
              return;
          }
          node_->call_robot_control("poweroff");
      });

      return widget;
}

void AppWindow::updateUI() {
      label_count_->setText("Heartbeat: " + QString::number(node_->count_.load()));
      
      {
          std::lock_guard<std::mutex> lock(node_->data_mutex_);
          if (!node_->last_robot_state_str_.empty()) {
              text_robot_state_->setText(QString::fromStdString(node_->last_robot_state_str_));
          }
      }

      {
          std::lock_guard<std::mutex> lock(node_->image_mutex_);
          if (node_->last_point_cloud_ && widget_point_cloud_->isVisible()) {
              widget_point_cloud_->updatePointCloud(node_->last_point_cloud_);
          }
          
          auto update_label = [](QLabel* label, const cv::Mat& mat, bool is_rgb) {
              if (!label || !label->isVisible()) return;
              if (mat.empty()) {
                  // label->setText("No Data"); // Keep last frame or "No Signal" text
                  return; 
              }
              QImage img;
              if (is_rgb) {
                  cv::Mat rgb;
                  cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
                  img = QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888).copy();
              } else {
                  // Grayscale (Depth / IR)
                  img = QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8).copy();
              }
              label->setPixmap(QPixmap::fromImage(img).scaled(label->size(), Qt::KeepAspectRatio));
          };

          update_label(label_color_stream_, node_->last_color_image_, true);
          update_label(label_depth_stream_, node_->last_depth_image_, false);
          update_label(label_ir_left_stream_, node_->last_ir_left_image_, false);
          update_label(label_ir_right_stream_, node_->last_ir_right_image_, false);
      }
}

QWidget* AppWindow::createMoveTab() {
      auto * widget = new QWidget();
      auto * layout = new QVBoxLayout();

      // --- Joint Move ---
      auto * group_joint = new QGroupBox("Joint Movement (MoveJ)");
      auto * layout_joint = new QGridLayout();
      
      for(int i=0; i<7; ++i) {
          layout_joint->addWidget(new QLabel(QString("J%1 (rad):").arg(i+1)), 0, i);
          spin_joints_[i] = new QDoubleSpinBox();
          spin_joints_[i]->setRange(-6.28, 6.28);
          spin_joints_[i]->setSingleStep(0.1);
          layout_joint->addWidget(spin_joints_[i], 1, i);
      }
      
      auto * btn_get_joints = new QPushButton("Get Current");
      auto * btn_movej = new QPushButton("Execute MoveJ");
      layout_joint->addWidget(btn_get_joints, 2, 0, 1, 2);
      layout_joint->addWidget(btn_movej, 2, 5, 1, 2);
      group_joint->setLayout(layout_joint);
      layout->addWidget(group_joint);

      // --- Cartesian Move ---
      auto * group_cart = new QGroupBox("Cartesian Movement (MoveL)");
      auto * layout_cart = new QGridLayout();
      QStringList labels = {"X", "Y", "Z", "RX", "RY", "RZ"};
      for(int i=0; i<6; ++i) {
          layout_cart->addWidget(new QLabel(labels[i] + " (m/rad):"), 0, i);
          spin_cart_[i] = new QDoubleSpinBox();
          spin_cart_[i]->setRange(-2.0, 2.0); // Meters
          spin_cart_[i]->setSingleStep(0.01);
          layout_cart->addWidget(spin_cart_[i], 1, i);
      }
      auto * btn_movel = new QPushButton("Execute MoveL");
      layout_cart->addWidget(btn_movel, 2, 5, 1, 1);
      group_cart->setLayout(layout_cart);
      layout->addWidget(group_cart);

      // --- Parameters ---
      auto * group_params = new QGroupBox("Motion Parameters");
      auto * layout_params = new QHBoxLayout();
      layout_params->addWidget(new QLabel("Vel (m/s or rad/s):"));
      spin_vel_ = new QDoubleSpinBox();
      spin_vel_->setValue(0.2);
      spin_vel_->setRange(0.01, 2.0);
      layout_params->addWidget(spin_vel_);

      layout_params->addWidget(new QLabel("Acc (m/s²):"));
      spin_acc_ = new QDoubleSpinBox();
      spin_acc_->setValue(0.5);
      spin_acc_->setRange(0.01, 5.0);
      layout_params->addWidget(spin_acc_);

      group_params->setLayout(layout_params);
      layout->addWidget(group_params);

      widget->setLayout(layout);

      // Connect Move Actions
      connect(btn_get_joints, &QPushButton::clicked, this, [this](){
          std::lock_guard<std::mutex> lock(node_->data_mutex_);
          const int n = static_cast<int>(node_->current_joints_.size());
          const int to_copy = std::min(n, 7);
          for(int i = 0; i < to_copy; ++i) {
              spin_joints_[i]->setValue(node_->current_joints_[i]);
          }
      });

      connect(btn_movej, &QPushButton::clicked, this, [this](){
          std::vector<float> q;
          for(int i=0; i<7; ++i) q.push_back((float)spin_joints_[i]->value());
          std::vector<float> p; // Empty for MoveJ
          // Use movej2 for rad/s units
          node_->call_robot_move("movej2", p, q, (float)spin_vel_->value(), (float)spin_acc_->value(), 0.0, "default", "default");
      });

      connect(btn_movel, &QPushButton::clicked, this, [this](){
          std::vector<float> p;
          for(int i=0; i<6; ++i) p.push_back((float)spin_cart_[i]->value());
          std::vector<float> q; // Empty for MoveL
          node_->call_robot_move("movel", p, q, (float)spin_vel_->value(), (float)spin_acc_->value(), 0.0, "default", "default");
      });

      return widget;
}

QWidget* AppWindow::createCameraTab() {
    auto * widget = new QWidget();
    auto * layout = new QVBoxLayout();
    
    // --- Configuration Area ---
    auto * group_config = new QGroupBox("Camera Selection");
    auto * layout_config = new QHBoxLayout();
    
    layout_config->addWidget(new QLabel("Camera Namespace:"));
    combo_camera_ = new QComboBox();
    combo_camera_->setEditable(true); // Allow typing custom ns
    combo_camera_->addItem("camera");
    combo_camera_->setMinimumWidth(150);
    layout_config->addWidget(combo_camera_);

    layout_config->addWidget(new QLabel("PC Topic:"));
    combo_pc_topic_ = new QComboBox();
    combo_pc_topic_->setEditable(true);
    combo_pc_topic_->setMinimumWidth(200);
    layout_config->addWidget(combo_pc_topic_);

    btn_scan_ = new QPushButton("Scan Cameras");
    layout_config->addWidget(btn_scan_);

    layout_config->addStretch();
    group_config->setLayout(layout_config);
    layout->addWidget(group_config);

    // --- Stream Selection ---
    auto * group_sensors = new QGroupBox("Active Sensors");
    auto * layout_sensors = new QHBoxLayout();
    check_color_ = new QCheckBox("Color Stream");
    check_depth_ = new QCheckBox("Depth Stream");
    check_point_cloud_ = new QCheckBox("Point Cloud");
    check_ir_left_ = new QCheckBox("IR Left");
    check_ir_right_ = new QCheckBox("IR Right");
    
    // Defaults: Color and Depth checked
    check_color_->setChecked(true);
    check_depth_->setChecked(true);
    check_point_cloud_->setChecked(false);
    check_ir_left_->setChecked(false);
    check_ir_right_->setChecked(false);

    layout_sensors->addWidget(check_color_);
    layout_sensors->addWidget(check_depth_);
    layout_sensors->addWidget(check_point_cloud_);
    layout_sensors->addWidget(check_ir_left_);
    layout_sensors->addWidget(check_ir_right_);
    layout_sensors->addStretch();
    group_sensors->setLayout(layout_sensors);
    layout->addWidget(group_sensors);

    // --- Video Grid ---
    // Use a ScrollArea in case screens are small
    auto * scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    container_video_ = new QWidget();
    auto * grid_video = new QGridLayout(container_video_);
    
    // Create Widgets (hidden by default if not checked)
    auto make_callback = [this]() {
        return [this](bool success, std::string msg) {
            QString qmsg = QString::fromStdString(msg);
            QMetaObject::invokeMethod(this, [this, success, qmsg](){
                if(success) {
                    QMessageBox::information(this, "Snapshot Saved", "Saved successfully to:\n" + qmsg);
                } else {
                    QMessageBox::warning(this, "Snapshot Failed", qmsg);
                }
            }, Qt::QueuedConnection);
        };
    };

    widget_color_ = createVideoWidget("Color Stream", label_color_stream_, [this, make_callback](){
        node_->save_snapshot(combo_camera_->currentText().toStdString(), true, false, false, false, false, make_callback());
    });
    widget_depth_ = createVideoWidget("Depth Stream", label_depth_stream_, [this, make_callback](){
        node_->save_snapshot(combo_camera_->currentText().toStdString(), false, true, false, false, false, make_callback());
    });
    widget_ir_left_ = createVideoWidget("IR Left Stream", label_ir_left_stream_, [this, make_callback](){
        node_->save_snapshot(combo_camera_->currentText().toStdString(), false, false, true, false, false, make_callback());
    });
    widget_ir_right_ = createVideoWidget("IR Right Stream", label_ir_right_stream_, [this, make_callback](){
        node_->save_snapshot(combo_camera_->currentText().toStdString(), false, false, false, true, false, make_callback());
    });

    widget_point_cloud_ = new PointCloudWidget();
    widget_point_cloud_->setMinimumSize(400, 300);

    // Add to grid (2x2)
    grid_video->addWidget(widget_color_, 0, 0);
    grid_video->addWidget(widget_depth_, 0, 1);
    grid_video->addWidget(widget_ir_left_, 1, 0);
    grid_video->addWidget(widget_ir_right_, 1, 1);
    grid_video->addWidget(widget_point_cloud_, 2, 0, 1, 2); // Span 2 columns
    
    scroll->setWidget(container_video_);
    layout->addWidget(scroll);

    // --- Connections ---
    connect(btn_scan_, &QPushButton::clicked, this, &AppWindow::refreshCameraList);
    
    auto update_config = [this]() { onCameraConfigChanged(); };
    connect(combo_camera_, &QComboBox::currentTextChanged, this, update_config);
    connect(combo_pc_topic_, &QComboBox::currentTextChanged, this, update_config);
    connect(check_color_, &QCheckBox::stateChanged, this, update_config);
    connect(check_depth_, &QCheckBox::stateChanged, this, update_config);
    connect(check_point_cloud_, &QCheckBox::stateChanged, this, update_config);
    connect(check_ir_left_, &QCheckBox::stateChanged, this, update_config);
    connect(check_ir_right_, &QCheckBox::stateChanged, this, update_config);

    // Initial sync
    onCameraConfigChanged();

    widget->setLayout(layout);
    return widget;
}

QWidget* AppWindow::createVideoWidget(const QString& title, QLabel*& label_ptr, std::function<void()> save_callback) {
    auto * group = new QGroupBox(title);
    auto * layout = new QVBoxLayout();
    
    label_ptr = new QLabel("No Signal");
    label_ptr->setMinimumSize(320, 240);
    label_ptr->setAlignment(Qt::AlignCenter);
    label_ptr->setObjectName("video_label");
    // label_ptr->setStyleSheet("border: 1px solid #555; background-color: #222; color: #aaa;");
    layout->addWidget(label_ptr);

    auto * btn = new QPushButton("Capture / Save");
    layout->addWidget(btn);
    
    // Connect save button
    QObject::connect(btn, &QPushButton::clicked, btn, [save_callback](){
        if(save_callback) save_callback();
    });

    group->setLayout(layout);
    return group;
}

void AppWindow::refreshCameraList() {
    auto cameras = node_->scan_cameras();
    combo_camera_->blockSignals(true); // Prevent triggering config change during update
    combo_camera_->clear();
    for(const auto& cam : cameras) {
        combo_camera_->addItem(QString::fromStdString(cam));
    }
    combo_camera_->blockSignals(false);

    auto pc_topics = node_->scan_point_clouds();
    combo_pc_topic_->blockSignals(true);
    combo_pc_topic_->clear();
    for(const auto& topic : pc_topics) {
        combo_pc_topic_->addItem(QString::fromStdString(topic));
    }
    
    combo_pc_topic_->blockSignals(false);

    // Trigger update if selection changed (or just force it)
    if (combo_camera_->count() > 0) {
        if(combo_camera_->currentIndex() == -1) combo_camera_->setCurrentIndex(0);
        onCameraConfigChanged(); 
    }
}

void AppWindow::onCameraConfigChanged() {
    std::string cam_ns = combo_camera_->currentText().toStdString();
    
    // Auto-detect capabilities and update UI state (Enable/Disable/Hide)
    auto caps = node_->get_camera_capabilities(cam_ns);

    // Update Checkbox visibility/state based on capabilities
    check_color_->setEnabled(caps.has_color);
    if (!caps.has_color && check_color_->isChecked()) check_color_->setChecked(false);

    check_depth_->setEnabled(caps.has_depth);
    if (!caps.has_depth && check_depth_->isChecked()) check_depth_->setChecked(false);

    // IR Logic
    check_ir_left_->setEnabled(caps.has_ir_left);
    if (caps.has_ir_left && !caps.has_ir_right) {
        // Mono IR case: Rename "IR Left" to "IR Stream" and hide "IR Right"
        check_ir_left_->setText("IR Stream");
        check_ir_right_->setVisible(false);
        check_ir_right_->setChecked(false);
    } else {
        // Dual IR case
        check_ir_left_->setText("IR Left");
        check_ir_right_->setVisible(true);
        check_ir_right_->setEnabled(caps.has_ir_right);
    }
    if (!caps.has_ir_left && check_ir_left_->isChecked()) check_ir_left_->setChecked(false);
    if (!caps.has_ir_right && check_ir_right_->isChecked()) check_ir_right_->setChecked(false);

    // Point Cloud: Enable if capability exists OR if user selected a custom topic in the dropdown
    std::string pc_topic = combo_pc_topic_->currentText().toStdString();
    bool has_custom_pc_topic = !pc_topic.empty();
    
    check_point_cloud_->setEnabled(caps.has_point_cloud || has_custom_pc_topic);
    // Don't auto-uncheck pointcloud as user might want to try force enabling or using custom topic

    bool c = check_color_->isChecked();
    bool d = check_depth_->isChecked();
    bool pc = check_point_cloud_->isChecked();
    bool ir_l = check_ir_left_->isChecked();
    bool ir_r = check_ir_right_->isChecked();

    // Update Node Subscriptions
    node_->update_camera_subscriptions(cam_ns, c, d, ir_l, ir_r, pc, pc_topic);

    // Update UI Visibility
    widget_color_->setVisible(c);
    widget_depth_->setVisible(d);
    widget_point_cloud_->setVisible(pc);
    widget_ir_left_->setVisible(ir_l);
    widget_ir_right_->setVisible(ir_r);
    
    // Update Widget Titles if Mono IR
    if (caps.has_ir_left && !caps.has_ir_right) {
        // widget_ir_left_ is a QGroupBox* because createVideoWidget returns a group box
        if (QGroupBox* gb = qobject_cast<QGroupBox*>(widget_ir_left_)) {
            gb->setTitle("IR Stream");
        }
    } else {
        if (QGroupBox* gb = qobject_cast<QGroupBox*>(widget_ir_left_)) {
            gb->setTitle("IR Left Stream");
        }
    }
}

void AppWindow::rebuildAdminTab() {
    if (!tabs_) {
        return;
    }
    bool can_admin = permission_manager_ &&
                     permission_manager_->hasPermission(current_role_, ActionType::CreateUser);
    int idx = admin_tab_ ? tabs_->indexOf(admin_tab_) : -1;
    if (can_admin) {
        if (!admin_tab_) {
            admin_tab_ = createAdminTab();
            tabs_->addTab(admin_tab_, "Admin Console");
        } else if (idx < 0) {
            tabs_->addTab(admin_tab_, "Admin Console");
        }
    } else {
        if (idx >= 0) {
            tabs_->removeTab(idx);
        }
        if (admin_tab_) {
            admin_tab_->deleteLater();
            admin_tab_ = nullptr;
            admin_user_table_ = nullptr;
            admin_log_table_ = nullptr;
            admin_log_user_filter_ = nullptr;
            admin_log_from_ = nullptr;
            admin_log_to_ = nullptr;
            admin_log_success_only_ = nullptr;
            admin_log_failure_only_ = nullptr;
        }
    }
}

void AppWindow::applyPermissionToControls() {
    if (!permission_manager_) {
        return;
    }
    if (btn_power_on_) {
        btn_power_on_->setEnabled(
            permission_manager_->hasPermission(current_role_, ActionType::PowerOn));
    }
    if (btn_power_off_) {
        btn_power_off_->setEnabled(
            permission_manager_->hasPermission(current_role_, ActionType::PowerOff));
    }
    bool can_modify = permission_manager_->hasPermission(current_role_, ActionType::ModifyParam);
    bool can_calib = permission_manager_->hasPermission(current_role_, ActionType::CalibrateSystem);

    if (btn_enable_) {
        btn_enable_->setEnabled(can_modify);
    }
    if (btn_disable_) {
        btn_disable_->setEnabled(can_modify);
    }

    if (btn_lhand_enable_) {
        btn_lhand_enable_->setEnabled(can_modify);
    }
    if (btn_lhand_disable_) {
        btn_lhand_disable_->setEnabled(can_modify);
    }
    if (btn_lhand_home_) {
        btn_lhand_home_->setEnabled(can_calib);
    }
    if (btn_lhand_move_) {
        btn_lhand_move_->setEnabled(can_modify);
    }
    if (btn_lhand_set_vel_) {
        btn_lhand_set_vel_->setEnabled(can_modify);
    }
    for (int i = 0; i < 6; ++i) {
        if (lhand_joint_buttons_[i]) {
            lhand_joint_buttons_[i]->setEnabled(can_modify);
        }
    }
}

QWidget* AppWindow::createLHandTab() {
    auto * main_widget = new QWidget();
    auto * main_layout = new QHBoxLayout(main_widget);

    auto * widget = new QWidget();
    auto * layout = new QVBoxLayout();

    // --- Global Controls ---
    auto * group_global = new QGroupBox("Global Control");
    auto * layout_global = new QHBoxLayout();
    
    btn_lhand_enable_ = new QPushButton("Enable All");
    btn_lhand_disable_ = new QPushButton("Disable All");
    btn_lhand_home_ = new QPushButton("Home All");
    
    layout_global->addWidget(btn_lhand_enable_);
    layout_global->addWidget(btn_lhand_disable_);
    layout_global->addWidget(btn_lhand_home_);
    
    group_global->setLayout(layout_global);
    layout->addWidget(group_global);

    // --- Joint Controls ---
    auto * group_joints = new QGroupBox("Joint Control");
    auto * layout_joints = new QGridLayout();
    
    layout_joints->addWidget(new QLabel("Joint"), 0, 0);
    layout_joints->addWidget(new QLabel("Position (0-1000)"), 0, 1);
    layout_joints->addWidget(new QLabel("Action"), 0, 2);

    for(int i=0; i<6; ++i) {
        layout_joints->addWidget(new QLabel(QString("J%1").arg(i+1)), i+1, 0);
        
        spin_lhand_pos_[i] = new QSpinBox();
        spin_lhand_pos_[i]->setRange(0, 10000); 
        spin_lhand_pos_[i]->setValue(0);
        layout_joints->addWidget(spin_lhand_pos_[i], i+1, 1);
        
        lhand_joint_buttons_[i] = new QPushButton("Set & Move");
        layout_joints->addWidget(lhand_joint_buttons_[i], i+1, 2);
        
        connect(lhand_joint_buttons_[i], &QPushButton::clicked, this, [this, i](){
            if (permission_manager_ &&
                !permission_manager_->hasPermission(current_role_, ActionType::ModifyParam)) {
                QMessageBox::warning(this, "权限不足", "当前用户无权单独控制手指关节");
                return;
            }
            int pos = spin_lhand_pos_[i]->value();
            node_->call_lhand_set_position(i+1, pos); 
            node_->call_lhand_move(i+1);
        });
    }
    
    group_joints->setLayout(layout_joints);
    layout->addWidget(group_joints);

    // --- Global Velocity ---
    auto * group_vel = new QGroupBox("Velocity Control");
    auto * layout_vel = new QHBoxLayout();
    layout_vel->addWidget(new QLabel("Velocity (0-20000):"));
    
    spin_lhand_vel_ = new QSpinBox();
    spin_lhand_vel_->setRange(0, 50000);
    spin_lhand_vel_->setValue(20000);
    layout_vel->addWidget(spin_lhand_vel_);
    
    btn_lhand_set_vel_ = new QPushButton("Set Velocity (All)");
    layout_vel->addWidget(btn_lhand_set_vel_);
    
    group_vel->setLayout(layout_vel);
    layout->addWidget(group_vel);

    // --- Global Move ---
    btn_lhand_move_ = new QPushButton("Move All Joints to Target");
    layout->addWidget(btn_lhand_move_);
    
    layout->addStretch();

    widget->setLayout(layout);
    
    // Right 3D Panel
    lhand_viz_ = new RobotVizWidget(node_);
    std::string lhand_path = node_->get_left_hand_urdf_path();
    if (!lhand_path.empty()) {
        lhand_viz_->loadRobotModel(lhand_path);
        // Adjust camera for hand scale
        lhand_viz_->setCameraPosition(QVector3D(0.3f, 0.2f, 0.3f), QVector3D(0.0f, 0.05f, 0.0f));
    }
    
    main_layout->addWidget(widget, 1);
    main_layout->addWidget(lhand_viz_, 2);

    // Connections
    connect(btn_lhand_enable_, &QPushButton::clicked, this, [this](){
        if (permission_manager_ &&
            !permission_manager_->hasPermission(current_role_, ActionType::ModifyParam)) {
            QMessageBox::warning(this, "权限不足", "当前用户无权使能灵巧手");
            return;
        }
        node_->call_lhand_enable(true);
    });
    
    connect(btn_lhand_disable_, &QPushButton::clicked, this, [this](){
        if (permission_manager_ &&
            !permission_manager_->hasPermission(current_role_, ActionType::ModifyParam)) {
            QMessageBox::warning(this, "权限不足", "当前用户无权去使能灵巧手");
            return;
        }
        node_->call_lhand_enable(false);
    });

    connect(btn_lhand_home_, &QPushButton::clicked, this, [this](){
        if (permission_manager_ &&
            !permission_manager_->hasPermission(current_role_, ActionType::CalibrateSystem)) {
            QMessageBox::warning(this, "权限不足", "当前用户无权执行灵巧手回零");
            return;
        }
        node_->call_lhand_home(0);
    });
    
    connect(btn_lhand_set_vel_, &QPushButton::clicked, this, [this](){
        if (permission_manager_ &&
            !permission_manager_->hasPermission(current_role_, ActionType::ModifyParam)) {
            QMessageBox::warning(this, "权限不足", "当前用户无权修改灵巧手速度");
            return;
        }
        int vel = spin_lhand_vel_->value();
        for(int i=0; i<6; ++i) node_->call_lhand_set_velocity(i+1, vel);
    });
    
    connect(btn_lhand_move_, &QPushButton::clicked, this, [this](){
        if (permission_manager_ &&
            !permission_manager_->hasPermission(current_role_, ActionType::ModifyParam)) {
            QMessageBox::warning(this, "权限不足", "当前用户无权整体移动灵巧手");
            return;
        }
        std::array<int, 6> positions;
        for(int i=0; i<6; ++i) {
             positions[i] = spin_lhand_pos_[i]->value();
        }
        node_->call_lhand_set_all_position(positions);
        
        QTimer::singleShot(10, this, [this](){
            node_->call_lhand_move(0);
        });
    });

    return main_widget;
}

QWidget* AppWindow::createAdminTab() {
    auto * widget = new QWidget();
    auto * layout = new QVBoxLayout(widget);

    auto * group_users = new QGroupBox("用户管理");
    auto * layout_users = new QVBoxLayout();
    admin_user_table_ = new QTableWidget();
    admin_user_table_->setColumnCount(5);
    admin_user_table_->setHorizontalHeaderLabels(QStringList() << "用户名" << "角色" << "失败次数" << "锁定" << "禁用");
    admin_user_table_->horizontalHeader()->setStretchLastSection(true);
    admin_user_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    admin_user_table_->setSelectionMode(QAbstractItemView::SingleSelection);
    layout_users->addWidget(admin_user_table_);

    auto * user_btn_layout = new QHBoxLayout();
    auto * btn_refresh_users = new QPushButton("刷新");
    auto * btn_create_user = new QPushButton("创建用户");
    auto * btn_delete_user = new QPushButton("删除用户");
    auto * btn_lock_user = new QPushButton("锁定");
    auto * btn_unlock_user = new QPushButton("解锁");
    auto * btn_reset_password = new QPushButton("重置密码");

    user_btn_layout->addWidget(btn_refresh_users);
    user_btn_layout->addWidget(btn_create_user);
    user_btn_layout->addWidget(btn_delete_user);
    user_btn_layout->addWidget(btn_lock_user);
    user_btn_layout->addWidget(btn_unlock_user);
    user_btn_layout->addWidget(btn_reset_password);
    user_btn_layout->addStretch();
    layout_users->addLayout(user_btn_layout);
    group_users->setLayout(layout_users);
    layout->addWidget(group_users);

    auto refreshUsers = [this]() {
        if (!auth_manager_ || !admin_user_table_) {
            return;
        }
        QVector<UserInfo> users;
        if (!auth_manager_->listUsers(users)) {
            return;
        }
        admin_user_table_->setRowCount(users.size());
        for (int i = 0; i < users.size(); ++i) {
            const UserInfo& u = users[i];
            auto * item_name = new QTableWidgetItem(u.username);
            QString role_str = "Admin";
            if (u.role == UserRole::Operator) role_str = "Operator";
            else if (u.role == UserRole::Maintainer) role_str = "Maintainer";
            auto * item_role = new QTableWidgetItem(role_str);
            auto * item_fail = new QTableWidgetItem(QString::number(u.failed_attempts));
            qint64 now = QDateTime::currentSecsSinceEpoch();
            bool locked = u.lock_until > now;
            auto * item_locked = new QTableWidgetItem(locked ? "是" : "否");
            auto * item_disabled = new QTableWidgetItem(u.disabled ? "是" : "否");
            admin_user_table_->setItem(i, 0, item_name);
            admin_user_table_->setItem(i, 1, item_role);
            admin_user_table_->setItem(i, 2, item_fail);
            admin_user_table_->setItem(i, 3, item_locked);
            admin_user_table_->setItem(i, 4, item_disabled);
        }
        admin_user_table_->resizeColumnsToContents();
    };

    bool can_create = permission_manager_ &&
                      permission_manager_->hasPermission(current_role_, ActionType::CreateUser);
    bool can_modify = permission_manager_ &&
                      permission_manager_->hasPermission(current_role_, ActionType::ModifyUser);
    bool can_delete = permission_manager_ &&
                      permission_manager_->hasPermission(current_role_, ActionType::DeleteUser);
    bool can_reset_pw = permission_manager_ &&
                        permission_manager_->hasPermission(current_role_, ActionType::ResetUserPassword);
    bool can_lock = permission_manager_ &&
                    permission_manager_->hasPermission(current_role_, ActionType::LockUnlockUser);
    bool can_view_logs = permission_manager_ &&
                         permission_manager_->hasPermission(current_role_, ActionType::ViewAuthLog);
    bool can_delete_logs = permission_manager_ &&
                           permission_manager_->hasPermission(current_role_, ActionType::DeleteAuthLog);

    btn_create_user->setEnabled(can_create);
    btn_delete_user->setEnabled(can_delete);
    btn_lock_user->setEnabled(can_lock);
    btn_unlock_user->setEnabled(can_lock);
    btn_reset_password->setEnabled(can_reset_pw);

    connect(btn_refresh_users, &QPushButton::clicked, this, [refreshUsers]() {
        refreshUsers();
    });

    connect(btn_create_user, &QPushButton::clicked, this, [this, can_create, refreshUsers]() {
        if (!can_create || !auth_manager_) {
            return;
        }
        bool ok = false;
        QString username = QInputDialog::getText(this, "创建用户", "用户名:", QLineEdit::Normal, "", &ok);
        if (!ok || username.trimmed().isEmpty()) {
            return;
        }
        QString password = QInputDialog::getText(this, "创建用户", "密码:", QLineEdit::Password, "", &ok);
        if (!ok || password.isEmpty()) {
            return;
        }
        QString confirm = QInputDialog::getText(this, "创建用户", "确认密码:", QLineEdit::Password, "", &ok);
        if (!ok || confirm != password) {
            QMessageBox::warning(this, "创建用户失败", "两次输入的密码不一致");
            return;
        }
        QStringList roles;
        roles << "Operator" << "Maintainer" << "Admin";
        QString role_str = QInputDialog::getItem(this, "创建用户", "角色:", roles, 0, false, &ok);
        if (!ok || role_str.isEmpty()) {
            return;
        }
        UserRole role = UserRole::Operator;
        if (role_str.compare("Maintainer", Qt::CaseInsensitive) == 0) {
            role = UserRole::Maintainer;
        } else if (role_str.compare("Admin", Qt::CaseInsensitive) == 0) {
            role = UserRole::Admin;
        }
        QString err;
        if (!auth_manager_->createUser(username.trimmed(), password, role, err)) {
            QMessageBox::warning(this, "创建用户失败", err);
            return;
        }
        refreshUsers();
    });

    auto selectedUsername = [this]() -> QString {
        if (!admin_user_table_) {
            return QString();
        }
        auto ranges = admin_user_table_->selectedRanges();
        if (ranges.isEmpty()) {
            return QString();
        }
        int row = ranges.first().topRow();
        auto * item = admin_user_table_->item(row, 0);
        if (!item) {
            return QString();
        }
        return item->text();
    };

    connect(btn_delete_user, &QPushButton::clicked, this, [this, can_delete, refreshUsers, selectedUsername]() {
        if (!can_delete || !auth_manager_) {
            return;
        }
        QString username = selectedUsername();
        if (username.isEmpty()) {
            QMessageBox::warning(this, "删除用户", "请先选择要删除的用户");
            return;
        }
        if (QMessageBox::question(this, "删除用户", "确认删除用户 " + username + " ?") != QMessageBox::Yes) {
            return;
        }
        QString err;
        if (!auth_manager_->deleteUser(username, err)) {
            QMessageBox::warning(this, "删除用户失败", err);
            return;
        }
        refreshUsers();
    });

    connect(btn_lock_user, &QPushButton::clicked, this, [this, can_lock, refreshUsers, selectedUsername]() {
        if (!can_lock || !auth_manager_) {
            return;
        }
        QString username = selectedUsername();
        if (username.isEmpty()) {
            QMessageBox::warning(this, "锁定用户", "请先选择要锁定的用户");
            return;
        }
        QString err;
        if (!auth_manager_->setLocked(username, true, err)) {
            QMessageBox::warning(this, "锁定用户失败", err);
            return;
        }
        refreshUsers();
    });

    connect(btn_unlock_user, &QPushButton::clicked, this, [this, can_lock, refreshUsers, selectedUsername]() {
        if (!can_lock || !auth_manager_) {
            return;
        }
        QString username = selectedUsername();
        if (username.isEmpty()) {
            QMessageBox::warning(this, "解锁用户", "请先选择要解锁的用户");
            return;
        }
        QString err;
        if (!auth_manager_->setLocked(username, false, err)) {
            QMessageBox::warning(this, "解锁用户失败", err);
            return;
        }
        refreshUsers();
    });

    connect(btn_reset_password, &QPushButton::clicked, this, [this, can_reset_pw, refreshUsers, selectedUsername]() {
        if (!can_reset_pw || !auth_manager_) {
            return;
        }
        QString username = selectedUsername();
        if (username.isEmpty()) {
            QMessageBox::warning(this, "重置密码", "请先选择要重置密码的用户");
            return;
        }
        bool ok = false;
        QString password = QInputDialog::getText(this, "重置密码", "新密码:", QLineEdit::Password, "", &ok);
        if (!ok || password.isEmpty()) {
            return;
        }
        QString confirm = QInputDialog::getText(this, "重置密码", "确认新密码:", QLineEdit::Password, "", &ok);
        if (!ok || confirm != password) {
            QMessageBox::warning(this, "重置密码失败", "两次输入的密码不一致");
            return;
        }
        QString err;
        if (!auth_manager_->adminResetPassword(username, password, true, err)) {
            QMessageBox::warning(this, "重置密码失败", err);
            return;
        }
        refreshUsers();
    });

    refreshUsers();

    if (can_view_logs && auth_log_manager_) {
        auto * group_logs = new QGroupBox("登录日志");
        auto * layout_logs = new QVBoxLayout();

        auto * filter_layout = new QHBoxLayout();
        admin_log_from_ = new QDateTimeEdit(QDateTime::currentDateTime().addDays(-7));
        admin_log_to_ = new QDateTimeEdit(QDateTime::currentDateTime());
        admin_log_from_->setDisplayFormat("yyyy-MM-dd HH:mm:ss");
        admin_log_to_->setDisplayFormat("yyyy-MM-dd HH:mm:ss");
        admin_log_user_filter_ = new QLineEdit();
        admin_log_success_only_ = new QCheckBox("仅成功");
        admin_log_failure_only_ = new QCheckBox("仅失败");
        auto * btn_refresh_logs = new QPushButton("刷新");
        auto * btn_delete_old = new QPushButton("删除早于起始时间的日志");

        filter_layout->addWidget(new QLabel("起始时间"));
        filter_layout->addWidget(admin_log_from_);
        filter_layout->addWidget(new QLabel("结束时间"));
        filter_layout->addWidget(admin_log_to_);
        filter_layout->addWidget(new QLabel("用户过滤"));
        filter_layout->addWidget(admin_log_user_filter_);
        filter_layout->addWidget(admin_log_success_only_);
        filter_layout->addWidget(admin_log_failure_only_);
        filter_layout->addWidget(btn_refresh_logs);
        if (!can_delete_logs) {
            btn_delete_old->setEnabled(false);
        }
        filter_layout->addWidget(btn_delete_old);
        filter_layout->addStretch();

        layout_logs->addLayout(filter_layout);

        admin_log_table_ = new QTableWidget();
        admin_log_table_->setColumnCount(5);
        admin_log_table_->setHorizontalHeaderLabels(QStringList() << "时间" << "用户" << "结果" << "原因" << "来源");
        admin_log_table_->horizontalHeader()->setStretchLastSection(true);
        admin_log_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
        admin_log_table_->setSelectionMode(QAbstractItemView::SingleSelection);
        layout_logs->addWidget(admin_log_table_);

        auto refreshLogs = [this]() {
            if (!auth_log_manager_ || !admin_log_table_) {
                return;
            }
            QDateTime from = admin_log_from_ ? admin_log_from_->dateTime() : QDateTime();
            QDateTime to = admin_log_to_ ? admin_log_to_->dateTime() : QDateTime();
            QString user_filter = admin_log_user_filter_ ? admin_log_user_filter_->text().trimmed() : QString();
            bool success_only = admin_log_success_only_ && admin_log_success_only_->isChecked();
            bool failure_only = admin_log_failure_only_ && admin_log_failure_only_->isChecked();
            QVector<AuthLogEntry> logs = auth_log_manager_->loadLogs(from, to, user_filter, success_only, failure_only);
            admin_log_table_->setRowCount(logs.size());
            for (int i = 0; i < logs.size(); ++i) {
                const AuthLogEntry& e = logs[i];
                auto * item_time = new QTableWidgetItem(e.timestamp.toString(Qt::ISODate));
                auto * item_user = new QTableWidgetItem(e.username);
                auto * item_result = new QTableWidgetItem(e.success ? "成功" : "失败");
                auto * item_reason = new QTableWidgetItem(e.reason);
                auto * item_source = new QTableWidgetItem(e.source);
                admin_log_table_->setItem(i, 0, item_time);
                admin_log_table_->setItem(i, 1, item_user);
                admin_log_table_->setItem(i, 2, item_result);
                admin_log_table_->setItem(i, 3, item_reason);
                admin_log_table_->setItem(i, 4, item_source);
            }
            admin_log_table_->resizeColumnsToContents();
        };

        connect(btn_refresh_logs, &QPushButton::clicked, this, [refreshLogs]() {
            refreshLogs();
        });

        connect(btn_delete_old, &QPushButton::clicked, this, [this, can_delete_logs]() {
            if (!can_delete_logs || !auth_log_manager_ || !admin_log_from_) {
                return;
            }
            QDateTime before = admin_log_from_->dateTime();
            if (!before.isValid()) {
                return;
            }
            if (QMessageBox::question(this, "删除日志", "确认删除早于起始时间的日志记录?") != QMessageBox::Yes) {
                return;
            }
            if (!auth_log_manager_->deleteLogs(before)) {
                QMessageBox::warning(this, "删除日志失败", "删除日志时发生错误");
            }
        });

        refreshLogs();

        group_logs->setLayout(layout_logs);
        layout->addWidget(group_logs);
    }

    return widget;
}
