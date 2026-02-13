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

AppWindow::AppWindow(std::shared_ptr<RosNode> node, QWidget *parent) 
    : QWidget(parent), node_(std::move(node)) 
{
    auto * main_layout = new QVBoxLayout();
    
    // --- Header ---
    label_count_ = new QLabel("Heartbeat: 0");
    main_layout->addWidget(label_count_);

    // --- Tab Widget ---
    auto * tabs = new QTabWidget();
    tabs->addTab(createControlTab(), "Power & Status");
    tabs->addTab(createMoveTab(), "Motion Control");
    tabs->addTab(createCameraTab(), "Vision System");
    
    // Robot Viz
    robot_viz_ = new RobotVizWidget(node_);
    tabs->addTab(robot_viz_, "3D Simulation");
    // Load model from parameter
    std::string urdf_path = node_->get_robot_urdf_path();
    if (urdf_path.empty()) {
         // Fallback default
         urdf_path = "/home/user/IntelliegntControllerHC/ros2_ws/src/duco_support/urdf/duco_gcr16_960.urdf";
    }
    robot_viz_->loadRobotModel(urdf_path);

    // Load Left Hand
    std::string lhand_path = node_->get_left_hand_urdf_path();
    if (!lhand_path.empty()) robot_viz_->loadRobotModel(lhand_path);

    // Load Right Hand
    std::string rhand_path = node_->get_right_hand_urdf_path();
    if (!rhand_path.empty()) robot_viz_->loadRobotModel(rhand_path);

    tabs->addTab(createLHandTab(), "LHand Control");
    main_layout->addWidget(tabs);
    
    setLayout(main_layout);
    
    // Timer for UI updates
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this](){
      updateUI();
    });
    timer_->start(100);
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

      // Connect
      connect(btn_power_on_, &QPushButton::clicked, this, [this](){ node_->call_robot_control("poweron"); });
      connect(btn_enable_, &QPushButton::clicked, this, [this](){ node_->call_robot_control("enable"); });
      connect(btn_disable_, &QPushButton::clicked, this, [this](){ node_->call_robot_control("disable"); });
      connect(btn_power_off_, &QPushButton::clicked, this, [this](){ node_->call_robot_control("poweroff"); });

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
          if(node_->current_joints_.size() >= 7) {
              for(int i=0; i<7; ++i) spin_joints_[i]->setValue(node_->current_joints_[i]);
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
        node_->save_snapshot(combo_camera_->currentText().toStdString(), true, false, false, false, make_callback());
    });
    widget_depth_ = createVideoWidget("Depth Stream", label_depth_stream_, [this, make_callback](){
        node_->save_snapshot(combo_camera_->currentText().toStdString(), false, true, false, false, make_callback());
    });
    widget_ir_left_ = createVideoWidget("IR Left Stream", label_ir_left_stream_, [this, make_callback](){
        node_->save_snapshot(combo_camera_->currentText().toStdString(), false, false, true, false, make_callback());
    });
    widget_ir_right_ = createVideoWidget("IR Right Stream", label_ir_right_stream_, [this, make_callback](){
        node_->save_snapshot(combo_camera_->currentText().toStdString(), false, false, false, true, make_callback());
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
    // Add default if empty and camera combo has items
    // This is purely UI logic now, we don't force push to combo unless needed.
    // The previous logic was:
    // if(combo_pc_topic_->count() == 0) { ... } 
    // We removed the hardcoded camera fallback in ros_node.cpp, so we should be clean here.
    
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

QWidget* AppWindow::createLHandTab() {
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
        
        auto * btn_set = new QPushButton("Set & Move");
        layout_joints->addWidget(btn_set, i+1, 2);
        
        // Connect per-joint set
        connect(btn_set, &QPushButton::clicked, this, [this, i](){
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
    
    auto * btn_set_vel = new QPushButton("Set Velocity (All)");
    layout_vel->addWidget(btn_set_vel);
    
    group_vel->setLayout(layout_vel);
    layout->addWidget(group_vel);

    // --- Global Move ---
    btn_lhand_move_ = new QPushButton("Move All Joints to Target");
    layout->addWidget(btn_lhand_move_);
    
    layout->addStretch();

    widget->setLayout(layout);
    
    // Connections
    connect(btn_lhand_enable_, &QPushButton::clicked, this, [this](){
        node_->call_lhand_enable(0, 1);
    });
    
    connect(btn_lhand_disable_, &QPushButton::clicked, this, [this](){
        node_->call_lhand_enable(0, 0);
    });

    connect(btn_lhand_home_, &QPushButton::clicked, this, [this](){
        node_->call_lhand_home(0);
    });
    
    connect(btn_set_vel, &QPushButton::clicked, this, [this](){
        int vel = spin_lhand_vel_->value();
        for(int i=0; i<6; ++i) node_->call_lhand_set_velocity(i+1, vel);
    });
    
    connect(btn_lhand_move_, &QPushButton::clicked, this, [this](){
        std::array<int, 6> positions;
        for(int i=0; i<6; ++i) {
             positions[i] = spin_lhand_pos_[i]->value();
        }
        node_->call_lhand_set_all_position(positions);
        
        QTimer::singleShot(10, this, [this](){
            node_->call_lhand_move(0);
        });
    });

    return widget;
}

