#include "ui_app/app_window.hpp"
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
    tabs->addTab(createIOTab(), "IO Control");
    tabs->addTab(createCameraTab(), "Vision System");
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
      
      btn_power_on_->setStyleSheet("background-color: #e0ffe0");
      btn_power_off_->setStyleSheet("background-color: #ffe0e0");

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

QWidget* AppWindow::createIOTab() {
      auto * widget = new QWidget();
      auto * layout = new QVBoxLayout();

      auto * group_io = new QGroupBox("Digital Output Control");
      auto * layout_io = new QGridLayout();

      layout_io->addWidget(new QLabel("IO Type:"), 0, 0);
      combo_io_type_ = new QComboBox();
      combo_io_type_->addItem("Controller IO (0)", 0);
      combo_io_type_->addItem("Tool IO (1)", 1);
      layout_io->addWidget(combo_io_type_, 0, 1);

      layout_io->addWidget(new QLabel("Port Index:"), 1, 0);
      spin_io_port_ = new QSpinBox();
      spin_io_port_->setRange(0, 15);
      layout_io->addWidget(spin_io_port_, 1, 1);

      layout_io->addWidget(new QLabel("Value:"), 2, 0);
      chk_io_value_ = new QCheckBox("High / On");
      layout_io->addWidget(chk_io_value_, 2, 1);

      auto * btn_set_io = new QPushButton("Set IO");
      layout_io->addWidget(btn_set_io, 3, 0, 1, 2);

      group_io->setLayout(layout_io);
      layout->addWidget(group_io);
      layout->addStretch();

      widget->setLayout(layout);

      connect(btn_set_io, &QPushButton::clicked, this, [this](){
          int type = combo_io_type_->currentData().toInt();
          int port = spin_io_port_->value();
          bool val = chk_io_value_->isChecked();
          node_->call_robot_io("setIo", type, port, val);
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
    check_ir_left_ = new QCheckBox("IR Left");
    check_ir_right_ = new QCheckBox("IR Right");
    
    // Defaults: Color and Depth checked
    check_color_->setChecked(true);
    check_depth_->setChecked(true);
    check_ir_left_->setChecked(false);
    check_ir_right_->setChecked(false);

    layout_sensors->addWidget(check_color_);
    layout_sensors->addWidget(check_depth_);
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
    widget_color_ = createVideoWidget("Color Stream", label_color_stream_, [this](){
        node_->save_snapshot(combo_camera_->currentText().toStdString(), true, false, false, false);
    });
    widget_depth_ = createVideoWidget("Depth Stream", label_depth_stream_, [this](){
        node_->save_snapshot(combo_camera_->currentText().toStdString(), false, true, false, false);
    });
    widget_ir_left_ = createVideoWidget("IR Left Stream", label_ir_left_stream_, [this](){
        node_->save_snapshot(combo_camera_->currentText().toStdString(), false, false, true, false);
    });
    widget_ir_right_ = createVideoWidget("IR Right Stream", label_ir_right_stream_, [this](){
        node_->save_snapshot(combo_camera_->currentText().toStdString(), false, false, false, true);
    });

    // Add to grid (2x2)
    grid_video->addWidget(widget_color_, 0, 0);
    grid_video->addWidget(widget_depth_, 0, 1);
    grid_video->addWidget(widget_ir_left_, 1, 0);
    grid_video->addWidget(widget_ir_right_, 1, 1);
    
    scroll->setWidget(container_video_);
    layout->addWidget(scroll);

    // --- Connections ---
    connect(btn_scan_, &QPushButton::clicked, this, &AppWindow::refreshCameraList);
    
    auto update_config = [this]() { onCameraConfigChanged(); };
    connect(combo_camera_, &QComboBox::currentTextChanged, this, update_config);
    connect(check_color_, &QCheckBox::stateChanged, this, update_config);
    connect(check_depth_, &QCheckBox::stateChanged, this, update_config);
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
    label_ptr->setStyleSheet("border: 1px solid #555; background-color: #222; color: #aaa;");
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
    // Trigger update if selection changed (or just force it)
    if (combo_camera_->count() > 0) {
        combo_camera_->setCurrentIndex(0);
        onCameraConfigChanged(); 
    }
}

void AppWindow::onCameraConfigChanged() {
    std::string cam_ns = combo_camera_->currentText().toStdString();
    bool c = check_color_->isChecked();
    bool d = check_depth_->isChecked();
    bool ir_l = check_ir_left_->isChecked();
    bool ir_r = check_ir_right_->isChecked();

    // Update Node Subscriptions
    node_->update_camera_subscriptions(cam_ns, c, d, ir_l, ir_r);

    // Update UI Visibility
    widget_color_->setVisible(c);
    widget_depth_->setVisible(d);
    widget_ir_left_->setVisible(ir_l);
    widget_ir_right_->setVisible(ir_r);
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
