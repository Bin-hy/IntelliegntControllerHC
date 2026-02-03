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

void AppWindow::updateUI() {
      label_count_->setText("Heartbeat: " + QString::number(node_->count_.load()));
      
      std::lock_guard<std::mutex> lock(node_->data_mutex_);
      
      if (!node_->last_robot_state_str_.empty()) {
          text_robot_state_->setText(QString::fromStdString(node_->last_robot_state_str_));
      }
}
