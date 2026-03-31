#include "ui_app/task_dialogs.hpp"
#include "ui_app/i18n_manager.hpp"
#include "ui_app/robot_viz_widget.hpp"
#include <cmath>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QTableWidget>
#include <QHeaderView>
#include <QTableWidgetItem>
#include <QListWidget>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QLineEdit>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QMessageBox>
#include <QStackedWidget>
#include <QTimer>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QFileDialog>
#include <QFile>
#include <sstream>

DeviceAddDialog::DeviceAddDialog(std::shared_ptr<RosNode> node, QWidget *parent)
    : QDialog(parent), node_(node) {
    setWindowTitle(tr_ui("添加设备", "Add Device"));
    auto * layout = new QVBoxLayout(this);

    auto * form = new QFormLayout();
    combo_type_ = new QComboBox();
    combo_type_->addItems({"duco", "lhand", "rhand", "orbbec"});
    form->addRow(tr_ui("设备类型:", "Device Type:"), combo_type_);

    edit_usage_ = new QLineEdit();
    form->addRow(tr_ui("用途:", "Usage:"), edit_usage_);

    combo_device_select_ = new QComboBox();
    form->addRow(tr_ui("设备:", "Device:"), combo_device_select_);

    edit_sn_ = new QLineEdit();
    form->addRow(tr_ui("SN码:", "SN:"), edit_sn_);

    edit_model_ = new QLineEdit();
    form->addRow(tr_ui("型号:", "Model:"), edit_model_);

    layout->addLayout(form);

    auto * h_btn = new QHBoxLayout();
    btn_scan_ = new QPushButton(tr_ui("扫描", "Scan"));
    h_btn->addStretch();
    h_btn->addWidget(btn_scan_);
    layout->addLayout(h_btn);

    auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);

    connect(combo_type_, &QComboBox::currentTextChanged, this, &DeviceAddDialog::onTypeChanged);
    connect(btn_scan_, &QPushButton::clicked, this, &DeviceAddDialog::onScan);
    connect(combo_device_select_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &DeviceAddDialog::onDeviceIndexChanged);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    onScan();
}

void DeviceAddDialog::onTypeChanged(const QString&) {
    onScan();
}

void DeviceAddDialog::onDeviceIndexChanged(int index) {
    if (index < 0) return;
    QString sn = combo_device_select_->itemData(index).toString();
    if (!sn.isEmpty()) edit_sn_->setText(sn);
}

void DeviceAddDialog::onScan() {
    combo_device_select_->clear();
    QString type = combo_type_->currentText();
    std::string target_type = type.toStdString();

    auto devices = node_->get_connected_devices();
    bool found = false;

    for (const auto& dev : devices) {
        bool match = false;
        if (target_type == "duco" && dev.device_type == "duco") match = true;
        else if (target_type == "lhand" && dev.device_type == "lhand") match = true;
        else if (target_type == "rhand" && dev.device_type == "rhand") match = true;
        else if (target_type == "orbbec" && (dev.device_type == "orbbec" || dev.device_type == "camera_server" || dev.device_type == "vision_system")) match = true;

        if (match) {
            QString label = QString::fromStdString(dev.device_model.empty() ? dev.device_name : dev.device_model);
            if (!dev.device_sn.empty()) label += " (" + QString::fromStdString(dev.device_sn) + ")";
            combo_device_select_->addItem(label, QString::fromStdString(dev.device_sn));
            found = true;
        }
    }

    if (!found && target_type == "orbbec") {
        auto cameras = node_->scan_cameras();
        for (const auto& cam : cameras) {
            combo_device_select_->addItem(QString::fromStdString(cam), QString::fromStdString(cam));
        }
        if (!cameras.empty()) found = true;
    }

    if (!found) {
        QMessageBox::information(this, tr_ui("扫描", "Scan"), tr_ui("未找到此类型的设备: ", "No devices found of type ") + type);
    }
}

common_msgs::msg::TaskDeviceCheck DeviceAddDialog::getDeviceCheck() const {
    common_msgs::msg::TaskDeviceCheck check;
    check.device_type = combo_type_->currentText().toStdString();
    check.device_usage = edit_usage_->text().toStdString();
    check.device_name = combo_device_select_->currentText().toStdString();
    check.device_sn = combo_device_select_->currentData().toString().toStdString();
    if (check.device_sn.empty()) check.device_sn = edit_sn_->text().toStdString();
    check.device_model = edit_model_->text().toStdString();
    return check;
}

StepAddDialog::StepAddDialog(std::shared_ptr<RosNode> node, const std::vector<common_msgs::msg::TaskDeviceCheck>& devices, QWidget *parent)
    : QDialog(parent), node_(node), devices_(devices) {
    setWindowTitle(tr_ui("添加步骤", "Add Step"));
    auto * layout = new QVBoxLayout(this);

    auto * h_name = new QHBoxLayout();
    h_name->addWidget(new QLabel(tr_ui("步骤名称:", "Step Name:")));
    edit_name_ = new QLineEdit();
    h_name->addWidget(edit_name_);
    layout->addLayout(h_name);

    combo_type_ = new QComboBox();
    combo_type_->addItems({"arm", "lhand", "rhand", "camera", "io", "lift", "control"});
    layout->addWidget(new QLabel(tr_ui("步骤类型:", "Step Type:")));
    layout->addWidget(combo_type_);

    layout->addWidget(new QLabel(tr_ui("目标设备:", "Target Device:")));
    combo_device_ = new QComboBox();
    layout->addWidget(combo_device_);

    auto * stack = new QStackedWidget();

    widget_arm_ = new QWidget();
    auto * arm_layout = new QVBoxLayout(widget_arm_);

    // MoveJ / MoveL command selector
    auto * h_arm_cmd = new QHBoxLayout();
    h_arm_cmd->addWidget(new QLabel(tr_ui("运动类型:", "Motion Type:")));
    combo_arm_command_ = new QComboBox();
    combo_arm_command_->addItem(tr_ui("关节运动 (MoveJ)", "Joint Move (MoveJ)"), "movej");
    combo_arm_command_->addItem(tr_ui("直线运动 (MoveL)", "Linear Move (MoveL)"), "movel");
    h_arm_cmd->addWidget(combo_arm_command_);
    arm_layout->addLayout(h_arm_cmd);

    // --- MoveJ: joint angle inputs (degrees) ---
    widget_arm_movej_ = new QWidget();
    auto * movej_layout = new QFormLayout(widget_arm_movej_);
    for (int i = 0; i < 6; ++i) {
        spin_arm_pos_[i] = new QDoubleSpinBox();
        spin_arm_pos_[i]->setRange(-360.0, 360.0);
        spin_arm_pos_[i]->setDecimals(3);
        spin_arm_pos_[i]->setSingleStep(1.0);
        spin_arm_pos_[i]->setSuffix(QString::fromUtf8(" °"));
        movej_layout->addRow(QString("J%1:").arg(i + 1), spin_arm_pos_[i]);
    }
    auto * btn_capture = new QPushButton(tr_ui("获取当前位置", "Capture Current"));
    movej_layout->addRow(btn_capture);
    connect(btn_capture, &QPushButton::clicked, this, &StepAddDialog::onCaptureCurrent);

    // --- MoveL: cartesian inputs ---
    widget_arm_movel_ = new QWidget();
    auto * movel_layout = new QFormLayout(widget_arm_movel_);
    const QStringList cart_labels = {"X (mm):", "Y (mm):", "Z (mm):", QString::fromUtf8("RX (°):"), QString::fromUtf8("RY (°):"), QString::fromUtf8("RZ (°):")};
    for (int i = 0; i < 6; ++i) {
        spin_arm_cart_[i] = new QDoubleSpinBox();
        if (i < 3) {
            spin_arm_cart_[i]->setRange(-3000.0, 3000.0);
            spin_arm_cart_[i]->setDecimals(3);
            spin_arm_cart_[i]->setSingleStep(1.0);
            spin_arm_cart_[i]->setSuffix(" mm");
        } else {
            spin_arm_cart_[i]->setRange(-360.0, 360.0);
            spin_arm_cart_[i]->setDecimals(3);
            spin_arm_cart_[i]->setSingleStep(1.0);
            spin_arm_cart_[i]->setSuffix(QString::fromUtf8(" °"));
        }
        movel_layout->addRow(cart_labels[i], spin_arm_cart_[i]);
    }

    arm_layout->addWidget(widget_arm_movej_);
    arm_layout->addWidget(widget_arm_movel_);

    // --- Velocity / Accel ---
    auto * h_vel = new QHBoxLayout();
    spin_arm_velocity_ = new QDoubleSpinBox();
    spin_arm_velocity_->setRange(0.1, 180.0);
    spin_arm_velocity_->setValue(30.0);
    spin_arm_velocity_->setDecimals(1);
    spin_arm_velocity_->setSingleStep(5.0);
    spin_arm_velocity_->setSuffix(QString::fromUtf8(" °/s"));
    h_vel->addWidget(new QLabel(tr_ui("速度:", "Velocity:")));
    h_vel->addWidget(spin_arm_velocity_);

    spin_arm_accel_ = new QDoubleSpinBox();
    spin_arm_accel_->setRange(0.1, 360.0);
    spin_arm_accel_->setValue(60.0);
    spin_arm_accel_->setDecimals(1);
    spin_arm_accel_->setSingleStep(10.0);
    spin_arm_accel_->setSuffix(QString::fromUtf8(" °/s²"));
    h_vel->addWidget(new QLabel(tr_ui("加速度:", "Accel:")));
    h_vel->addWidget(spin_arm_accel_);
    arm_layout->addLayout(h_vel);

    connect(combo_arm_command_, &QComboBox::currentTextChanged, this, &StepAddDialog::onArmCommandChanged);
    onArmCommandChanged(combo_arm_command_->currentText());

    widget_hand_ = new QWidget();
    auto * hand_layout = new QFormLayout(widget_hand_);
    for(int i=0; i<6; ++i) {
        spin_hand_pos_[i] = new QSpinBox();
        spin_hand_pos_[i]->setRange(0, 100000);
        hand_layout->addRow(tr_ui("手指 %1", "Finger %1").arg(i+1), spin_hand_pos_[i]);
    }

    widget_camera_ = new QWidget();
    auto * cam_layout = new QFormLayout(widget_camera_);
    combo_camera_type_ = new QComboBox();
    combo_camera_type_->addItem(tr_ui("彩色 (color)", "Color"), "color");
    combo_camera_type_->addItem(tr_ui("深度 (depth)", "Depth"), "depth");
    combo_camera_type_->addItem(tr_ui("左红外 (ir_left)", "IR Left"), "ir_left");
    combo_camera_type_->addItem(tr_ui("右红外 (ir_right)", "IR Right"), "ir_right");
    combo_camera_type_->addItem(tr_ui("点云 (point_cloud)", "Point Cloud"), "point_cloud");
    combo_camera_type_->addItem(tr_ui("深度测量 (depth_measure)", "Depth Measure"), "depth_measure");
    combo_camera_type_->addItem(tr_ui("耳机基准 (vision_baseline)", "Earphone Baseline"), "vision_baseline");
    combo_camera_type_->addItem(tr_ui("耳机测量 (vision_measure)", "Earphone Measure"), "vision_measure");
    cam_layout->addRow(tr_ui("相机类型:", "Camera Type:"), combo_camera_type_);

    widget_io_ = new QWidget();
    auto * io_layout = new QFormLayout(widget_io_);
    combo_io_group_ = new QComboBox();
    combo_io_group_->addItem(tr_ui("标准 DIO (1-8)", "Standard DIO (1-8)"), 0);   // type=0, base=1
    combo_io_group_->addItem(tr_ui("标准 DIO (9-16)", "Standard DIO (9-16)"), 1);  // type=0, base=9
    combo_io_group_->addItem(tr_ui("工具 IO (1-2)", "Tool IO (1-2)"), 2);    // type=1, base=1
    io_layout->addRow(tr_ui("IO 分组:", "IO Group:"), combo_io_group_);

    combo_io_port_ = new QComboBox();
    io_layout->addRow(tr_ui("IO 端口:", "IO Port:"), combo_io_port_);

    combo_io_value_ = new QComboBox();
    combo_io_value_->addItem(tr_ui("开启 (HIGH)", "Open (HIGH)"), true);
    combo_io_value_->addItem(tr_ui("关闭 (LOW)", "Close (LOW)"), false);
    io_layout->addRow(tr_ui("电平:", "Level:"), combo_io_value_);

    connect(combo_io_group_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &StepAddDialog::onIOGroupChanged);
    onIOGroupChanged(0);

    widget_lift_ = new QWidget();
    auto * lift_layout = new QFormLayout(widget_lift_);
    combo_lift_command_ = new QComboBox();
    combo_lift_command_->addItem(tr_ui("上升 (move_up)", "Up (move_up)"), "move_up");
    combo_lift_command_->addItem(tr_ui("下降 (move_down)", "Down (move_down)"), "move_down");
    combo_lift_command_->addItem(tr_ui("停止 (stop)", "Stop (stop)"), "stop");
    combo_lift_command_->addItem(tr_ui("定位移动 (position_move)", "Position Move"), "position_move");
    combo_lift_command_->addItem(tr_ui("回原点 (position_next)", "Home (position_next)"), "position_next");
    combo_lift_command_->addItem(tr_ui("定位停止 (position_stop)", "Position Stop"), "position_stop");
    lift_layout->addRow(tr_ui("升降命令:", "Lift Command:"), combo_lift_command_);
    spin_lift_speed_rpm_ = new QSpinBox();
    spin_lift_speed_rpm_->setRange(1, 65535);
    spin_lift_speed_rpm_->setValue(1000);
    spin_lift_speed_rpm_->setSingleStep(100);
    lift_layout->addRow(tr_ui("速度:", "Speed:"), spin_lift_speed_rpm_);
    spin_lift_target_pulses_ = new QSpinBox();
    spin_lift_target_pulses_->setRange(-99999999, 99999999);
    spin_lift_target_pulses_->setValue(10000);
    spin_lift_target_pulses_->setSingleStep(1000);
    lift_layout->addRow(tr_ui("目标脉冲:", "Target Pulses:"), spin_lift_target_pulses_);
    spin_lift_accel_ms_ = new QSpinBox();
    spin_lift_accel_ms_->setRange(0, 65535);
    spin_lift_accel_ms_->setValue(1000);
    lift_layout->addRow(tr_ui("加速时间(ms):", "Accel Time(ms):"), spin_lift_accel_ms_);
    spin_lift_decel_ms_ = new QSpinBox();
    spin_lift_decel_ms_->setRange(0, 65535);
    spin_lift_decel_ms_->setValue(1000);
    lift_layout->addRow(tr_ui("减速时间(ms):", "Decel Time(ms):"), spin_lift_decel_ms_);

    stack->addWidget(widget_arm_);
    stack->addWidget(widget_hand_);
    stack->addWidget(widget_camera_);
    stack->addWidget(widget_io_);
    stack->addWidget(widget_lift_);

    // --- Control step widget ---
    widget_control_ = new QWidget();
    auto * ctrl_layout = new QFormLayout(widget_control_);
    combo_control_target_ = new QComboBox();
    combo_control_target_->addItem(tr_ui("机械臂 (arm)", "Arm (arm)"), "arm");
    combo_control_target_->addItem(tr_ui("左手 (lhand)", "Left Hand (lhand)"), "lhand");
    combo_control_target_->addItem(tr_ui("右手 (rhand)", "Right Hand (rhand)"), "rhand");
    combo_control_target_->addItem(tr_ui("升降台 (lift)", "Lift (lift)"), "lift");
    ctrl_layout->addRow(tr_ui("控制目标:", "Target:"), combo_control_target_);
    combo_control_command_ = new QComboBox();
    ctrl_layout->addRow(tr_ui("控制指令:", "Command:"), combo_control_command_);
    stack->addWidget(widget_control_);

    connect(combo_control_target_, &QComboBox::currentTextChanged,
            this, &StepAddDialog::onControlTargetChanged);
    onControlTargetChanged(combo_control_target_->currentText());
    layout->addWidget(stack);

    auto * h_delay = new QHBoxLayout();
    h_delay->addWidget(new QLabel(tr_ui("步骤后延时 (ms):", "Delay after step (ms):")));
    spin_delay_ms_ = new QSpinBox();
    spin_delay_ms_->setRange(0, 3600000); // Up to 1 hour
    spin_delay_ms_->setValue(0);
    h_delay->addWidget(spin_delay_ms_);
    layout->addLayout(h_delay);

    auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);

    connect(combo_type_, &QComboBox::currentTextChanged, this, &StepAddDialog::onTypeChanged);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    onTypeChanged(combo_type_->currentText());
}

void StepAddDialog::onTypeChanged(const QString& type) {
    combo_device_->clear();
    if (type == "camera") {
        auto cams = node_->scan_cameras();
        for (const auto& cam : cams) {
            combo_device_->addItem(QString::fromStdString(cam), QString::fromStdString(cam));
        }
    } else {
        // arm 和 io 都属于 duco 机械臂设备
        std::string target_type;
        if (type == "arm" || type == "io") {
            target_type = "duco";
        } else {
            target_type = type.toStdString();
        }

        for (const auto& dev : devices_) {
            if (dev.device_type == target_type) {
                QString label = QString::fromStdString(dev.device_name.empty() ? dev.device_sn : dev.device_name);
                combo_device_->addItem(label, QString::fromStdString(dev.device_sn));
            }
        }

        // fallback: 任务设备列表中没有时从已连接设备扫描
        if (combo_device_->count() == 0) {
            auto connected = node_->get_connected_devices();
            for (const auto& dev : connected) {
                if (dev.device_type == target_type) {
                    QString label = QString::fromStdString(dev.device_model.empty() ? dev.device_name : dev.device_model);
                    if (!dev.device_sn.empty()) label += " (" + QString::fromStdString(dev.device_sn) + ")";
                    combo_device_->addItem(label, QString::fromStdString(dev.device_sn));
                }
            }
        }
    }

    auto * stack = this->findChild<QStackedWidget*>();
    if (!stack) return;
    if (type == "arm") stack->setCurrentWidget(widget_arm_);
    else if (type == "lhand" || type == "rhand") stack->setCurrentWidget(widget_hand_);
    else if (type == "io") stack->setCurrentWidget(widget_io_);
    else if (type == "lift") stack->setCurrentWidget(widget_lift_);
    else if (type == "control") stack->setCurrentWidget(widget_control_);
    else stack->setCurrentWidget(widget_camera_);
}

void StepAddDialog::onIOGroupChanged(int index) {
    combo_io_port_->clear();
    static const QString std_names[] = {
        tr_ui("清洗机", "Washer"), tr_ui("左侧吹风机", "Left Blower"),
        tr_ui("右侧烘干机", "Right Dryer"), tr_ui("升降平台解锁", "Lift Unlock"),
        tr_ui("黄灯", "Yellow Light"), tr_ui("绿灯", "Green Light"),
        tr_ui("红灯", "Red Light"), tr_ui("蜂鸣器", "Buzzer"),
    };
    if (index == 0) { // Standard 1-8
        for (int i = 0; i < 8; ++i)
            combo_io_port_->addItem(QString("%1 - %2").arg(i + 1).arg(std_names[i]), i + 1);
    } else if (index == 1) { // Standard 9-16
        for (int i = 0; i < 8; ++i)
            combo_io_port_->addItem(QString("DIO %1").arg(i + 9), i + 9);
    } else if (index == 2) { // Tool 1-2
        combo_io_port_->addItem(QString::fromUtf8("Tool IO 1"), 1);
        combo_io_port_->addItem(QString::fromUtf8("Tool IO 2"), 2);
    }
}

void StepAddDialog::onControlTargetChanged(const QString& /*target*/) {
    combo_control_command_->clear();
    QString t = combo_control_target_->currentData().toString();
    if (t == "arm") {
        combo_control_command_->addItem(tr_ui("上电 (poweron)",  "Power On"),   "poweron");
        combo_control_command_->addItem(tr_ui("使能 (enable)",   "Enable"),     "enable");
        combo_control_command_->addItem(tr_ui("断使能 (disable)","Disable"),    "disable");
        combo_control_command_->addItem(tr_ui("下电 (poweroff)", "Power Off"),  "poweroff");
    } else if (t == "lhand" || t == "rhand") {
        combo_control_command_->addItem(tr_ui("使能 (enable)",   "Enable"),     "enable");
        combo_control_command_->addItem(tr_ui("断使能 (disable)","Disable"),    "disable");
        combo_control_command_->addItem(tr_ui("回零 (home)",     "Home"),       "home");
    } else if (t == "lift") {
        combo_control_command_->addItem(tr_ui("使能 (enable)",   "Enable"),     "enable");
        combo_control_command_->addItem(tr_ui("断使能 (disable)","Disable"),    "disable");
    }
}

void StepAddDialog::onArmCommandChanged(const QString& /*cmd*/) {
    bool is_movej = (combo_arm_command_->currentData().toString() == "movej");
    widget_arm_movej_->setVisible(is_movej);
    widget_arm_movel_->setVisible(!is_movej);
    if (is_movej) {
        spin_arm_velocity_->setRange(0.1, 180.0);
        spin_arm_velocity_->setSuffix(QString::fromUtf8(" °/s"));
        spin_arm_accel_->setRange(0.1, 360.0);
        spin_arm_accel_->setSuffix(QString::fromUtf8(" °/s²"));
    } else {
        spin_arm_velocity_->setRange(0.1, 500.0);
        spin_arm_velocity_->setSuffix(tr_ui(" mm/s", " mm/s"));
        spin_arm_accel_->setRange(0.1, 2000.0);
        spin_arm_accel_->setSuffix(QString::fromUtf8(" mm/s²"));
        spin_arm_velocity_->setValue(100.0);
        spin_arm_accel_->setValue(500.0);
    }
}

void StepAddDialog::onCaptureCurrent() {
    std::lock_guard<std::mutex> lock(node_->data_mutex_);
    // current_joints_ stores radians; display as degrees
    if (node_->current_joints_.size() >= 6) {
        for (int i = 0; i < 6; ++i) {
            spin_arm_pos_[i]->setValue(node_->current_joints_[i] * 180.0 / M_PI);
        }
    }
}

void StepAddDialog::setStep(const common_msgs::msg::TaskStep& step) {
    edit_name_->setText(QString::fromStdString(step.name));

    // Set combo_type_ which triggers onTypeChanged (refreshes device list & stacked widget)
    int type_idx = combo_type_->findText(QString::fromStdString(step.type));
    if (type_idx >= 0) {
        combo_type_->setCurrentIndex(type_idx);
    }

    // Try to match device_sn in combo_device_
    if (!step.device_sn.empty()) {
        QString sn = QString::fromStdString(step.device_sn);
        // First try matching by data (sn stored in itemData)
        for (int i = 0; i < combo_device_->count(); ++i) {
            if (combo_device_->itemData(i).toString() == sn ||
                combo_device_->itemText(i) == sn) {
                combo_device_->setCurrentIndex(i);
                break;
            }
        }
    }

    spin_delay_ms_->setValue(step.delay_ms);

    // Fill type-specific fields
    if (step.type == "arm") {
        std::string cmd = step.arm_command.empty() ? "movej" : step.arm_command;
        int cmd_idx = combo_arm_command_->findData(QString::fromStdString(cmd));
        if (cmd_idx >= 0) combo_arm_command_->setCurrentIndex(cmd_idx);
        onArmCommandChanged(combo_arm_command_->currentText());

        if (cmd == "movej") {
            int n = std::min(static_cast<int>(step.arm_pos.size()), 6);
            for (int i = 0; i < n; ++i)
                spin_arm_pos_[i]->setValue(step.arm_pos[i]);
        } else {
            int n = std::min(static_cast<int>(step.arm_cart_pos.size()), 6);
            for (int i = 0; i < n; ++i)
                spin_arm_cart_[i]->setValue(step.arm_cart_pos[i]);
        }
        double vel = step.arm_velocity > 0 ? step.arm_velocity : (cmd == "movej" ? 30.0 : 100.0);
        double acc = step.arm_accel > 0 ? step.arm_accel : (cmd == "movej" ? 60.0 : 500.0);
        spin_arm_velocity_->setValue(vel);
        spin_arm_accel_->setValue(acc);
    } else if (step.type == "lhand" || step.type == "rhand") {
        int n = std::min(static_cast<int>(step.hand_pos.size()), 6);
        for (int i = 0; i < n; ++i) {
            spin_hand_pos_[i]->setValue(static_cast<int>(step.hand_pos[i]));
        }
    } else if (step.type == "camera") {
        if (!step.camera_type.empty()) {
            QString type_str = QString::fromStdString(step.camera_type[0]);
            int ct_idx = combo_camera_type_->findData(type_str);
            if (ct_idx < 0) ct_idx = combo_camera_type_->findText(type_str);
            if (ct_idx >= 0) combo_camera_type_->setCurrentIndex(ct_idx);
        }
    } else if (step.type == "io") {
        // Determine group from io_type and io_port
        int group = 0;
        if (step.io_type == 1) {
            group = 2; // Tool IO
        } else if (step.io_port >= 9) {
            group = 1; // Standard 9-16
        }
        combo_io_group_->setCurrentIndex(group);
        // Find port in combo
        for (int i = 0; i < combo_io_port_->count(); ++i) {
            if (combo_io_port_->itemData(i).toInt() == step.io_port) {
                combo_io_port_->setCurrentIndex(i);
                break;
            }
        }
        combo_io_value_->setCurrentIndex(step.io_value ? 0 : 1);
    } else if (step.type == "lift") {
        for (int i = 0; i < combo_lift_command_->count(); ++i) {
            if (combo_lift_command_->itemData(i).toString().toStdString() == step.lift_command) {
                combo_lift_command_->setCurrentIndex(i);
                break;
            }
        }
        spin_lift_speed_rpm_->setValue(step.lift_speed_rpm > 0 ? step.lift_speed_rpm : 1000);
        spin_lift_target_pulses_->setValue(step.lift_target_pulses);
        spin_lift_accel_ms_->setValue(step.lift_accel_ms > 0 ? step.lift_accel_ms : 1000);
        spin_lift_decel_ms_->setValue(step.lift_decel_ms > 0 ? step.lift_decel_ms : 1000);
    } else if (step.type == "control") {
        for (int i = 0; i < combo_control_target_->count(); ++i) {
            if (combo_control_target_->itemData(i).toString().toStdString() == step.control_target) {
                combo_control_target_->setCurrentIndex(i);
                break;
            }
        }
        onControlTargetChanged(combo_control_target_->currentText());
        for (int i = 0; i < combo_control_command_->count(); ++i) {
            if (combo_control_command_->itemData(i).toString().toStdString() == step.control_command) {
                combo_control_command_->setCurrentIndex(i);
                break;
            }
        }
    }
}

common_msgs::msg::TaskStep StepAddDialog::getStep() const {
    common_msgs::msg::TaskStep step;
    step.name = edit_name_->text().toStdString();
    step.type = combo_type_->currentText().toStdString();
    step.device_sn = combo_device_->currentData().toString().toStdString();
    if (step.device_sn.empty()) step.device_sn = combo_device_->currentText().toStdString();

    step.delay_ms = spin_delay_ms_->value();

    if (step.type == "arm") {
        step.arm_command = combo_arm_command_->currentData().toString().toStdString();
        step.arm_velocity = spin_arm_velocity_->value();
        step.arm_accel = spin_arm_accel_->value();
        step.arm_pos.clear();
        step.arm_cart_pos.clear();
        if (step.arm_command == "movej") {
            for (int i = 0; i < 6; ++i) step.arm_pos.push_back(spin_arm_pos_[i]->value());
        } else {
            for (int i = 0; i < 6; ++i) step.arm_cart_pos.push_back(spin_arm_cart_[i]->value());
        }
    } else if (step.type == "lhand" || step.type == "rhand") {
        step.hand_pos.clear();
        for(int i=0; i<6; ++i) step.hand_pos.push_back(spin_hand_pos_[i]->value());
    } else if (step.type == "camera") {
        step.camera_type.clear();
        step.camera_type.push_back(combo_camera_type_->currentData().toString().toStdString());
    } else if (step.type == "io") {
        int group = combo_io_group_->currentIndex();
        step.io_type = (group == 2) ? 1 : 0; // 0=standard, 1=tool
        step.io_port = static_cast<int8_t>(combo_io_port_->currentData().toInt());
        step.io_value = combo_io_value_->currentData().toBool();
    } else if (step.type == "lift") {
        step.lift_command = combo_lift_command_->currentData().toString().toStdString();
        step.lift_speed_rpm = spin_lift_speed_rpm_->value();
        step.lift_target_pulses = spin_lift_target_pulses_->value();
        step.lift_accel_ms = spin_lift_accel_ms_->value();
        step.lift_decel_ms = spin_lift_decel_ms_->value();
    } else if (step.type == "control") {
        step.control_target  = combo_control_target_->currentData().toString().toStdString();
        step.control_command = combo_control_command_->currentData().toString().toStdString();
    }
    return step;
}

TaskConfigDialog::TaskConfigDialog(std::shared_ptr<RosNode> node, QWidget *parent)
    : QDialog(parent), node_(node) {
    setWindowTitle(tr_ui("任务配置", "Task Configuration"));
    auto * layout = new QVBoxLayout(this);

    auto * h_name = new QHBoxLayout();
    h_name->addWidget(new QLabel(tr_ui("任务名称:", "Task Name:")));
    edit_name_ = new QLineEdit();
    h_name->addWidget(edit_name_);
    layout->addLayout(h_name);

    auto * h_rounds = new QHBoxLayout();
    h_rounds->addWidget(new QLabel(tr_ui("执行轮次:", "Execution Rounds:")));
    spin_rounds_ = new QSpinBox();
    spin_rounds_->setRange(1, 999);
    spin_rounds_->setValue(1);
    h_rounds->addWidget(spin_rounds_);
    layout->addLayout(h_rounds);

    layout->addWidget(new QLabel(tr_ui("设备:", "Devices:")));
    table_devices_ = new QTableWidget();
    table_devices_->setColumnCount(4);
    table_devices_->setHorizontalHeaderLabels({tr_ui("类型", "Type"), tr_ui("名称", "Name"), tr_ui("SN码", "SN"), tr_ui("用途", "Usage")});
    table_devices_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    layout->addWidget(table_devices_);

    auto * h_dev_btn = new QHBoxLayout();
    auto * btn_add_dev = new QPushButton(tr_ui("添加设备", "Add Device"));
    auto * btn_del_dev = new QPushButton(tr_ui("删除设备", "Delete Device"));
    h_dev_btn->addWidget(btn_add_dev);
    h_dev_btn->addWidget(btn_del_dev);
    layout->addLayout(h_dev_btn);

    layout->addWidget(new QLabel(tr_ui("步骤:", "Steps:")));
    list_steps_ = new QListWidget();
    list_steps_->setDragDropMode(QAbstractItemView::InternalMove);
    list_steps_->setDefaultDropAction(Qt::MoveAction);
    layout->addWidget(list_steps_);

    // Sync steps_ vector when drag-drop reorders items
    connect(list_steps_->model(), &QAbstractItemModel::rowsMoved,
            this, [this](const QModelIndex&, int start, int /*end*/, const QModelIndex&, int dest) {
        if (start < 0 || start >= static_cast<int>(steps_.size())) return;
        int to = (dest > start) ? dest - 1 : dest;
        auto step = steps_[start];
        steps_.erase(steps_.begin() + start);
        steps_.insert(steps_.begin() + to, step);
        // Update numbering in-place (no clear/rebuild during drag!)
        for (int i = 0; i < list_steps_->count(); ++i) {
            auto * item = list_steps_->item(i);
            int idx = i;
            if (idx < static_cast<int>(steps_.size())) {
                item->setText(QString("%1. [%2] %3")
                    .arg(idx + 1)
                    .arg(QString::fromStdString(steps_[idx].type))
                    .arg(QString::fromStdString(steps_[idx].name)));
                item->setData(Qt::UserRole, idx);
            }
        }
    });

    auto * h_step_btn = new QHBoxLayout();
    auto * btn_add_step = new QPushButton(tr_ui("添加步骤", "Add Step"));
    auto * btn_edit_step = new QPushButton(tr_ui("编辑步骤", "Edit Step"));
    auto * btn_del_step = new QPushButton(tr_ui("删除步骤", "Delete Step"));
    auto * btn_import_step = new QPushButton(tr_ui("从其他任务导入", "Import from Task"));
    auto * btn_import_file = new QPushButton(tr_ui("从文件导入", "Import from File"));
    auto * btn_export_step = new QPushButton(tr_ui("导出步骤", "Export Steps"));
    h_step_btn->addWidget(btn_add_step);
    h_step_btn->addWidget(btn_edit_step);
    h_step_btn->addWidget(btn_del_step);
    h_step_btn->addWidget(btn_import_step);
    h_step_btn->addWidget(btn_import_file);
    h_step_btn->addWidget(btn_export_step);
    layout->addLayout(h_step_btn);

    auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);

    connect(btn_add_dev, &QPushButton::clicked, this, &TaskConfigDialog::onAddDevice);
    connect(btn_del_dev, &QPushButton::clicked, this, &TaskConfigDialog::onDeleteDevice);
    connect(btn_add_step, &QPushButton::clicked, this, &TaskConfigDialog::onAddStep);
    connect(btn_edit_step, &QPushButton::clicked, this, &TaskConfigDialog::onEditStep);
    connect(btn_del_step, &QPushButton::clicked, this, &TaskConfigDialog::onDeleteStep);
    connect(btn_import_step, &QPushButton::clicked, this, &TaskConfigDialog::onImportSteps);
    connect(btn_import_file, &QPushButton::clicked, this, &TaskConfigDialog::onImportFromFile);
    connect(btn_export_step, &QPushButton::clicked, this, &TaskConfigDialog::onExportSteps);
    connect(list_steps_, &QListWidget::itemDoubleClicked, this, &TaskConfigDialog::onEditStep);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
}

void TaskConfigDialog::setTask(const common_msgs::msg::TaskConfig& task) {
    edit_name_->setText(QString::fromStdString(task.task_name));
    int rounds = task.exec_rounds > 0 ? task.exec_rounds : 1;
    spin_rounds_->setValue(rounds);
    devices_ = task.device_checks;
    steps_ = task.task_seqs;
    updateDeviceTable();
    updateStepList();
}

common_msgs::msg::TaskConfig TaskConfigDialog::getTask() const {
    common_msgs::msg::TaskConfig task;
    task.task_name = edit_name_->text().toStdString();
    task.exec_rounds = spin_rounds_->value();
    task.device_checks = devices_;
    task.task_seqs = steps_;
    return task;
}

void TaskConfigDialog::updateDeviceTable() {
    table_devices_->setRowCount(static_cast<int>(devices_.size()));
    for (int i = 0; i < (int)devices_.size(); ++i) {
        const auto& d = devices_[i];
        table_devices_->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(d.device_type)));
        table_devices_->setItem(i, 1, new QTableWidgetItem(QString::fromStdString(d.device_name)));
        table_devices_->setItem(i, 2, new QTableWidgetItem(QString::fromStdString(d.device_sn)));
        table_devices_->setItem(i, 3, new QTableWidgetItem(QString::fromStdString(d.device_usage)));
    }
}

void TaskConfigDialog::updateStepList() {
    list_steps_->clear();
    for (size_t i = 0; i < steps_.size(); ++i) {
        const auto& s = steps_[i];
        QString label = QString("%1. [%2] %3")
            .arg(i + 1)
            .arg(QString::fromStdString(s.type))
            .arg(QString::fromStdString(s.name));
        auto * item = new QListWidgetItem(label);
        item->setData(Qt::UserRole, static_cast<int>(i));
        list_steps_->addItem(item);
    }
}

void TaskConfigDialog::onAddDevice() {
    DeviceAddDialog dlg(node_, this);
    if (dlg.exec() == QDialog::Accepted) {
        devices_.push_back(dlg.getDeviceCheck());
        updateDeviceTable();
    }
}

void TaskConfigDialog::onDeleteDevice() {
    int row = table_devices_->currentRow();
    if (row >= 0 && row < (int)devices_.size()) {
        devices_.erase(devices_.begin() + row);
        updateDeviceTable();
    }
}

void TaskConfigDialog::onAddStep() {
    StepAddDialog dlg(node_, devices_, this);
    if (dlg.exec() == QDialog::Accepted) {
        steps_.push_back(dlg.getStep());
        updateStepList();
    }
}

void TaskConfigDialog::onEditStep() {
    int row = list_steps_->currentRow();
    if (row < 0 || row >= static_cast<int>(steps_.size())) {
        QMessageBox::information(this, tr_ui("编辑步骤", "Edit Step"), tr_ui("请选择要编辑的步骤。", "Please select a step to edit."));
        return;
    }
    StepAddDialog dlg(node_, devices_, this);
    dlg.setWindowTitle(tr_ui("编辑步骤", "Edit Step"));
    dlg.setStep(steps_[row]);
    if (dlg.exec() == QDialog::Accepted) {
        steps_[row] = dlg.getStep();
        updateStepList();
    }
}

void TaskConfigDialog::onDeleteStep() {
    int row = list_steps_->currentRow();
    if (row >= 0 && row < (int)steps_.size()) {
        steps_.erase(steps_.begin() + row);
        updateStepList();
    }
}

void TaskConfigDialog::setAllTasks(const std::vector<common_msgs::msg::TaskConfig>& tasks) {
    all_tasks_ = tasks;
}

void TaskConfigDialog::onImportSteps() {
    if (all_tasks_.empty()) {
        QMessageBox::information(this, tr_ui("导入步骤", "Import Steps"),
                                 tr_ui("没有其他任务可导入", "No other tasks available"));
        return;
    }

    QDialog dlg(this);
    dlg.setWindowTitle(tr_ui("从其他任务导入步骤", "Import Steps from Task"));
    dlg.resize(500, 400);
    auto * lay = new QVBoxLayout(&dlg);

    lay->addWidget(new QLabel(tr_ui("选择任务:", "Select Task:")));
    auto * combo_task = new QComboBox();
    for (const auto& t : all_tasks_) {
        combo_task->addItem(QString::fromStdString(t.task_name));
    }
    lay->addWidget(combo_task);

    lay->addWidget(new QLabel(tr_ui("选择步骤 (可多选):", "Select Steps (multi-select):")));
    auto * list = new QListWidget();
    list->setSelectionMode(QAbstractItemView::MultiSelection);
    lay->addWidget(list);

    auto refreshSteps = [&]() {
        list->clear();
        int idx = combo_task->currentIndex();
        if (idx < 0 || idx >= static_cast<int>(all_tasks_.size())) return;
        for (const auto& s : all_tasks_[idx].task_seqs) {
            QString label = QString("[%1] %2").arg(QString::fromStdString(s.type),
                                                    QString::fromStdString(s.name));
            list->addItem(label);
        }
    };
    connect(combo_task, QOverload<int>::of(&QComboBox::currentIndexChanged),
            &dlg, refreshSteps);
    refreshSteps();

    auto * btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    lay->addWidget(btns);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if (dlg.exec() != QDialog::Accepted) return;

    int task_idx = combo_task->currentIndex();
    if (task_idx < 0 || task_idx >= static_cast<int>(all_tasks_.size())) return;

    const auto& src_steps = all_tasks_[task_idx].task_seqs;
    int imported = 0;
    for (int i = 0; i < list->count(); ++i) {
        if (list->item(i)->isSelected() && i < static_cast<int>(src_steps.size())) {
            steps_.push_back(src_steps[i]);
            ++imported;
        }
    }
    if (imported > 0) updateStepList();
}

QJsonObject TaskConfigDialog::stepToJson(const common_msgs::msg::TaskStep& step) {
    QJsonObject obj;
    obj["step_name"] = QString::fromStdString(step.name);
    obj["step_type"] = QString::fromStdString(step.type);
    obj["device_sn"] = QString::fromStdString(step.device_sn);
    obj["arm_command"] = QString::fromStdString(step.arm_command);
    QJsonArray ap; for (double v : step.arm_pos) ap.append(v);
    obj["arm_pos"] = ap;
    QJsonArray ac; for (double v : step.arm_cart_pos) ac.append(v);
    obj["arm_cart_pos"] = ac;
    obj["arm_velocity"] = step.arm_velocity;
    obj["arm_accel"] = step.arm_accel;
    QJsonArray hp; for (int v : step.hand_pos) hp.append(v);
    obj["hand_pos"] = hp;
    QJsonArray ct; for (const auto& c : step.camera_type) ct.append(QString::fromStdString(c));
    obj["camera_type"] = ct;
    obj["io_type"] = step.io_type;
    obj["io_port"] = step.io_port;
    obj["io_value"] = step.io_value;
    obj["delay_ms"] = step.delay_ms;
    obj["lift_command"] = QString::fromStdString(step.lift_command);
    obj["lift_speed_rpm"] = step.lift_speed_rpm;
    obj["lift_target_pulses"] = step.lift_target_pulses;
    obj["lift_accel_ms"] = step.lift_accel_ms;
    obj["lift_decel_ms"] = step.lift_decel_ms;
    obj["control_target"] = QString::fromStdString(step.control_target);
    obj["control_command"] = QString::fromStdString(step.control_command);
    return obj;
}

common_msgs::msg::TaskStep TaskConfigDialog::stepFromJson(const QJsonObject& obj) {
    common_msgs::msg::TaskStep step;
    step.name = obj["step_name"].toString().toStdString();
    step.type = obj["step_type"].toString().toStdString();
    step.device_sn = obj["device_sn"].toString().toStdString();
    step.arm_command = obj["arm_command"].toString().toStdString();
    for (const auto& v : obj["arm_pos"].toArray()) step.arm_pos.push_back(v.toDouble());
    for (const auto& v : obj["arm_cart_pos"].toArray()) step.arm_cart_pos.push_back(v.toDouble());
    step.arm_velocity = obj["arm_velocity"].toDouble(0);
    step.arm_accel = obj["arm_accel"].toDouble(0);
    for (const auto& v : obj["hand_pos"].toArray()) step.hand_pos.push_back(v.toInt());
    if (obj["camera_type"].isArray()) {
        for (const auto& v : obj["camera_type"].toArray()) step.camera_type.push_back(v.toString().toStdString());
    }
    step.io_type = static_cast<int8_t>(obj["io_type"].toInt(0));
    step.io_port = static_cast<int8_t>(obj["io_port"].toInt(0));
    step.io_value = obj["io_value"].toBool(false);
    step.delay_ms = obj["delay_ms"].toInt(0);
    step.lift_command = obj["lift_command"].toString().toStdString();
    step.lift_speed_rpm = obj["lift_speed_rpm"].toInt(0);
    step.lift_target_pulses = obj["lift_target_pulses"].toInt(0);
    step.lift_accel_ms = obj["lift_accel_ms"].toInt(0);
    step.lift_decel_ms = obj["lift_decel_ms"].toInt(0);
    step.control_target = obj["control_target"].toString().toStdString();
    step.control_command = obj["control_command"].toString().toStdString();
    return step;
}

void TaskConfigDialog::onExportSteps() {
    if (steps_.empty()) {
        QMessageBox::information(this, tr_ui("导出步骤", "Export Steps"),
                                 tr_ui("当前任务没有步骤可导出", "No steps to export"));
        return;
    }
    QString path = QFileDialog::getSaveFileName(
        this, tr_ui("导出步骤", "Export Steps"), QString(),
        tr_ui("步骤文件 (*.steps.json)", "Step Files (*.steps.json)"));
    if (path.isEmpty()) return;

    QJsonObject root;
    root["source_task"] = edit_name_->text();
    QJsonArray arr;
    for (const auto& s : steps_) arr.append(stepToJson(s));
    root["steps"] = arr;

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        QMessageBox::warning(this, tr_ui("导出失败", "Export Failed"),
                             tr_ui("无法写入文件", "Cannot write file"));
        return;
    }
    file.write(QJsonDocument(root).toJson());
    file.close();
}

void TaskConfigDialog::onImportFromFile() {
    QString path = QFileDialog::getOpenFileName(
        this, tr_ui("导入步骤", "Import Steps"), QString(),
        tr_ui("步骤文件 (*.steps.json)", "Step Files (*.steps.json)"));
    if (path.isEmpty()) return;

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(this, tr_ui("导入失败", "Import Failed"),
                             tr_ui("无法读取文件", "Cannot read file"));
        return;
    }
    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    file.close();

    QJsonArray arr = doc.object()["steps"].toArray();
    int imported = 0;
    for (const auto& v : arr) {
        steps_.push_back(stepFromJson(v.toObject()));
        ++imported;
    }
    if (imported > 0) {
        updateStepList();
        QMessageBox::information(this, tr_ui("导入成功", "Import Success"),
            tr_ui("已导入 %1 个步骤", "Imported %1 steps").arg(imported));
    }
}

TaskRunDialog::TaskRunDialog(std::shared_ptr<RosNode> node, const common_msgs::msg::TaskConfig& task, std::shared_ptr<TaskRecordManager> record_manager, QWidget *parent)
    : QDialog(parent), node_(node), record_manager_(record_manager), task_(task), devices_ready_(false) {
    setWindowTitle(tr_ui("运行任务", "Run Task"));
    setWindowFlags(windowFlags() | Qt::WindowMinMaxButtonsHint | Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint);
    
    // 强制模态以防止点击后面内容，但允许系统操作
    setWindowModality(Qt::ApplicationModal);
    
    auto * layout = new QVBoxLayout(this);

    label_status_ = new QLabel(tr_ui("检查设备中...", "Checking devices..."));
    layout->addWidget(label_status_);

    progress_steps_ = new QProgressBar();
    int rounds = task_.exec_rounds > 0 ? task_.exec_rounds : 1;
    int total_steps = static_cast<int>(task_.task_seqs.size()) * rounds;
    progress_steps_->setRange(0, total_steps);
    progress_steps_->setValue(0);
    layout->addWidget(progress_steps_);

    auto * h_btn = new QHBoxLayout();
    btn_start_ = new QPushButton(tr_ui("开始", "Start"));
    btn_pause_ = new QPushButton(tr_ui("暂停", "Pause"));
    btn_stop_ = new QPushButton(tr_ui("停止", "Stop"));
    auto * btn_edit = new QPushButton(tr_ui("编辑", "Edit"));
    h_btn->addWidget(btn_start_);
    h_btn->addWidget(btn_pause_);
    h_btn->addWidget(btn_stop_);
    h_btn->addWidget(btn_edit);
    layout->addLayout(h_btn);

    btn_start_->setEnabled(false);
    btn_pause_->setEnabled(false);

    connect(btn_start_, &QPushButton::clicked, this, &TaskRunDialog::onStart);
    connect(btn_pause_, &QPushButton::clicked, this, &TaskRunDialog::onPause);
    connect(btn_stop_, &QPushButton::clicked, this, &TaskRunDialog::onStop);
    connect(btn_edit, &QPushButton::clicked, this, &TaskRunDialog::onEdit);

    QTimer::singleShot(500, this, &TaskRunDialog::checkDevices);
}

void TaskRunDialog::checkDevices() {
    bool all_ok = true;
    QString missing;

    if (!node_->is_task_action_ready()) {
        all_ok = false;
        missing += "system_controller ";
    }

    for (const auto& check : task_.device_checks) {
        if (!node_->check_device_availability(check)) {
            all_ok = false;
            missing += QString::fromStdString(check.device_type) + " ";
        }
    }

    if (all_ok) {
        label_status_->setText(tr_ui("设备检查: 全部就绪", "Devices Checked: All Ready"));
        btn_start_->setEnabled(true);
        devices_ready_ = true;
    } else {
        label_status_->setText(tr_ui("缺失设备: ", "Missing Devices: ") + missing);
        btn_start_->setEnabled(false);
        devices_ready_ = false;
    }
}

void TaskRunDialog::onStart() {
    if (!devices_ready_) return;
    btn_start_->setEnabled(false);
    btn_pause_->setEnabled(true);
    label_status_->setText(tr_ui("运行中...", "Running..."));

    current_record_ = TaskExecutionRecord();
    current_record_.task_name = QString::fromStdString(task_.task_name);
    current_record_.start_time = QDateTime::currentDateTimeUtc();
    current_record_.success = false;

    node_->call_execute_task(task_,
        std::bind(&TaskRunDialog::onTaskResult, this, std::placeholders::_1),
        std::bind(&TaskRunDialog::onTaskFeedback, this, std::placeholders::_1)
    );
}

void TaskRunDialog::onPause() {
    if (btn_pause_->text() == tr_ui("暂停", "Pause")) {
        node_->call_pause_task(true);
        btn_pause_->setText(tr_ui("继续", "Resume"));
    } else {
        node_->call_pause_task(false);
        btn_pause_->setText(tr_ui("暂停", "Pause"));
    }
}

void TaskRunDialog::onStop() {
    node_->cancel_current_task();
    reject();
}

void TaskRunDialog::onEdit() {
    reject();
}

void TaskRunDialog::onTaskResult(const rclcpp_action::ClientGoalHandle<common_msgs::action::ExecuteTask>::WrappedResult& result) {
    btn_pause_->setEnabled(false);
    current_record_.end_time = QDateTime::currentDateTimeUtc();

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result && result.result->success) {
        label_status_->setText(tr_ui("任务完成: ", "Task Completed: ") + QString::fromStdString(result.result->message));
        current_record_.success = true;
        current_record_.error_msg.clear();
    } else if (result.result) {
        label_status_->setText(tr_ui("任务失败: ", "Task Failed: ") + QString::fromStdString(result.result->message));
        current_record_.success = false;
        current_record_.error_msg = QString::fromStdString(result.result->message);
    } else {
        label_status_->setText(tr_ui("任务结束(无结果)", "Task Finished with no result"));
        current_record_.success = false;
        current_record_.error_msg = "No result received";
    }

    if (record_manager_) {
        record_manager_->saveRecord(current_record_);
    }
}

void TaskRunDialog::onTaskFeedback(const std::shared_ptr<const common_msgs::action::ExecuteTask::Feedback> feedback) {
    int idx = feedback->current_step_index;
    std::string status = feedback->current_status;

    // Ensure steps vector is large enough
    while (current_record_.steps.size() <= idx) {
        StepRecord sr;
        int si = current_record_.steps.size();
        if (si < static_cast<int>(task_.task_seqs.size())) {
            sr.name = QString::fromStdString(task_.task_seqs[si].name);
            sr.type = QString::fromStdString(task_.task_seqs[si].type);
        }
        sr.success = true;
        sr.duration_ms = 0;
        current_record_.steps.append(sr);
    }

    // Parse structured feedback prefixes
    const std::string saved_prefix = "SAVED_FILE:";
    const std::string vision_prefix = "VISION_RESULT:";
    const std::string baseline_prefix = "VISION_BASELINE_OK:";

    if (status.rfind(saved_prefix, 0) == 0) {
        // SAVED_FILE:/path/to/file.png
        std::string path = status.substr(saved_prefix.size());
        if (idx >= 0 && idx < current_record_.steps.size()) {
            current_record_.steps[idx].captured_files.append(QString::fromStdString(path));
        }
    } else if (status.rfind(vision_prefix, 0) == 0) {
        // VISION_RESULT:angle,depth,confidence,saved_path
        std::string data = status.substr(vision_prefix.size());
        if (idx >= 0 && idx < current_record_.steps.size()) {
            auto& vd = current_record_.steps[idx].vision_data;
            vd.has_data = true;
            // Parse comma-separated values
            std::istringstream iss(data);
            std::string token;
            if (std::getline(iss, token, ',')) vd.angle_deg = std::stod(token);
            if (std::getline(iss, token, ',')) vd.depth_mm = std::stod(token);
            if (std::getline(iss, token, ',')) vd.confidence = std::stod(token);
            if (std::getline(iss, token, ',')) {
                if (!token.empty()) {
                    current_record_.steps[idx].captured_files.append(QString::fromStdString(token));
                }
            }
        }
    } else if (status.rfind(baseline_prefix, 0) == 0) {
        // VISION_BASELINE_OK:message
        if (idx >= 0 && idx < current_record_.steps.size()) {
            current_record_.steps[idx].vision_data.has_data = true;
            current_record_.steps[idx].vision_data.message = QString::fromStdString(status.substr(baseline_prefix.size()));
        }
    }

    progress_steps_->setValue(idx + 1);
    // Show user-friendly status (strip structured prefixes)
    if (status.rfind("SAVED_FILE:", 0) == 0 || status.rfind("VISION_RESULT:", 0) == 0 || status.rfind("VISION_BASELINE_OK:", 0) == 0) {
        label_status_->setText(tr_ui("执行中...", "Running..."));
    } else {
        label_status_->setText(QString::fromStdString(status));
    }
}
