#include "ui_app/task_dialogs.hpp"
#include "ui_app/i18n_manager.hpp"
#include "ui_app/robot_viz_widget.hpp"
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
    combo_type_->addItems({"arm", "lhand", "rhand", "camera", "io", "lift"});
    layout->addWidget(new QLabel(tr_ui("步骤类型:", "Step Type:")));
    layout->addWidget(combo_type_);

    layout->addWidget(new QLabel(tr_ui("目标设备:", "Target Device:")));
    combo_device_ = new QComboBox();
    layout->addWidget(combo_device_);

    auto * stack = new QStackedWidget();

    widget_arm_ = new QWidget();
    auto * arm_layout = new QFormLayout(widget_arm_);
    for(int i=0; i<6; ++i) {
        spin_arm_pos_[i] = new QDoubleSpinBox();
        spin_arm_pos_[i]->setRange(-6.28, 6.28);
        spin_arm_pos_[i]->setSingleStep(0.01);
        arm_layout->addRow(QString("J%1").arg(i+1), spin_arm_pos_[i]);
    }
    auto * btn_capture = new QPushButton(tr_ui("获取当前位置", "Capture Current"));
    arm_layout->addRow(btn_capture);
    connect(btn_capture, &QPushButton::clicked, this, &StepAddDialog::onCaptureCurrent);

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
    combo_camera_type_->addItems({"color", "depth", "ir_left", "ir_right", "point_cloud"});
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

void StepAddDialog::onCaptureCurrent() {
    std::lock_guard<std::mutex> lock(node_->data_mutex_);
    if (node_->current_joints_.size() >= 6) {
        for (int i = 0; i < 6; ++i) {
            spin_arm_pos_[i]->setValue(node_->current_joints_[i]);
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
        int n = std::min(static_cast<int>(step.arm_pos.size()), 6);
        for (int i = 0; i < n; ++i) {
            spin_arm_pos_[i]->setValue(step.arm_pos[i]);
        }
    } else if (step.type == "lhand" || step.type == "rhand") {
        int n = std::min(static_cast<int>(step.hand_pos.size()), 6);
        for (int i = 0; i < n; ++i) {
            spin_hand_pos_[i]->setValue(static_cast<int>(step.hand_pos[i]));
        }
    } else if (step.type == "camera") {
        if (!step.camera_type.empty()) {
            int ct_idx = combo_camera_type_->findText(QString::fromStdString(step.camera_type[0]));
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
        step.arm_pos.clear();
        for(int i=0; i<6; ++i) step.arm_pos.push_back(spin_arm_pos_[i]->value());
    } else if (step.type == "lhand" || step.type == "rhand") {
        step.hand_pos.clear();
        for(int i=0; i<6; ++i) step.hand_pos.push_back(spin_hand_pos_[i]->value());
    } else if (step.type == "camera") {
        step.camera_type.clear();
        step.camera_type.push_back(combo_camera_type_->currentText().toStdString());
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
    layout->addWidget(list_steps_);

    auto * h_step_btn = new QHBoxLayout();
    auto * btn_add_step = new QPushButton(tr_ui("添加步骤", "Add Step"));
    auto * btn_edit_step = new QPushButton(tr_ui("编辑步骤", "Edit Step"));
    auto * btn_del_step = new QPushButton(tr_ui("删除步骤", "Delete Step"));
    h_step_btn->addWidget(btn_add_step);
    h_step_btn->addWidget(btn_edit_step);
    h_step_btn->addWidget(btn_del_step);
    layout->addLayout(h_step_btn);

    auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);

    connect(btn_add_dev, &QPushButton::clicked, this, &TaskConfigDialog::onAddDevice);
    connect(btn_del_dev, &QPushButton::clicked, this, &TaskConfigDialog::onDeleteDevice);
    connect(btn_add_step, &QPushButton::clicked, this, &TaskConfigDialog::onAddStep);
    connect(btn_edit_step, &QPushButton::clicked, this, &TaskConfigDialog::onEditStep);
    connect(btn_del_step, &QPushButton::clicked, this, &TaskConfigDialog::onDeleteStep);
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
    for (const auto& s : steps_) {
        QString label = QString::fromStdString(s.name) + " [" + QString::fromStdString(s.type) + "]";
        list_steps_->addItem(label);
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

TaskRunDialog::TaskRunDialog(std::shared_ptr<RosNode> node, const common_msgs::msg::TaskConfig& task, std::shared_ptr<TaskRecordManager> record_manager, QWidget *parent)
    : QDialog(parent), node_(node), record_manager_(record_manager), task_(task), devices_ready_(false) {
    setWindowTitle(tr_ui("运行任务", "Run Task"));
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
    progress_steps_->setValue(idx + 1);
    label_status_->setText(QString::fromStdString(feedback->current_status));

    // Track step completion in the record
    while (current_record_.steps.size() <= idx) {
        StepRecord sr;
        if (idx < static_cast<int>(task_.task_seqs.size())) {
            sr.name = QString::fromStdString(task_.task_seqs[idx].name);
            sr.type = QString::fromStdString(task_.task_seqs[idx].type);
        }
        sr.success = true;
        sr.duration_ms = 0;
        current_record_.steps.append(sr);
    }
}
