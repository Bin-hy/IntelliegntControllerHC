#include "ui_app/task_dialogs.hpp"
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
    setWindowTitle("Add Device");
    auto * layout = new QVBoxLayout(this);

    auto * form = new QFormLayout();
    combo_type_ = new QComboBox();
    combo_type_->addItems({"duco", "lhand", "rhand", "orbbec"});
    form->addRow("Device Type:", combo_type_);

    edit_usage_ = new QLineEdit();
    form->addRow("Usage:", edit_usage_);

    combo_device_select_ = new QComboBox();
    form->addRow("Device:", combo_device_select_);

    edit_sn_ = new QLineEdit();
    form->addRow("SN:", edit_sn_);

    edit_model_ = new QLineEdit();
    form->addRow("Model:", edit_model_);

    layout->addLayout(form);

    auto * h_btn = new QHBoxLayout();
    btn_scan_ = new QPushButton("Scan");
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
        QMessageBox::information(this, "Scan", "No devices found of type " + type);
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
    setWindowTitle("Add Step");
    auto * layout = new QVBoxLayout(this);

    auto * h_name = new QHBoxLayout();
    h_name->addWidget(new QLabel("Step Name:"));
    edit_name_ = new QLineEdit();
    h_name->addWidget(edit_name_);
    layout->addLayout(h_name);

    combo_type_ = new QComboBox();
    combo_type_->addItems({"arm", "lhand", "rhand", "camera", "io"});
    layout->addWidget(new QLabel("Step Type:"));
    layout->addWidget(combo_type_);

    layout->addWidget(new QLabel("Target Device:"));
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
    auto * btn_capture = new QPushButton("Capture Current");
    arm_layout->addRow(btn_capture);
    connect(btn_capture, &QPushButton::clicked, this, &StepAddDialog::onCaptureCurrent);

    widget_hand_ = new QWidget();
    auto * hand_layout = new QFormLayout(widget_hand_);
    for(int i=0; i<6; ++i) {
        spin_hand_pos_[i] = new QSpinBox();
        spin_hand_pos_[i]->setRange(0, 100000);
        hand_layout->addRow(QString("Finger %1").arg(i+1), spin_hand_pos_[i]);
    }

    widget_camera_ = new QWidget();
    auto * cam_layout = new QFormLayout(widget_camera_);
    combo_camera_type_ = new QComboBox();
    combo_camera_type_->addItems({"color", "depth", "ir_left", "ir_right", "point_cloud"});
    cam_layout->addRow("Camera Type:", combo_camera_type_);

    widget_io_ = new QWidget();
    auto * io_layout = new QFormLayout(widget_io_);
    combo_io_device_ = new QComboBox();
    combo_io_device_->addItem(QString::fromUtf8("清洗机 (IO 1)"), 1);
    combo_io_device_->addItem(QString::fromUtf8("吹风机 (IO 2)"), 2);
    io_layout->addRow(QString::fromUtf8("IO 设备:"), combo_io_device_);
    combo_io_value_ = new QComboBox();
    combo_io_value_->addItem(QString::fromUtf8("开启 (HIGH)"), true);
    combo_io_value_->addItem(QString::fromUtf8("关闭 (LOW)"), false);
    io_layout->addRow(QString::fromUtf8("电平:"), combo_io_value_);

    stack->addWidget(widget_arm_);
    stack->addWidget(widget_hand_);
    stack->addWidget(widget_camera_);
    stack->addWidget(widget_io_);
    layout->addWidget(stack);

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
    else stack->setCurrentWidget(widget_camera_);
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
        for (int i = 0; i < combo_io_device_->count(); ++i) {
            if (combo_io_device_->itemData(i).toInt() == step.io_port) {
                combo_io_device_->setCurrentIndex(i);
                break;
            }
        }
        combo_io_value_->setCurrentIndex(step.io_value ? 0 : 1);
    }
}

common_msgs::msg::TaskStep StepAddDialog::getStep() const {
    common_msgs::msg::TaskStep step;
    step.name = edit_name_->text().toStdString();
    step.type = combo_type_->currentText().toStdString();
    step.device_sn = combo_device_->currentData().toString().toStdString();
    if (step.device_sn.empty()) step.device_sn = combo_device_->currentText().toStdString();

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
        step.io_port = static_cast<int8_t>(combo_io_device_->currentData().toInt());
        step.io_value = combo_io_value_->currentData().toBool();
    }
    return step;
}

TaskConfigDialog::TaskConfigDialog(std::shared_ptr<RosNode> node, QWidget *parent)
    : QDialog(parent), node_(node) {
    setWindowTitle("Task Configuration");
    auto * layout = new QVBoxLayout(this);

    auto * h_name = new QHBoxLayout();
    h_name->addWidget(new QLabel("Task Name:"));
    edit_name_ = new QLineEdit();
    h_name->addWidget(edit_name_);
    layout->addLayout(h_name);

    auto * h_rounds = new QHBoxLayout();
    h_rounds->addWidget(new QLabel("执行轮次:"));
    spin_rounds_ = new QSpinBox();
    spin_rounds_->setRange(1, 999);
    spin_rounds_->setValue(1);
    h_rounds->addWidget(spin_rounds_);
    layout->addLayout(h_rounds);

    layout->addWidget(new QLabel("Devices:"));
    table_devices_ = new QTableWidget();
    table_devices_->setColumnCount(4);
    table_devices_->setHorizontalHeaderLabels({"Type", "Name", "SN", "Usage"});
    table_devices_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    layout->addWidget(table_devices_);

    auto * h_dev_btn = new QHBoxLayout();
    auto * btn_add_dev = new QPushButton("Add Device");
    auto * btn_del_dev = new QPushButton("Delete Device");
    h_dev_btn->addWidget(btn_add_dev);
    h_dev_btn->addWidget(btn_del_dev);
    layout->addLayout(h_dev_btn);

    layout->addWidget(new QLabel("Steps:"));
    list_steps_ = new QListWidget();
    layout->addWidget(list_steps_);

    auto * h_step_btn = new QHBoxLayout();
    auto * btn_add_step = new QPushButton("Add Step");
    auto * btn_edit_step = new QPushButton("Edit Step");
    auto * btn_del_step = new QPushButton("Delete Step");
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
        QMessageBox::information(this, "Edit Step", "Please select a step to edit.");
        return;
    }
    StepAddDialog dlg(node_, devices_, this);
    dlg.setWindowTitle("Edit Step");
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
    setWindowTitle("Run Task");
    auto * layout = new QVBoxLayout(this);

    label_status_ = new QLabel("Checking devices...");
    layout->addWidget(label_status_);

    progress_steps_ = new QProgressBar();
    int rounds = task_.exec_rounds > 0 ? task_.exec_rounds : 1;
    int total_steps = static_cast<int>(task_.task_seqs.size()) * rounds;
    progress_steps_->setRange(0, total_steps);
    progress_steps_->setValue(0);
    layout->addWidget(progress_steps_);

    auto * h_btn = new QHBoxLayout();
    btn_start_ = new QPushButton("Start");
    btn_pause_ = new QPushButton("Pause");
    btn_stop_ = new QPushButton("Stop");
    auto * btn_edit = new QPushButton("Edit");
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
        label_status_->setText("Devices Checked: All Ready");
        btn_start_->setEnabled(true);
        devices_ready_ = true;
    } else {
        label_status_->setText("Missing Devices: " + missing);
        btn_start_->setEnabled(false);
        devices_ready_ = false;
    }
}

void TaskRunDialog::onStart() {
    if (!devices_ready_) return;
    btn_start_->setEnabled(false);
    btn_pause_->setEnabled(true);
    label_status_->setText("Running...");

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
    if (btn_pause_->text() == "Pause") {
        node_->call_pause_task(true);
        btn_pause_->setText("Resume");
    } else {
        node_->call_pause_task(false);
        btn_pause_->setText("Pause");
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
        label_status_->setText("Task Completed: " + QString::fromStdString(result.result->message));
        current_record_.success = true;
        current_record_.error_msg.clear();
    } else if (result.result) {
        label_status_->setText("Task Failed: " + QString::fromStdString(result.result->message));
        current_record_.success = false;
        current_record_.error_msg = QString::fromStdString(result.result->message);
    } else {
        label_status_->setText("Task Finished with no result");
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
