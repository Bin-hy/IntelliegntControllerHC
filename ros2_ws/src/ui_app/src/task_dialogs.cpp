#include <QGroupBox>
#include "ui_app/task_dialogs.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QTableWidget>
#include <QHeaderView>
#include <QListWidget>
#include <QMessageBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QProgressBar>
#include <QStackedWidget>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QTimer>
#include "ui_app/robot_viz_widget.hpp"
#include "ui_app/point_cloud_widget.hpp"

// ----------------------------------------------------------------------------
// DeviceAddDialog
// ----------------------------------------------------------------------------
DeviceAddDialog::DeviceAddDialog(std::shared_ptr<RosNode> node, QWidget *parent)
    : QDialog(parent), node_(node) {
    setWindowTitle("Add Device");
    auto layout = new QVBoxLayout(this);
    auto form = new QFormLayout();

    combo_type_ = new QComboBox();
    combo_type_->addItems({"duco", "lhand", "rhand", "orbbec"});
    form->addRow("Type:", combo_type_);

    edit_usage_ = new QLineEdit();
    form->addRow("Usage:", edit_usage_);

    combo_device_select_ = new QComboBox();
    auto h_layout = new QHBoxLayout();
    h_layout->addWidget(combo_device_select_);
    btn_scan_ = new QPushButton("Scan");
    h_layout->addWidget(btn_scan_);
    form->addRow("Select:", h_layout);

    edit_sn_ = new QLineEdit();
    form->addRow("SN:", edit_sn_);

    edit_model_ = new QLineEdit();
    form->addRow("Model:", edit_model_);

    layout->addLayout(form);

    auto btn_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(btn_box);

    connect(btn_box, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(btn_box, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(btn_scan_, &QPushButton::clicked, this, &DeviceAddDialog::onScan);
    connect(combo_type_, &QComboBox::currentTextChanged, this, &DeviceAddDialog::onTypeChanged);
    connect(combo_device_select_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &DeviceAddDialog::onDeviceIndexChanged);
}

void DeviceAddDialog::onDeviceIndexChanged(int index) {
    if (index >= 0) {
        edit_sn_->setText(combo_device_select_->itemData(index).toString());
    }
}

void DeviceAddDialog::onTypeChanged(const QString& type) {
    combo_device_select_->clear();
    // Logic to filter devices based on type?
    if (type == "duco") {
         edit_model_->setText("Duco Robot");
    } else if (type == "lhand") {
         edit_model_->setText("LHand Pro");
    } else if (type == "orbbec") {
         edit_model_->setText("Orbbec Camera");
    }
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
        else if (target_type == "orbbec" && (dev.device_type == "orbbec" || dev.device_type == "camera_server")) match = true;
        
        if (match) {
            QString label = QString::fromStdString(dev.device_model) + " (" + QString::fromStdString(dev.device_sn) + ")";
            combo_device_select_->addItem(label, QString::fromStdString(dev.device_sn));
            found = true;
        }
    }

    if (!found) {
        if (target_type == "orbbec") {
            auto cameras = node_->scan_cameras();
            for (const auto& cam : cameras) {
                combo_device_select_->addItem(QString::fromStdString(cam), QString::fromStdString(cam));
            }
            if (!cameras.empty()) found = true;
        }
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

// ----------------------------------------------------------------------------
// StepAddDialog
// ----------------------------------------------------------------------------
StepAddDialog::StepAddDialog(std::shared_ptr<RosNode> node, QWidget *parent)
    : QDialog(parent), node_(node) {
    setWindowTitle("Add Step");
    auto layout = new QVBoxLayout(this);
    
    auto h_name = new QHBoxLayout();
    h_name->addWidget(new QLabel("Step Name:"));
    edit_name_ = new QLineEdit();
    h_name->addWidget(edit_name_);
    layout->addLayout(h_name);

    combo_type_ = new QComboBox();
    combo_type_->addItems({"arm", "lhand", "rhand", "camera"});
    layout->addWidget(new QLabel("Step Type:"));
    layout->addWidget(combo_type_);

    auto stack = new QStackedWidget();
    
    // Arm Widget
    widget_arm_ = new QWidget();
    auto arm_layout = new QFormLayout(widget_arm_);
    for(int i=0; i<6; ++i) { // 6-axis
        spin_arm_pos_[i] = new QDoubleSpinBox();
        spin_arm_pos_[i]->setRange(-360.0, 360.0);
        arm_layout->addRow("J" + QString::number(i+1), spin_arm_pos_[i]);
    }
    stack->addWidget(widget_arm_);

    // Hand Widget
    widget_hand_ = new QWidget();
    auto hand_layout = new QFormLayout(widget_hand_);
    for(int i=0; i<6; ++i) {
        spin_hand_pos_[i] = new QSpinBox();
        spin_hand_pos_[i]->setRange(0, 1000); // 0-1000 range
        hand_layout->addRow("F" + QString::number(i+1), spin_hand_pos_[i]);
    }
    stack->addWidget(widget_hand_);

    // Camera Widget
    widget_camera_ = new QWidget();
    auto cam_layout = new QFormLayout(widget_camera_);
    combo_camera_type_ = new QComboBox();
    combo_camera_type_->addItems({"color", "depth", "ir"}); // IR left/right?
    cam_layout->addRow("Capture Type:", combo_camera_type_);
    stack->addWidget(widget_camera_);

    layout->addWidget(stack);

    auto btn_capture = new QPushButton("Capture Current State");
    layout->addWidget(btn_capture);
    
    auto btn_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(btn_box);

    connect(btn_box, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(btn_box, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(btn_capture, &QPushButton::clicked, this, &StepAddDialog::onCaptureCurrent);
    connect(combo_type_, &QComboBox::currentTextChanged, this, &StepAddDialog::onTypeChanged);
}

void StepAddDialog::onTypeChanged(const QString& type) {
    if (type == "arm") {
        static_cast<QStackedWidget*>(widget_arm_->parentWidget())->setCurrentWidget(widget_arm_);
    } else if (type.contains("hand")) {
        static_cast<QStackedWidget*>(widget_hand_->parentWidget())->setCurrentWidget(widget_hand_);
    } else if (type == "camera") {
        static_cast<QStackedWidget*>(widget_camera_->parentWidget())->setCurrentWidget(widget_camera_);
    }
}

void StepAddDialog::onCaptureCurrent() {
    QString type = combo_type_->currentText();
    if (type == "arm") {
        if (node_->current_joints_.size() >= 6) {
             for(int i=0; i<6; ++i) spin_arm_pos_[i]->setValue(node_->current_joints_[i] * 180.0 / M_PI); // Rad to Deg
        } else {
            QMessageBox::warning(this, "Capture", "Robot joint state not available yet.");
        }
    } else if (type.contains("hand")) {
        for(int i=0; i<6; ++i) {
             // joint_id 1-6
             node_->call_lhand_get_position(i + 1, [this, i](int pos){
                 QMetaObject::invokeMethod(this, [this, i, pos](){
                     if(i < 6) spin_hand_pos_[i]->setValue(pos);
                 });
             });
        }
    }
}

common_msgs::msg::TaskStep StepAddDialog::getStep() const {
    common_msgs::msg::TaskStep step;
    step.name = edit_name_->text().toStdString();
    step.type = combo_type_->currentText().toStdString();
    if (step.type == "arm") {
        for(int i=0; i<6; ++i) step.arm_pos.push_back(spin_arm_pos_[i]->value() * M_PI / 180.0);
    } else if (step.type.find("hand") != std::string::npos) {
        for(int i=0; i<6; ++i) step.hand_pos.push_back(spin_hand_pos_[i]->value());
    } else if (step.type == "camera") {
        step.camera_type.push_back(combo_camera_type_->currentText().toStdString());
    }
    return step;
}

// ----------------------------------------------------------------------------
// TaskConfigDialog
// ----------------------------------------------------------------------------
TaskConfigDialog::TaskConfigDialog(std::shared_ptr<RosNode> node, QWidget *parent)
    : QDialog(parent), node_(node) {
    setWindowTitle("Task Configuration");
    resize(800, 600);
    auto layout = new QVBoxLayout(this);

    auto form = new QFormLayout();
    edit_name_ = new QLineEdit();
    form->addRow("Task Name:", edit_name_);
    layout->addLayout(form);

    // Devices
    auto grp_devices = new QGroupBox("Devices");
    auto dev_layout = new QVBoxLayout(grp_devices);
    table_devices_ = new QTableWidget();
    table_devices_->setColumnCount(4);
    table_devices_->setHorizontalHeaderLabels({"Type", "Usage", "Name", "SN"});
    table_devices_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    dev_layout->addWidget(table_devices_);
    
    auto h_dev_btn = new QHBoxLayout();
    auto btn_add_dev = new QPushButton("Add Device");
    auto btn_del_dev = new QPushButton("Delete Device");
    h_dev_btn->addWidget(btn_add_dev);
    h_dev_btn->addWidget(btn_del_dev);
    dev_layout->addLayout(h_dev_btn);
    layout->addWidget(grp_devices);

    // Steps
    auto grp_steps = new QGroupBox("Steps");
    auto step_layout = new QVBoxLayout(grp_steps);
    list_steps_ = new QListWidget();
    step_layout->addWidget(list_steps_);

    auto h_step_btn = new QHBoxLayout();
    auto btn_add_step = new QPushButton("Add Step");
    auto btn_del_step = new QPushButton("Delete Step");
    h_step_btn->addWidget(btn_add_step);
    h_step_btn->addWidget(btn_del_step);
    step_layout->addLayout(h_step_btn);
    layout->addWidget(grp_steps);

    auto btn_box = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel);
    layout->addWidget(btn_box);

    connect(btn_box, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(btn_box, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(btn_add_dev, &QPushButton::clicked, this, &TaskConfigDialog::onAddDevice);
    connect(btn_del_dev, &QPushButton::clicked, this, &TaskConfigDialog::onDeleteDevice);
    connect(btn_add_step, &QPushButton::clicked, this, &TaskConfigDialog::onAddStep);
    connect(btn_del_step, &QPushButton::clicked, this, &TaskConfigDialog::onDeleteStep);
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
    StepAddDialog dlg(node_, this);
    if (dlg.exec() == QDialog::Accepted) {
        steps_.push_back(dlg.getStep());
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

void TaskConfigDialog::updateDeviceTable() {
    table_devices_->setRowCount(devices_.size());
    for(size_t i=0; i<devices_.size(); ++i) {
        table_devices_->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(devices_[i].device_type)));
        table_devices_->setItem(i, 1, new QTableWidgetItem(QString::fromStdString(devices_[i].device_usage)));
        table_devices_->setItem(i, 2, new QTableWidgetItem(QString::fromStdString(devices_[i].device_name)));
        table_devices_->setItem(i, 3, new QTableWidgetItem(QString::fromStdString(devices_[i].device_sn)));
    }
}

void TaskConfigDialog::updateStepList() {
    list_steps_->clear();
    for(size_t i=0; i<steps_.size(); ++i) {
        QString txt = "Step " + QString::number(i+1) + ": " + QString::fromStdString(steps_[i].name) + " [" + QString::fromStdString(steps_[i].type) + "]";
        list_steps_->addItem(txt);
    }
}

common_msgs::msg::TaskConfig TaskConfigDialog::getTask() const {
    common_msgs::msg::TaskConfig task;
    task.task_name = edit_name_->text().toStdString();
    task.device_checks = devices_;
    task.task_seqs = steps_;
    // timestamp
    task.created_time = std::time(nullptr);
    return task;
}

void TaskConfigDialog::setTask(const common_msgs::msg::TaskConfig& task) {
    edit_name_->setText(QString::fromStdString(task.task_name));
    devices_ = task.device_checks;
    steps_ = task.task_seqs;
    updateDeviceTable();
    updateStepList();
}

// ----------------------------------------------------------------------------
// TaskRunDialog
// ----------------------------------------------------------------------------
TaskRunDialog::TaskRunDialog(std::shared_ptr<RosNode> node, const common_msgs::msg::TaskConfig& task, QWidget *parent)
    : QDialog(parent), node_(node), task_(task), devices_ready_(false) {
    setWindowTitle("Run Task: " + QString::fromStdString(task.task_name));
    resize(1000, 700);
    auto layout = new QVBoxLayout(this);

    // Header
    label_status_ = new QLabel("Checking Devices...");
    layout->addWidget(label_status_);
    progress_steps_ = new QProgressBar();
    progress_steps_->setRange(0, task.task_seqs.size());
    layout->addWidget(progress_steps_);

    // Viz Area
    auto h_viz = new QHBoxLayout();
    robot_viz_ = new RobotVizWidget(node);
    h_viz->addWidget(robot_viz_);
    
    // If needed, add LHand viz
    // lhand_viz_ = new RobotVizWidget(node); 
    // h_viz->addWidget(lhand_viz_);
    
    layout->addLayout(h_viz);

    // Controls
    auto h_ctrl = new QHBoxLayout();
    btn_start_ = new QPushButton("Start Task");
    btn_pause_ = new QPushButton("Pause");
    btn_stop_ = new QPushButton("Stop / Emergency");
    
    btn_start_->setEnabled(false); // Enable after check
    btn_pause_->setEnabled(false);
    btn_stop_->setEnabled(true); // Always enabled for safety? Or only when running?

    h_ctrl->addWidget(btn_start_);
    h_ctrl->addWidget(btn_pause_);
    h_ctrl->addWidget(btn_stop_);
    layout->addLayout(h_ctrl);

    connect(btn_start_, &QPushButton::clicked, this, &TaskRunDialog::onStart);
    connect(btn_pause_, &QPushButton::clicked, this, &TaskRunDialog::onPause);
    connect(btn_stop_, &QPushButton::clicked, this, &TaskRunDialog::onStop);
    
    // Add Edit button
    auto btn_edit = new QPushButton("Edit Task");
    h_ctrl->addWidget(btn_edit);
    connect(btn_edit, &QPushButton::clicked, this, &TaskRunDialog::onEdit);

    // Auto check devices
    QTimer::singleShot(500, this, &TaskRunDialog::checkDevices);
}

void TaskRunDialog::checkDevices() {
    // Mock check or call system_controller check?
    // System Controller check is part of execution action.
    // User wants check BEFORE execution.
    // We can simulate check here by checking available nodes/topics manually.
    
    bool all_ok = true;
    QString missing;

    for (const auto& check : task_.device_checks) {
        if (!node_->check_device_availability(check)) {
            all_ok = false;
            missing += QString::fromStdString(check.device_type) + " ";
        }
    }

    if (all_ok) {
        label_status_->setText("Devices Checked: All Ready");
        btn_start_->setEnabled(true);
    } else {
        label_status_->setText("Missing Devices: " + missing);
        btn_start_->setEnabled(false);
    }
}

void TaskRunDialog::onStart() {
    btn_start_->setEnabled(false);
    btn_pause_->setEnabled(true);
    label_status_->setText("Running...");
    
    node_->call_execute_task(task_, 
        std::bind(&TaskRunDialog::onTaskResult, this, std::placeholders::_1),
        std::bind(&TaskRunDialog::onTaskFeedback, this, std::placeholders::_1)
    );
}

void TaskRunDialog::onPause() {
    RCLCPP_INFO(node_->get_logger(), "Task Paused.");
    if (btn_pause_->text() == "Pause") {
        node_->call_pause_task(true);
        btn_pause_->setText("Resume");
    } else {
        node_->call_pause_task(false);
        btn_pause_->setText("Pause");
    }
}

void TaskRunDialog::onStop() {
    RCLCPP_INFO(node_->get_logger(), "Task Stopped by user.");
    node_->cancel_current_task();
    reject();
}

void TaskRunDialog::onEdit() {
    // Edit the task configuration
    // This dialog is modal, so we might need to close it or open another dialog on top
    TaskConfigDialog dialog(node_, this);
    dialog.setTask(task_);
    if (dialog.exec() == QDialog::Accepted) {
        task_ = dialog.getTask();
        // Update UI or restart check?
        checkDevices();
    }
}

void TaskRunDialog::onTaskResult(const rclcpp_action::ClientGoalHandle<common_msgs::action::ExecuteTask>::WrappedResult& result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        label_status_->setText("Task Completed Successfully!");
        progress_steps_->setValue(task_.task_seqs.size());
    } else {
        label_status_->setText("Task Failed or Canceled");
    }
    btn_start_->setEnabled(true);
    btn_pause_->setEnabled(false);
    btn_pause_->setText("Pause"); // Reset text
}

void TaskRunDialog::onTaskFeedback(const std::shared_ptr<const common_msgs::action::ExecuteTask::Feedback> feedback) {
    label_status_->setText(QString::fromStdString(feedback->current_status));
    progress_steps_->setValue(feedback->current_step_index);
}

