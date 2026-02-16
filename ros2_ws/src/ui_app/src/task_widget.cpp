#include "ui_app/task_widget.hpp"
#include "ui_app/task_dialogs.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QListWidget>
#include <QLabel>
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QStandardPaths>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>
#include <QDateTime>
#include <iostream>
#include "common_msgs/msg/task_config.hpp"
// #include "ament_index_cpp/get_package_share_directory.hpp"

TaskWidget::TaskWidget(std::shared_ptr<RosNode> node, QWidget *parent)
    : QWidget(parent), node_(node) {
    auto * layout = new QVBoxLayout(this);

    layout->addWidget(new QLabel("Task List"));
    task_list_ = new QListWidget();
    layout->addWidget(task_list_);

    auto * h_btn = new QHBoxLayout();
    auto * btn_add = new QPushButton("New Task");
    auto * btn_edit = new QPushButton("Edit Task");
    auto * btn_del = new QPushButton("Delete Task");
    auto * btn_run = new QPushButton("Run Task");

    h_btn->addWidget(btn_add);
    h_btn->addWidget(btn_edit);
    h_btn->addWidget(btn_del);
    h_btn->addWidget(btn_run);
    layout->addLayout(h_btn);

    connect(btn_add, &QPushButton::clicked, this, &TaskWidget::onAddTask);
    connect(btn_edit, &QPushButton::clicked, this, &TaskWidget::onEditTask);
    connect(btn_del, &QPushButton::clicked, this, &TaskWidget::onDeleteTask);
    connect(btn_run, &QPushButton::clicked, this, &TaskWidget::onRunTask);

    loadTasks();
}

void TaskWidget::onAddTask() {
    TaskConfigDialog dlg(node_, this);
    if (dlg.exec() == QDialog::Accepted) {
        tasks_.push_back(dlg.getTask());
        updateTaskList();
        saveTasks();
    }
}

void TaskWidget::onEditTask() {
    int row = task_list_->currentRow();
    if (row >= 0 && row < (int)tasks_.size()) {
        TaskConfigDialog dlg(node_, this);
        dlg.setTask(tasks_[row]);
        if (dlg.exec() == QDialog::Accepted) {
            tasks_[row] = dlg.getTask();
            updateTaskList();
            saveTasks();
        }
    }
}

void TaskWidget::onDeleteTask() {
    int row = task_list_->currentRow();
    if (row >= 0 && row < (int)tasks_.size()) {
        if (QMessageBox::question(this, "Delete Task", "Are you sure you want to delete this task?", QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes) {
            tasks_.erase(tasks_.begin() + row);
            updateTaskList();
            saveTasks();
        }
    }
}

void TaskWidget::onRunTask() {
    int row = task_list_->currentRow();
    if (row >= 0 && row < (int)tasks_.size()) {
        TaskRunDialog dlg(node_, tasks_[row], this);
        dlg.exec();
    }
}

void TaskWidget::updateTaskList() {
    task_list_->clear();
    for (const auto& task : tasks_) {
        QString name = QString::fromStdString(task.task_name);
        QString info = QString("Steps: %1 | Devices: %2").arg(task.task_seqs.size()).arg(task.device_checks.size());
        task_list_->addItem(name + " [" + info + "]");
    }
}

// Simple JSON-like or CSV-like storage, or just binary?
// Let's use simple custom text format for now to avoid dependencies, or just don't persist complexly.
// User didn't strictly ask for persistence but it's good practice.
// I will just skip persistence implementation details for brevity or use a very simple one.
// Let's implement a dummy save/load for now or basic one.

void TaskWidget::saveTasks() {
    QDir dir(QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation));
    if (!dir.exists()) {
        dir.mkpath(".");
    }
    QString path = dir.filePath("tasks.json");
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        std::cerr << "Failed to open file for writing: " << path.toStdString() << std::endl;
        return;
    }

    QJsonArray jsonArray;
    for (const auto& task : tasks_) {
        QJsonObject taskObj;
        taskObj["task_name"] = QString::fromStdString(task.task_name);
        
        QJsonArray deviceChecks;
        for (const auto& check : task.device_checks) {
            QJsonObject checkObj;
            checkObj["device_type"] = QString::fromStdString(check.device_type);
            checkObj["device_name"] = QString::fromStdString(check.device_name);
            checkObj["device_sn"] = QString::fromStdString(check.device_sn);
            checkObj["device_usage"] = QString::fromStdString(check.device_usage);
            deviceChecks.append(checkObj);
        }
        taskObj["device_checks"] = deviceChecks;

        QJsonArray steps;
        for (const auto& step : task.task_seqs) {
            QJsonObject stepObj;
            stepObj["step_name"] = QString::fromStdString(step.name);
            stepObj["step_type"] = QString::fromStdString(step.type);
            
            QJsonArray armPos;
            for (double p : step.arm_pos) armPos.append(p);
            stepObj["arm_pos"] = armPos;
            
            QJsonArray handPos;
            for (int p : step.hand_pos) handPos.append(p);
            stepObj["hand_pos"] = handPos;
            
            QJsonArray camTypes;
            for (const auto& c : step.camera_type) camTypes.append(QString::fromStdString(c));
            stepObj["camera_type"] = camTypes;
            
            steps.append(stepObj);
        }
        taskObj["task_seqs"] = steps;
        
        jsonArray.append(taskObj);
    }
    
    QJsonDocument doc(jsonArray);
    file.write(doc.toJson());
    file.close();
}

void TaskWidget::loadTasks() {
    QDir dir(QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation));
    QString path = dir.filePath("tasks.json");
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        // No file or can't open
        return;
    }
    
    QByteArray data = file.readAll();
    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (!doc.isArray()) return;
    
    QJsonArray jsonArray = doc.array();
    tasks_.clear();
    
    for (const auto& val : jsonArray) {
        if (!val.isObject()) continue;
        QJsonObject taskObj = val.toObject();
        
        common_msgs::msg::TaskConfig task;
        task.task_name = taskObj["task_name"].toString().toStdString();
        
        QJsonArray deviceChecks = taskObj["device_checks"].toArray();
        for (const auto& dVal : deviceChecks) {
            QJsonObject dObj = dVal.toObject();
            common_msgs::msg::TaskDeviceCheck check;
            check.device_type = dObj["device_type"].toString().toStdString();
            check.device_name = dObj["device_name"].toString().toStdString();
            check.device_sn = dObj["device_sn"].toString().toStdString();
            check.device_usage = dObj["device_usage"].toString().toStdString();
            task.device_checks.push_back(check);
        }
        
        QJsonArray steps = taskObj["task_seqs"].toArray();
        for (const auto& sVal : steps) {
            QJsonObject sObj = sVal.toObject();
            common_msgs::msg::TaskStep step;
            step.name = sObj["step_name"].toString().toStdString();
            step.type = sObj["step_type"].toString().toStdString();
            
            QJsonArray armPos = sObj["arm_pos"].toArray();
            for (const auto& p : armPos) step.arm_pos.push_back(p.toDouble());
            
            QJsonArray handPos = sObj["hand_pos"].toArray();
            for (const auto& p : handPos) step.hand_pos.push_back(p.toInt());
            
            if (sObj["camera_type"].isArray()) {
                QJsonArray cams = sObj["camera_type"].toArray();
                for (const auto& c : cams) step.camera_type.push_back(c.toString().toStdString());
            } else {
                 // Fallback for legacy single string
                 step.camera_type.push_back(sObj["camera_type"].toString().toStdString());
            }
            
            task.task_seqs.push_back(step);
        }
        
        tasks_.push_back(task);
    }
    updateTaskList();
}
