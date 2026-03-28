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
#include <QFileDialog>
#include <QHeaderView>
#include <QTableWidget>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDirIterator>
#include <QPixmap>
#include <iostream>
#include "common_msgs/msg/task_config.hpp"
// #include "ament_index_cpp/get_package_share_directory.hpp"

TaskWidget::TaskWidget(std::shared_ptr<RosNode> node, const QString& username, UserRole role, QWidget *parent)
    : QWidget(parent), node_(node), current_user_(username), current_role_(role) {
    // Initialize record manager
    QDir records_dir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation));
    record_manager_ = std::make_shared<TaskRecordManager>(records_dir.filePath("task_records"));

    auto * layout = new QVBoxLayout(this);

    layout->addWidget(new QLabel("Task List"));
    task_list_ = new QListWidget();
    layout->addWidget(task_list_);

    auto * h_btn = new QHBoxLayout();
    auto * btn_add = new QPushButton("New Task");
    auto * btn_edit = new QPushButton("Edit Task");
    auto * btn_del = new QPushButton("Delete Task");
    auto * btn_run = new QPushButton("Run Task");
    auto * btn_history = new QPushButton("历史记录");
    auto * btn_photos = new QPushButton("照片管理");
    auto * btn_export = new QPushButton("导出 CSV");

    h_btn->addWidget(btn_add);
    h_btn->addWidget(btn_edit);
    h_btn->addWidget(btn_del);
    h_btn->addWidget(btn_run);
    h_btn->addWidget(btn_history);
    h_btn->addWidget(btn_photos);
    h_btn->addWidget(btn_export);
    layout->addLayout(h_btn);

    connect(btn_add, &QPushButton::clicked, this, &TaskWidget::onAddTask);
    connect(btn_edit, &QPushButton::clicked, this, &TaskWidget::onEditTask);
    connect(btn_del, &QPushButton::clicked, this, &TaskWidget::onDeleteTask);
    connect(btn_run, &QPushButton::clicked, this, &TaskWidget::onRunTask);
    connect(btn_history, &QPushButton::clicked, this, &TaskWidget::onShowHistory);
    connect(btn_photos, &QPushButton::clicked, this, &TaskWidget::onShowPhotos);
    connect(btn_export, &QPushButton::clicked, this, &TaskWidget::onExportCSV);

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
        TaskRunDialog dlg(node_, tasks_[row], record_manager_, this);
        dlg.exec();
    }
}

void TaskWidget::updateTaskList() {
    task_list_->clear();
    for (const auto& task : tasks_) {
        QString name = QString::fromStdString(task.task_name);
        int rounds = task.exec_rounds > 0 ? task.exec_rounds : 1;
        QString info = QString("Steps: %1 | Devices: %2 | Rounds: %3")
                           .arg(task.task_seqs.size())
                           .arg(task.device_checks.size())
                           .arg(rounds);
        task_list_->addItem(name + " [" + info + "]");
    }
}

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
        taskObj["exec_rounds"] = task.exec_rounds > 0 ? task.exec_rounds : 1;
        
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
            stepObj["device_sn"] = QString::fromStdString(step.device_sn);
            
            QJsonArray armPos;
            for (double p : step.arm_pos) armPos.append(p);
            stepObj["arm_pos"] = armPos;
            
            QJsonArray handPos;
            for (int p : step.hand_pos) handPos.append(p);
            stepObj["hand_pos"] = handPos;
            
            QJsonArray camTypes;
            for (const auto& c : step.camera_type) camTypes.append(QString::fromStdString(c));
            stepObj["camera_type"] = camTypes;

            stepObj["io_port"] = step.io_port;
            stepObj["io_value"] = step.io_value;

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
        int rounds = taskObj.contains("exec_rounds") ? taskObj["exec_rounds"].toInt() : 1;
        task.exec_rounds = rounds > 0 ? rounds : 1;
        
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
            step.device_sn = sObj["device_sn"].toString().toStdString();
            
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

            step.io_port = static_cast<int8_t>(sObj["io_port"].toInt(0));
            step.io_value = sObj["io_value"].toBool(false);

            task.task_seqs.push_back(step);
        }
        
        tasks_.push_back(task);
    }
    updateTaskList();
}

void TaskWidget::onShowHistory() {
    if (!record_manager_) return;

    auto records = record_manager_->loadAllRecords();

    QDialog dlg(this);
    dlg.setWindowTitle("任务执行历史");
    dlg.resize(700, 400);
    auto * layout = new QVBoxLayout(&dlg);

    auto * table = new QTableWidget();
    table->setColumnCount(5);
    table->setHorizontalHeaderLabels({"任务名称", "开始时间", "结束时间", "结果", "错误信息"});
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setRowCount(records.size());

    for (int i = 0; i < records.size(); ++i) {
        const auto& r = records[i];
        table->setItem(i, 0, new QTableWidgetItem(r.task_name));
        table->setItem(i, 1, new QTableWidgetItem(r.start_time.toLocalTime().toString("yyyy-MM-dd HH:mm:ss")));
        table->setItem(i, 2, new QTableWidgetItem(r.end_time.toLocalTime().toString("yyyy-MM-dd HH:mm:ss")));
        auto * result_item = new QTableWidgetItem(r.success ? "成功" : "失败");
        result_item->setForeground(r.success ? Qt::darkGreen : Qt::red);
        table->setItem(i, 3, result_item);
        table->setItem(i, 4, new QTableWidgetItem(r.error_msg));
    }

    layout->addWidget(table);

    auto * buttons = new QDialogButtonBox(QDialogButtonBox::Close);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    layout->addWidget(buttons);

    dlg.exec();
}

void TaskWidget::onShowPhotos() {
    QString base_dir = QDir::homePath() + "/.ros/task_photos";
    QDir root(base_dir);
    if (!root.exists()) {
        QMessageBox::information(this, "照片管理", "未找到照片目录: " + base_dir);
        return;
    }

    struct PhotoItem {
        QString task;
        QString time;
        QString round;
        QString user;
        QString file;
        QString path;
    };

    QVector<PhotoItem> items;
    QDirIterator it(base_dir, QStringList() << "*.png", QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) {
        QString path = it.next();
        QString rel = root.relativeFilePath(path);
        QStringList parts = rel.split("/", Qt::SkipEmptyParts);
        if (parts.size() < 5) {
            continue;
        }
        PhotoItem item;
        item.task = parts[0];
        item.time = parts[1];
        item.round = parts[2];
        item.user = parts[3];
        item.file = parts.last();
        item.path = path;
        if (current_role_ == UserRole::Operator) {
            if (item.user != current_user_) {
                continue;
            }
        }
        items.append(item);
    }

    QDialog dlg(this);
    dlg.setWindowTitle("照片管理");
    dlg.resize(900, 500);
    auto * layout = new QHBoxLayout(&dlg);

    auto * table = new QTableWidget();
    table->setColumnCount(5);
    table->setHorizontalHeaderLabels({"任务", "时间", "轮次", "用户", "文件"});
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setRowCount(items.size());
    for (int i = 0; i < items.size(); ++i) {
        const auto& p = items[i];
        table->setItem(i, 0, new QTableWidgetItem(p.task));
        table->setItem(i, 1, new QTableWidgetItem(p.time));
        table->setItem(i, 2, new QTableWidgetItem(p.round));
        table->setItem(i, 3, new QTableWidgetItem(p.user));
        table->setItem(i, 4, new QTableWidgetItem(p.file));
    }

    auto * right = new QVBoxLayout();
    auto * preview = new QLabel();
    preview->setMinimumSize(320, 240);
    preview->setAlignment(Qt::AlignCenter);
    right->addWidget(preview, 1);

    auto * btn_delete = new QPushButton("删除照片");
    bool can_delete = current_role_ == UserRole::Admin || current_role_ == UserRole::Operator;
    btn_delete->setEnabled(can_delete);
    right->addWidget(btn_delete);

    layout->addWidget(table, 2);
    layout->addLayout(right, 1);

    auto refresh_preview = [table, preview, &items]() {
        int row = table->currentRow();
        if (row < 0 || row >= items.size()) {
            preview->clear();
            return;
        }
        QPixmap pix(items[row].path);
        if (!pix.isNull()) {
            preview->setPixmap(pix.scaled(preview->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        } else {
            preview->setText("无法加载图片");
        }
    };

    connect(table, &QTableWidget::itemSelectionChanged, &dlg, refresh_preview);

    connect(btn_delete, &QPushButton::clicked, &dlg, [this, table, &items]() {
        int row = table->currentRow();
        if (row < 0 || row >= items.size()) return;
        if (current_role_ == UserRole::Maintainer) return;
        QString path = items[row].path;
        if (QMessageBox::question(this, "删除照片", "确认删除所选照片？", QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
            return;
        }
        if (QFile::remove(path)) {
            items.removeAt(row);
            table->removeRow(row);
        } else {
            QMessageBox::warning(this, "删除失败", "无法删除文件: " + path);
        }
    });

    if (items.size() > 0) {
        table->selectRow(0);
    }

    dlg.exec();
}

void TaskWidget::onExportCSV() {
    int row = task_list_->currentRow();
    if (row < 0 || row >= static_cast<int>(tasks_.size())) {
        QMessageBox::warning(this, "导出 CSV", "请先选择一个任务");
        return;
    }

    QString path = QFileDialog::getSaveFileName(this, "导出任务 CSV", QString(), "CSV Files (*.csv)");
    if (path.isEmpty()) return;

    const auto& task = tasks_[row];

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "导出失败", "无法打开文件: " + path);
        return;
    }

    QTextStream out(&file);
    // Header
    out << "step_name,type,J1,J2,J3,J4,J5,J6,H1,H2,H3,H4,H5,H6,camera_type,io_port,io_value\n";

    for (const auto& step : task.task_seqs) {
        out << QString::fromStdString(step.name) << ","
            << QString::fromStdString(step.type) << ",";

        // Arm positions (J1-J6)
        for (int i = 0; i < 6; ++i) {
            if (i < static_cast<int>(step.arm_pos.size())) {
                out << step.arm_pos[i];
            }
            out << ",";
        }

        // Hand positions (H1-H6)
        for (int i = 0; i < 6; ++i) {
            if (i < static_cast<int>(step.hand_pos.size())) {
                out << step.hand_pos[i];
            }
            if (i < 5) out << ",";
        }

        // Camera type
        out << ",";
        if (!step.camera_type.empty()) {
            out << QString::fromStdString(step.camera_type[0]);
        }
        // IO fields
        out << "," << step.io_port;
        out << "," << (step.io_value ? "HIGH" : "LOW");
        out << "\n";
    }

    file.close();
    QMessageBox::information(this, "导出成功", "CSV 文件已保存到:\n" + path);
}
