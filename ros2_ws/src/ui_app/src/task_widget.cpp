#include "ui_app/task_widget.hpp"
#include "ui_app/i18n_manager.hpp"
#include "ui_app/task_dialogs.hpp"
#include "ui_app/i18n_manager.hpp"
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
#include <QRegularExpression>
#include <iostream>
#include "common_msgs/msg/task_config.hpp"
// #include "ament_index_cpp/get_package_share_directory.hpp"

TaskWidget::TaskWidget(std::shared_ptr<RosNode> node, const QString& username, UserRole role,
                       PermissionManager* permission_manager, QWidget *parent)
    : QWidget(parent), node_(node), current_user_(username), current_role_(role),
      permission_manager_(permission_manager) {
    // Initialize record manager
    QDir records_dir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation));
    record_manager_ = std::make_shared<TaskRecordManager>(records_dir.filePath("task_records"));

    auto * layout = new QVBoxLayout(this);

    layout->addWidget(new QLabel(tr_ui("任务列表", "Task List")));
    task_list_ = new QListWidget();
    layout->addWidget(task_list_);

    auto * h_btn = new QHBoxLayout();
    auto * btn_add = new QPushButton(tr_ui("新建任务", "New Task"));
    auto * btn_edit = new QPushButton(tr_ui("编辑任务", "Edit Task"));
    auto * btn_del = new QPushButton(tr_ui("删除任务", "Delete Task"));
    auto * btn_run = new QPushButton(tr_ui("运行任务", "Run Task"));
    auto * btn_history = new QPushButton(tr_ui("历史记录", "History"));
    auto * btn_photos = new QPushButton(tr_ui("照片管理", "Photos"));
    auto * btn_export = new QPushButton(tr_ui("导出 CSV", "Export CSV"));

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
    if (permission_manager_ &&
        !permission_manager_->hasPermission(current_role_, ActionType::ManageTask)) {
        QMessageBox::warning(this, tr_ui("权限不足", "Insufficient Permission"),
                             tr_ui("当前用户无权创建任务", "Insufficient permission to create tasks"));
        return;
    }
    TaskConfigDialog dlg(node_, this);
    dlg.setAllTasks(tasks_);
    if (dlg.exec() == QDialog::Accepted) {
        tasks_.push_back(dlg.getTask());
        updateTaskList();
        saveTasks();
    }
}

void TaskWidget::onEditTask() {
    if (permission_manager_ &&
        !permission_manager_->hasPermission(current_role_, ActionType::ManageTask)) {
        QMessageBox::warning(this, tr_ui("权限不足", "Insufficient Permission"),
                             tr_ui("当前用户无权编辑任务", "Insufficient permission to edit tasks"));
        return;
    }
    int row = task_list_->currentRow();
    if (row >= 0 && row < (int)tasks_.size()) {
        TaskConfigDialog dlg(node_, this);
        dlg.setAllTasks(tasks_);
        dlg.setTask(tasks_[row]);
        if (dlg.exec() == QDialog::Accepted) {
            tasks_[row] = dlg.getTask();
            updateTaskList();
            saveTasks();
        }
    }
}

void TaskWidget::onDeleteTask() {
    if (permission_manager_ &&
        !permission_manager_->hasPermission(current_role_, ActionType::ManageTask)) {
        QMessageBox::warning(this, tr_ui("权限不足", "Insufficient Permission"),
                             tr_ui("当前用户无权删除任务", "Insufficient permission to delete tasks"));
        return;
    }
    int row = task_list_->currentRow();
    if (row >= 0 && row < (int)tasks_.size()) {
        if (QMessageBox::question(this, tr_ui("删除任务", "Delete Task"), tr_ui("确认删除此任务？", "Are you sure you want to delete this task?"), QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes) {
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
        QString info = QString(tr_ui("步骤: %1 | 设备: %2 | 轮次: %3", "Steps: %1 | Devices: %2 | Rounds: %3"))
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

            stepObj["arm_command"] = QString::fromStdString(step.arm_command);
            QJsonArray armCartPos;
            for (double p : step.arm_cart_pos) armCartPos.append(p);
            stepObj["arm_cart_pos"] = armCartPos;
            stepObj["arm_velocity"] = step.arm_velocity;
            stepObj["arm_accel"] = step.arm_accel;

            QJsonArray handPos;
            for (int p : step.hand_pos) handPos.append(p);
            stepObj["hand_pos"] = handPos;
            
            QJsonArray camTypes;
            for (const auto& c : step.camera_type) camTypes.append(QString::fromStdString(c));
            stepObj["camera_type"] = camTypes;

            stepObj["io_type"] = step.io_type;
            stepObj["io_port"] = step.io_port;
            stepObj["io_value"] = step.io_value;
            stepObj["lift_command"] = QString::fromStdString(step.lift_command);
            stepObj["lift_speed_rpm"] = step.lift_speed_rpm;
            stepObj["lift_target_pulses"] = step.lift_target_pulses;
            stepObj["lift_accel_ms"] = step.lift_accel_ms;
            stepObj["lift_decel_ms"] = step.lift_decel_ms;
            stepObj["delay_ms"] = step.delay_ms;
            stepObj["control_target"]  = QString::fromStdString(step.control_target);
            stepObj["control_command"] = QString::fromStdString(step.control_command);

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

            step.arm_command = sObj["arm_command"].toString().toStdString();
            QJsonArray armCartPos = sObj["arm_cart_pos"].toArray();
            for (const auto& p : armCartPos) step.arm_cart_pos.push_back(p.toDouble());
            step.arm_velocity = sObj["arm_velocity"].toDouble(0);
            step.arm_accel = sObj["arm_accel"].toDouble(0);

            QJsonArray handPos = sObj["hand_pos"].toArray();
            for (const auto& p : handPos) step.hand_pos.push_back(p.toInt());
            
            if (sObj["camera_type"].isArray()) {
                QJsonArray cams = sObj["camera_type"].toArray();
                for (const auto& c : cams) step.camera_type.push_back(c.toString().toStdString());
            } else {
                 // Fallback for legacy single string
                 step.camera_type.push_back(sObj["camera_type"].toString().toStdString());
            }

            step.io_type = static_cast<int8_t>(sObj["io_type"].toInt(0));
            step.io_port = static_cast<int8_t>(sObj["io_port"].toInt(0));
            step.io_value = sObj["io_value"].toBool(false);
            step.lift_command = sObj["lift_command"].toString().toStdString();
            step.lift_speed_rpm = sObj["lift_speed_rpm"].toInt(0);
            step.lift_target_pulses = sObj["lift_target_pulses"].toInt(0);
            step.lift_accel_ms = sObj["lift_accel_ms"].toInt(0);
            step.lift_decel_ms = sObj["lift_decel_ms"].toInt(0);
            step.delay_ms = sObj["delay_ms"].toInt(0);
            step.control_target  = sObj["control_target"].toString().toStdString();
            step.control_command = sObj["control_command"].toString().toStdString();

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
    dlg.setWindowTitle(tr_ui("任务执行历史", "Task Execution History"));
    dlg.resize(1100, 600);
    auto * layout = new QHBoxLayout(&dlg);

    // --- Left: task list ---
    auto * left = new QVBoxLayout();
    left->addWidget(new QLabel(tr_ui("任务列表", "Task List")));

    auto * table = new QTableWidget();
    table->setColumnCount(4);
    table->setHorizontalHeaderLabels({tr_ui("任务名称", "Task Name"), tr_ui("开始时间", "Start Time"), tr_ui("结果", "Result"), tr_ui("错误信息", "Error Msg")});
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setRowCount(records.size());

    for (int i = 0; i < records.size(); ++i) {
        const auto& r = records[i];
        table->setItem(i, 0, new QTableWidgetItem(r.task_name));
        table->setItem(i, 1, new QTableWidgetItem(r.start_time.toLocalTime().toString("yyyy-MM-dd HH:mm:ss")));
        auto * result_item = new QTableWidgetItem(r.success ? tr_ui("成功", "Success") : tr_ui("失败", "Failure"));
        result_item->setForeground(r.success ? Qt::darkGreen : Qt::red);
        table->setItem(i, 2, result_item);
        table->setItem(i, 3, new QTableWidgetItem(r.error_msg));
    }
    left->addWidget(table);
    layout->addLayout(left, 1);

    // --- Right: step details + photo preview ---
    auto * right = new QVBoxLayout();
    right->addWidget(new QLabel(tr_ui("步骤详情", "Step Details")));

    auto * step_table = new QTableWidget();
    step_table->setColumnCount(6);
    step_table->setHorizontalHeaderLabels({tr_ui("步骤", "Step"), tr_ui("类型", "Type"), tr_ui("结果", "Result"),
                                            tr_ui("耳机角度", "Angle"), tr_ui("耳机深度", "Depth"), tr_ui("置信度", "Conf")});
    step_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    step_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    step_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    right->addWidget(step_table);

    right->addWidget(new QLabel(tr_ui("拍摄照片", "Captured Photos")));
    auto * photo_list = new QListWidget();
    right->addWidget(photo_list);

    auto * preview = new QLabel();
    preview->setMinimumSize(320, 240);
    preview->setAlignment(Qt::AlignCenter);
    right->addWidget(preview, 1);

    layout->addLayout(right, 2);

    // When task row changes, update step details
    QObject::connect(table, &QTableWidget::itemSelectionChanged, &dlg, [&]() {
        step_table->setRowCount(0);
        photo_list->clear();
        preview->clear();
        int row = table->currentRow();
        if (row < 0 || row >= records.size()) return;
        const auto& rec = records[row];

        step_table->setRowCount(rec.steps.size());
        for (int i = 0; i < rec.steps.size(); ++i) {
            const auto& s = rec.steps[i];
            step_table->setItem(i, 0, new QTableWidgetItem(s.name));
            step_table->setItem(i, 1, new QTableWidgetItem(s.type));
            auto * si = new QTableWidgetItem(s.success ? tr_ui("成功", "OK") : tr_ui("失败", "Fail"));
            si->setForeground(s.success ? Qt::darkGreen : Qt::red);
            step_table->setItem(i, 2, si);
            if (s.vision_data.has_data) {
                step_table->setItem(i, 3, new QTableWidgetItem(QString::number(s.vision_data.angle_deg, 'f', 1) + QString::fromUtf8("°")));
                step_table->setItem(i, 4, new QTableWidgetItem(QString::number(s.vision_data.depth_mm, 'f', 1) + " mm"));
                step_table->setItem(i, 5, new QTableWidgetItem(QString::number(s.vision_data.confidence, 'f', 2)));
            } else {
                step_table->setItem(i, 3, new QTableWidgetItem("-"));
                step_table->setItem(i, 4, new QTableWidgetItem("-"));
                step_table->setItem(i, 5, new QTableWidgetItem("-"));
            }
        }

        // Collect all captured files from all steps + scan task_photos directory
        QStringList all_files;
        for (const auto& s : rec.steps) {
            all_files.append(s.captured_files);
        }
        // Also scan the task_photos directory for matching files
        QString base_dir = QDir::homePath() + "/.ros/task_photos";
        QDir photos_dir(base_dir);
        if (photos_dir.exists()) {
            QString task_tag = rec.task_name;
            task_tag.replace(QRegularExpression("[^a-zA-Z0-9_-]"), "_");
            QDirIterator dit(base_dir, QStringList() << "*.png", QDir::Files, QDirIterator::Subdirectories);
            while (dit.hasNext()) {
                QString path = dit.next();
                if (path.contains(task_tag) && !all_files.contains(path)) {
                    all_files.append(path);
                }
            }
        }
        for (const auto& f : all_files) {
            photo_list->addItem(f);
        }
    });

    // When photo is selected, show preview
    QObject::connect(photo_list, &QListWidget::currentItemChanged, &dlg, [preview](QListWidgetItem* item) {
        if (!item) { preview->clear(); return; }
        QPixmap pix(item->text());
        if (!pix.isNull()) {
            preview->setPixmap(pix.scaled(preview->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        } else {
            preview->setText(tr_ui("无法加载图片", "Cannot load image"));
        }
    });

    auto * buttons = new QDialogButtonBox(QDialogButtonBox::Close);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    // Add to the right side
    right->addWidget(buttons);

    if (records.size() > 0) {
        table->selectRow(0);
    }

    dlg.exec();
}

void TaskWidget::onShowPhotos() {
    QString base_dir = QDir::homePath() + "/.ros/task_photos";
    QDir root(base_dir);
    if (!root.exists()) {
        QMessageBox::information(this, tr_ui("照片管理", "Photos"), tr_ui("未找到照片目录: ", "Photo directory not found: ") + base_dir);
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
    dlg.setWindowTitle(tr_ui("照片管理", "Photos"));
    dlg.resize(900, 500);
    auto * layout = new QHBoxLayout(&dlg);

    auto * table = new QTableWidget();
    table->setColumnCount(5);
    table->setHorizontalHeaderLabels({tr_ui("任务", "Task"), tr_ui("时间", "Time"), tr_ui("轮次", "Round"), tr_ui("用户", "User"), tr_ui("文件", "File")});
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

    auto * btn_delete = new QPushButton(tr_ui("删除照片", "Delete Photo"));
    bool can_delete = permission_manager_ &&
                      permission_manager_->hasPermission(current_role_, ActionType::DeletePhoto);
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
            preview->setText(tr_ui("无法加载图片", "Cannot load image"));
        }
    };

    connect(table, &QTableWidget::itemSelectionChanged, &dlg, refresh_preview);

    connect(btn_delete, &QPushButton::clicked, &dlg, [this, table, &items]() {
        int row = table->currentRow();
        if (row < 0 || row >= items.size()) return;
        if (!permission_manager_ ||
            !permission_manager_->hasPermission(current_role_, ActionType::DeletePhoto)) return;
        QString path = items[row].path;
        if (QMessageBox::question(this, tr_ui("删除照片", "Delete Photo"), tr_ui("确认删除所选照片？", "Are you sure to delete selected photo?"), QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
            return;
        }
        if (QFile::remove(path)) {
            items.removeAt(row);
            table->removeRow(row);
        } else {
            QMessageBox::warning(this, tr_ui("删除失败", "Delete Failed"), tr_ui("无法删除文件: ", "Cannot delete file: ") + path);
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
        QMessageBox::warning(this, tr_ui("导出 CSV", "Export CSV"), tr_ui("请先选择一个任务", "Please select a task first"));
        return;
    }

    QString path = QFileDialog::getSaveFileName(this, tr_ui("导出任务 CSV", "Export Task CSV"), QString(), "CSV Files (*.csv)");
    if (path.isEmpty()) return;

    const auto& task = tasks_[row];

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, tr_ui("导出失败", "Export Failed"), tr_ui("无法打开文件: ", "Cannot open file: ") + path);
        return;
    }

    QTextStream out(&file);
    // Header
    out << "step_name,type,J1,J2,J3,J4,J5,J6,H1,H2,H3,H4,H5,H6,camera_type,io_type,io_port,io_value,lift_command,lift_speed_rpm\n";

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
        out << "," << (step.io_type == 1 ? "tool" : "standard");
        out << "," << step.io_port;
        out << "," << (step.io_value ? "HIGH" : "LOW");
        // Lift fields
        out << "," << QString::fromStdString(step.lift_command);
        out << "," << step.lift_speed_rpm;
        out << "\n";
    }

    file.close();
    QMessageBox::information(this, tr_ui("导出成功", "Export Success"), tr_ui("CSV 文件已保存到:\n", "CSV file saved to:\n") + path);
}
