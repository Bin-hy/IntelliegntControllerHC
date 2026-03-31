#include "ui_app/task_record_manager.hpp"
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>

QJsonObject StepRecord::toJson() const {
    QJsonObject obj;
    obj["name"] = name;
    obj["type"] = type;
    obj["success"] = success;
    obj["duration_ms"] = duration_ms;
    QJsonArray files;
    for (const auto& f : captured_files) {
        files.append(f);
    }
    obj["captured_files"] = files;
    if (vision_data.has_data) {
        QJsonObject vd;
        vd["angle_deg"] = vision_data.angle_deg;
        vd["depth_mm"] = vision_data.depth_mm;
        vd["confidence"] = vision_data.confidence;
        vd["message"] = vision_data.message;
        obj["vision_data"] = vd;
    }
    return obj;
}

StepRecord StepRecord::fromJson(const QJsonObject& obj) {
    StepRecord r;
    r.name = obj["name"].toString();
    r.type = obj["type"].toString();
    r.success = obj["success"].toBool();
    r.duration_ms = obj["duration_ms"].toInt();
    QJsonArray files = obj["captured_files"].toArray();
    for (const auto& f : files) {
        r.captured_files.append(f.toString());
    }
    if (obj.contains("vision_data")) {
        QJsonObject vd = obj["vision_data"].toObject();
        r.vision_data.has_data = true;
        r.vision_data.angle_deg = vd["angle_deg"].toDouble();
        r.vision_data.depth_mm = vd["depth_mm"].toDouble();
        r.vision_data.confidence = vd["confidence"].toDouble();
        r.vision_data.message = vd["message"].toString();
    }
    return r;
}

QJsonObject TaskExecutionRecord::toJson() const {
    QJsonObject obj;
    obj["task_name"] = task_name;
    obj["start_time"] = start_time.toString(Qt::ISODate);
    obj["end_time"] = end_time.toString(Qt::ISODate);
    obj["success"] = success;
    obj["error_msg"] = error_msg;
    QJsonArray step_arr;
    for (const auto& s : steps) {
        step_arr.append(s.toJson());
    }
    obj["steps"] = step_arr;
    return obj;
}

TaskExecutionRecord TaskExecutionRecord::fromJson(const QJsonObject& obj) {
    TaskExecutionRecord r;
    r.task_name = obj["task_name"].toString();
    r.start_time = QDateTime::fromString(obj["start_time"].toString(), Qt::ISODate);
    r.end_time = QDateTime::fromString(obj["end_time"].toString(), Qt::ISODate);
    r.success = obj["success"].toBool();
    r.error_msg = obj["error_msg"].toString();
    QJsonArray step_arr = obj["steps"].toArray();
    for (const auto& v : step_arr) {
        r.steps.append(StepRecord::fromJson(v.toObject()));
    }
    return r;
}

TaskRecordManager::TaskRecordManager(const QString& records_dir)
    : records_dir_(records_dir) {
    ensureDir();
}

void TaskRecordManager::ensureDir() const {
    QDir dir(records_dir_);
    if (!dir.exists()) {
        dir.mkpath(".");
    }
}

void TaskRecordManager::saveRecord(const TaskExecutionRecord& record) {
    ensureDir();
    QString filename = QString("record_%1.json")
        .arg(record.start_time.toString("yyyyMMdd_HHmmss"));
    QDir dir(records_dir_);
    QString path = dir.filePath(filename);

    QJsonDocument doc(record.toJson());
    QFile file(path);
    if (file.open(QIODevice::WriteOnly)) {
        file.write(doc.toJson());
        file.close();
    }
}

QVector<TaskExecutionRecord> TaskRecordManager::loadAllRecords() const {
    QVector<TaskExecutionRecord> records;
    QDir dir(records_dir_);
    QStringList files = dir.entryList(QStringList() << "record_*.json",
                                      QDir::Files | QDir::Readable,
                                      QDir::Name | QDir::Reversed);
    for (const QString& name : files) {
        QFile file(dir.filePath(name));
        if (!file.open(QIODevice::ReadOnly)) continue;
        QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
        file.close();
        if (doc.isObject()) {
            records.append(TaskExecutionRecord::fromJson(doc.object()));
        }
    }
    return records;
}

bool TaskRecordManager::deleteRecord(const QString& filename) {
    QDir dir(records_dir_);
    return QFile::remove(dir.filePath(filename));
}

void TaskRecordManager::cleanupOldRecords(int months_to_keep) {
    QDateTime threshold = QDateTime::currentDateTimeUtc().addMonths(-months_to_keep);
    QDir dir(records_dir_);
    QStringList files = dir.entryList(QStringList() << "record_*.json",
                                      QDir::Files | QDir::Readable);
    for (const QString& name : files) {
        QFile file(dir.filePath(name));
        QFileInfo info(file);
        if (info.lastModified() < threshold) {
            file.remove();
        }
    }
}
