#ifndef TASK_RECORD_MANAGER_HPP
#define TASK_RECORD_MANAGER_HPP

#include <QString>
#include <QVector>
#include <QDateTime>
#include <QJsonObject>

struct StepRecord {
    QString name;
    QString type;
    bool success;
    int duration_ms;
    QStringList captured_files;

    QJsonObject toJson() const;
    static StepRecord fromJson(const QJsonObject& obj);
};

struct TaskExecutionRecord {
    QString task_name;
    QDateTime start_time;
    QDateTime end_time;
    bool success;
    QString error_msg;
    QVector<StepRecord> steps;

    QJsonObject toJson() const;
    static TaskExecutionRecord fromJson(const QJsonObject& obj);
};

class TaskRecordManager {
public:
    explicit TaskRecordManager(const QString& records_dir);

    void saveRecord(const TaskExecutionRecord& record);
    QVector<TaskExecutionRecord> loadAllRecords() const;
    bool deleteRecord(const QString& filename);
    void cleanupOldRecords(int months_to_keep);

private:
    QString records_dir_;
    void ensureDir() const;
};

#endif // TASK_RECORD_MANAGER_HPP
