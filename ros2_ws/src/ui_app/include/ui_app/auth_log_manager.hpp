#ifndef AUTH_LOG_MANAGER_HPP
#define AUTH_LOG_MANAGER_HPP

#include <QString>
#include <QVector>
#include <QDateTime>

struct AuthLogEntry {
    QDateTime timestamp;
    QString username;
    bool success;
    QString reason;
    QString source;
};

class AuthLogManager {
public:
    explicit AuthLogManager(const QString& log_dir);

    void logSuccess(const QString& username);
    void logFailure(const QString& username, const QString& reason);

    QVector<AuthLogEntry> loadLogs(const QDateTime& from,
                                   const QDateTime& to,
                                   const QString& user_filter,
                                   bool success_only,
                                   bool failure_only) const;

    bool deleteLogs(const QDateTime& before);
    void cleanupOldLogs(int months_to_keep = 6);

private:
    QString log_dir_;
    QString currentLogFilePath() const;
};

#endif
