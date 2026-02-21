#include "ui_app/auth_log_manager.hpp"
#include <QDir>
#include <QFile>
#include <QTextStream>

AuthLogManager::AuthLogManager(const QString& log_dir)
    : log_dir_(log_dir) {
    QDir dir(log_dir_);
    if (!dir.exists()) {
        dir.mkpath(".");
    }
    cleanupOldLogs(6);
}

QString AuthLogManager::currentLogFilePath() const {
    QDateTime now = QDateTime::currentDateTimeUtc();
    QString name = QString("auth_%1_%2.log")
                       .arg(now.date().year())
                       .arg(now.date().month(), 2, 10, QLatin1Char('0'));
    QDir dir(log_dir_);
    return dir.filePath(name);
}

void AuthLogManager::logSuccess(const QString& username) {
    QFile file(currentLogFilePath());
    if (!file.open(QIODevice::Append | QIODevice::Text)) {
        return;
    }
    QTextStream out(&file);
    QDateTime now = QDateTime::currentDateTimeUtc();
    out << now.toString(Qt::ISODate) << " "
        << username << " "
        << "success"
        << " "
        << "-"
        << " "
        << "local"
        << "\n";
    file.close();
}

void AuthLogManager::logFailure(const QString& username, const QString& reason) {
    QFile file(currentLogFilePath());
    if (!file.open(QIODevice::Append | QIODevice::Text)) {
        return;
    }
    QTextStream out(&file);
    QDateTime now = QDateTime::currentDateTimeUtc();
    QString sanitized = reason;
    sanitized.replace("\n", " ");
    out << now.toString(Qt::ISODate) << " "
        << username << " "
        << "failure"
        << " "
        << sanitized
        << " "
        << "local"
        << "\n";
    file.close();
}

QVector<AuthLogEntry> AuthLogManager::loadLogs(const QDateTime& from,
                                               const QDateTime& to,
                                               const QString& user_filter,
                                               bool success_only,
                                               bool failure_only) const {
    QVector<AuthLogEntry> result;
    QDir dir(log_dir_);
    QStringList files = dir.entryList(QStringList() << "auth_*.log",
                                      QDir::Files | QDir::Readable,
                                      QDir::Name);
    for (const QString& name : files) {
        QFile file(dir.filePath(name));
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            continue;
        }
        QTextStream in(&file);
        while (!in.atEnd()) {
            QString line = in.readLine().trimmed();
            if (line.isEmpty()) {
                continue;
            }
            QStringList parts = line.split(' ');
            if (parts.size() < 5) {
                continue;
            }
            AuthLogEntry e;
            e.timestamp = QDateTime::fromString(parts[0], Qt::ISODate);
            e.username = parts[1];
            QString res = parts[2];
            e.success = (res == "success");
            e.reason = parts[3];
            e.source = parts[4];
            if (from.isValid() && e.timestamp < from) {
                continue;
            }
            if (to.isValid() && e.timestamp > to) {
                continue;
            }
            if (!user_filter.isEmpty() && e.username != user_filter) {
                continue;
            }
            if (success_only && !e.success) {
                continue;
            }
            if (failure_only && e.success) {
                continue;
            }
            result.append(e);
        }
        file.close();
    }
    return result;
}

bool AuthLogManager::deleteLogs(const QDateTime& before) {
    QDir dir(log_dir_);
    QStringList files = dir.entryList(QStringList() << "auth_*.log",
                                      QDir::Files | QDir::Readable,
                                      QDir::Name);
    bool ok = true;
    for (const QString& name : files) {
        QFile file(dir.filePath(name));
        QFileInfo info(file);
        QDateTime t = info.lastModified();
        if (t < before) {
            if (!file.remove()) {
                ok = false;
            }
        }
    }
    return ok;
}

void AuthLogManager::cleanupOldLogs(int months_to_keep) {
    QDateTime now = QDateTime::currentDateTimeUtc();
    QDateTime threshold = now.addMonths(-months_to_keep);
    deleteLogs(threshold);
}
