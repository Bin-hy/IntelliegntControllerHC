#ifndef AUTH_MANAGER_HPP
#define AUTH_MANAGER_HPP

#include <QString>
#include <QByteArray>
#include <QHash>
#include <QVector>

enum class UserRole {
    Operator,
    Maintainer,
    Admin
};

struct UserInfo {
    QString username;
    UserRole role;
    int failed_attempts;
    qint64 lock_until;
    bool must_change_password;
    bool disabled;
};

class AuthManager {
public:
    explicit AuthManager(const QString& user_file_path);

    bool verifyPassword(const QString& username,
                        const QString& password,
                        UserInfo& out_user,
                        QString& out_error_reason);

    bool changePassword(const QString& username,
                        const QString& old_password,
                        const QString& new_password,
                        QString& out_error);

    bool adminResetPassword(const QString& username,
                            const QString& new_password,
                            bool force_change,
                            QString& out_error);

    bool createUser(const QString& username,
                    const QString& password,
                    UserRole role,
                    QString& out_error);

    bool deleteUser(const QString& username, QString& out_error);
    bool renameUser(const QString& old_name,
                    const QString& new_name,
                    QString& out_error);

    bool setLocked(const QString& username, bool locked, QString& out_error);
    bool listUsers(QVector<UserInfo>& users) const;

private:
    struct InternalUser {
        QString username;
        QByteArray salt;
        QByteArray password_hash;
        UserRole role;
        int failed_attempts;
        qint64 lock_until;
        bool must_change_password;
        bool disabled;
    };

    QHash<QString, InternalUser> users_;
    QString user_file_path_;

    void load();
    void save() const;

    QByteArray generateSalt() const;
    QByteArray hashPassword(const QString& password,
                            const QByteArray& salt) const;
    bool checkComplexity(const QString& password) const;
};

#endif
