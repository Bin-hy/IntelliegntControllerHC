#include "ui_app/auth_manager.hpp"
#include <QCryptographicHash>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDir>
#include <QDateTime>
#include <QRandomGenerator>
#include <QFileInfo>
#include <iostream>
#include <sys/stat.h>

namespace {
QString generateRandomPassword() {
    const QString upper = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    const QString lower = "abcdefghijklmnopqrstuvwxyz";
    const QString digits = "0123456789";
    const QString special = "!@#$%^&*()-_=+";
    const QString all = upper + lower + digits + special;

    QString password;
    // Ensure at least one from each category
    password += upper[QRandomGenerator::global()->bounded(upper.size())];
    password += lower[QRandomGenerator::global()->bounded(lower.size())];
    password += digits[QRandomGenerator::global()->bounded(digits.size())];
    password += special[QRandomGenerator::global()->bounded(special.size())];
    // Fill remaining 8 chars randomly
    for (int i = 0; i < 8; ++i) {
        password += all[QRandomGenerator::global()->bounded(all.size())];
    }
    // Shuffle
    for (int i = password.size() - 1; i > 0; --i) {
        int j = QRandomGenerator::global()->bounded(i + 1);
        QChar tmp = password[i];
        password[i] = password[j];
        password[j] = tmp;
    }
    return password;
}
} // anonymous namespace

AuthManager::AuthManager(const QString& user_file_path)
    : user_file_path_(user_file_path) {
    load();
}

QByteArray AuthManager::generateSalt() const {
    QByteArray salt(16, 0);
    for (int i = 0; i < salt.size(); ++i) {
        salt[i] = static_cast<char>(QRandomGenerator::global()->bounded(0, 256));
    }
    return salt;
}

QByteArray AuthManager::hashPassword(const QString& password,
                                     const QByteArray& salt) const {
    QCryptographicHash hasher(QCryptographicHash::Sha256);
    hasher.addData(salt);
    hasher.addData(password.toUtf8());
    return hasher.result();
}

bool AuthManager::checkComplexity(const QString& password) const {
    if (password.size() < 8) {
        return false;
    }
    bool has_upper = false;
    bool has_lower = false;
    bool has_digit = false;
    bool has_special = false;
    for (QChar c : password) {
        if (c.isUpper()) {
            has_upper = true;
        } else if (c.isLower()) {
            has_lower = true;
        } else if (c.isDigit()) {
            has_digit = true;
        } else {
            has_special = true;
        }
    }
    int count = 0;
    if (has_upper) {
        ++count;
    }
    if (has_lower) {
        ++count;
    }
    if (has_digit) {
        ++count;
    }
    if (has_special) {
        ++count;
    }
    return count >= 3;
}

void AuthManager::load() {
    users_.clear();
    QFile file(user_file_path_);
    if (!file.exists()) {
        InternalUser admin;
        admin.username = "admin";
        QByteArray salt = generateSalt();
        QString default_password = "admin@123";
        admin.salt = salt;
        admin.password_hash = hashPassword(default_password, salt);
        std::cout << "========================================" << std::endl;
        std::cout << "  FIRST-TIME SETUP" << std::endl;
        std::cout << "  Default admin password: " << default_password.toStdString() << std::endl;
        std::cout << "  Please change it after first login." << std::endl;
        std::cout << "========================================" << std::endl;
        admin.role = UserRole::Admin;
        admin.failed_attempts = 0;
        admin.lock_until = 0;
        admin.must_change_password = true;
        admin.disabled = false;
        users_.insert(admin.username.toLower(), admin);
        save();
        return;
    }
    if (!file.open(QIODevice::ReadOnly)) {
        return;
    }
    QByteArray data = file.readAll();
    file.close();
    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (!doc.isArray()) {
        return;
    }
    QJsonArray arr = doc.array();
    for (const QJsonValue& v : arr) {
        if (!v.isObject()) {
            continue;
        }
        QJsonObject obj = v.toObject();
        InternalUser u;
        u.username = obj.value("username").toString();
        QByteArray salt = QByteArray::fromHex(obj.value("salt").toString().toUtf8());
        QByteArray hash = QByteArray::fromHex(obj.value("password_hash").toString().toUtf8());
        u.salt = salt;
        u.password_hash = hash;
        QString role_str = obj.value("role").toString();
        if (role_str == "operator") {
            u.role = UserRole::Operator;
        } else if (role_str == "maintainer") {
            u.role = UserRole::Maintainer;
        } else {
            u.role = UserRole::Admin;
        }
        u.failed_attempts = obj.value("failed_attempts").toInt();
        u.lock_until = static_cast<qint64>(obj.value("lock_until").toDouble());
        u.must_change_password = obj.value("must_change_password").toBool();
        u.disabled = obj.value("disabled").toBool();
        users_.insert(u.username.toLower(), u);
    }
}

void AuthManager::save() const {
    QDir dir = QFileInfo(user_file_path_).dir();
    if (!dir.exists()) {
        dir.mkpath(".");
    }
    // Harden directory permissions to 700 (owner only)
    ::chmod(dir.absolutePath().toLocal8Bit().constData(), 0700);
    QJsonArray arr;
    for (const InternalUser& u : users_) {
        QJsonObject obj;
        obj.insert("username", u.username);
        QString role_str = "admin";
        if (u.role == UserRole::Operator) {
            role_str = "operator";
        } else if (u.role == UserRole::Maintainer) {
            role_str = "maintainer";
        }
        obj.insert("role", role_str);
        obj.insert("salt", QString::fromUtf8(u.salt.toHex()));
        obj.insert("password_hash", QString::fromUtf8(u.password_hash.toHex()));
        obj.insert("failed_attempts", u.failed_attempts);
        obj.insert("lock_until", static_cast<double>(u.lock_until));
        obj.insert("must_change_password", u.must_change_password);
        obj.insert("disabled", u.disabled);
        arr.append(obj);
    }
    QJsonDocument doc(arr);
    QFile file(user_file_path_);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return;
    }
    file.write(doc.toJson(QJsonDocument::Compact));
    file.close();
    // Harden file permissions to 600 (owner read/write only)
    ::chmod(user_file_path_.toLocal8Bit().constData(), 0600);
}

bool AuthManager::verifyPassword(const QString& username,
                                 const QString& password,
                                 UserInfo& out_user,
                                 QString& out_error_reason) {
    QString key = username.toLower();
    if (!users_.contains(key)) {
        out_error_reason = "用户不存在";
        return false;
    }
    InternalUser& u = users_[key];

    // Always populate out_user so the caller can inspect state even on failure
    auto populate_out = [&]() {
        out_user.username = u.username;
        out_user.role = u.role;
        out_user.failed_attempts = u.failed_attempts;
        out_user.lock_until = u.lock_until;
        out_user.must_change_password = u.must_change_password;
        out_user.disabled = u.disabled;
    };

    qint64 now = QDateTime::currentSecsSinceEpoch();
    if (u.disabled) {
        populate_out();
        out_error_reason = "账号已禁用";
        return false;
    }
    if (u.lock_until > now) {
        populate_out();
        out_error_reason = "账号已锁定";
        return false;
    }
    QByteArray h = hashPassword(password, u.salt);
    if (h == u.password_hash) {
        u.failed_attempts = 0;
        u.lock_until = 0;
        save();
        populate_out();
        out_error_reason.clear();
        return true;
    }
    ++u.failed_attempts;
    if (u.failed_attempts >= 3) {   // Lock after 3 consecutive failures
        u.lock_until = now + 10 * 60;
        u.failed_attempts = 0;
    }
    save();
    populate_out();
    out_error_reason = "密码错误";
    return false;
}

bool AuthManager::changePassword(const QString& username,
                                 const QString& old_password,
                                 const QString& new_password,
                                 QString& out_error) {
    QString key = username.toLower();
    if (!users_.contains(key)) {
        out_error = "用户不存在";
        return false;
    }
    InternalUser& u = users_[key];
    QByteArray old_hash = hashPassword(old_password, u.salt);
    if (old_hash != u.password_hash) {
        out_error = "原密码错误";
        return false;
    }
    if (!checkComplexity(new_password)) {
        out_error = "新密码不满足复杂度要求";
        return false;
    }
    QByteArray salt = generateSalt();
    QByteArray new_hash = hashPassword(new_password, salt);
    u.salt = salt;
    u.password_hash = new_hash;
    u.must_change_password = false;
    save();
    out_error.clear();
    return true;
}

bool AuthManager::adminResetPassword(const QString& username,
                                     const QString& new_password,
                                     bool force_change,
                                     QString& out_error) {
    QString key = username.toLower();
    if (!users_.contains(key)) {
        out_error = "用户不存在";
        return false;
    }
    if (!checkComplexity(new_password)) {
        out_error = "新密码不满足复杂度要求";
        return false;
    }
    InternalUser& u = users_[key];
    QByteArray salt = generateSalt();
    QByteArray new_hash = hashPassword(new_password, salt);
    u.salt = salt;
    u.password_hash = new_hash;
    u.must_change_password = force_change;
    u.failed_attempts = 0;
    u.lock_until = 0;
    save();
    out_error.clear();
    return true;
}

bool AuthManager::createUser(const QString& username,
                             const QString& password,
                             UserRole role,
                             QString& out_error) {
    QString key = username.toLower();
    if (users_.contains(key)) {
        out_error = "用户已存在";
        return false;
    }
    if (!checkComplexity(password)) {
        out_error = "密码不满足复杂度要求";
        return false;
    }
    InternalUser u;
    u.username = username;
    u.role = role;
    u.failed_attempts = 0;
    u.lock_until = 0;
    u.must_change_password = false;
    u.disabled = false;
    u.salt = generateSalt();
    u.password_hash = hashPassword(password, u.salt);
    users_.insert(key, u);
    save();
    out_error.clear();
    return true;
}

bool AuthManager::deleteUser(const QString& username, QString& out_error) {
    QString key = username.toLower();
    if (!users_.contains(key)) {
        out_error = "用户不存在";
        return false;
    }
    users_.remove(key);
    save();
    out_error.clear();
    return true;
}

bool AuthManager::renameUser(const QString& old_name,
                             const QString& new_name,
                             QString& out_error) {
    QString old_key = old_name.toLower();
    QString new_key = new_name.toLower();
    if (!users_.contains(old_key)) {
        out_error = "原用户不存在";
        return false;
    }
    if (users_.contains(new_key)) {
        out_error = "新用户名已存在";
        return false;
    }
    InternalUser u = users_.value(old_key);
    users_.remove(old_key);
    u.username = new_name;
    users_.insert(new_key, u);
    save();
    out_error.clear();
    return true;
}

bool AuthManager::setLocked(const QString& username, bool locked, QString& out_error) {
    QString key = username.toLower();
    if (!users_.contains(key)) {
        out_error = "用户不存在";
        return false;
    }
    InternalUser& u = users_[key];
    qint64 now = QDateTime::currentSecsSinceEpoch();
    if (locked) {
        u.lock_until = now + 10 * 60;
    } else {
        u.lock_until = 0;
        u.failed_attempts = 0;
    }
    save();
    out_error.clear();
    return true;
}

bool AuthManager::listUsers(QVector<UserInfo>& users) const {
    users.clear();
    for (const InternalUser& u : users_) {
        UserInfo info;
        info.username = u.username;
        info.role = u.role;
        info.failed_attempts = u.failed_attempts;
        info.lock_until = u.lock_until;
        info.must_change_password = u.must_change_password;
        info.disabled = u.disabled;
        users.append(info);
    }
    return true;
}
