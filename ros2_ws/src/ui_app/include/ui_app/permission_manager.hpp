#ifndef PERMISSION_MANAGER_HPP
#define PERMISSION_MANAGER_HPP

#include <QHash>
#include <QSet>
#include <QString>

#include "ui_app/auth_manager.hpp"

enum class ActionType {
    PowerOn,
    PowerOff,
    StartTask,
    StopTask,
    ModifyParam,
    CalibrateSystem,
    ViewAuthLog,
    DeleteAuthLog,
    CreateUser,
    ModifyUser,
    DeleteUser,
    ResetUserPassword,
    LockUnlockUser,
    ManageTask,
    DeletePhoto
};

inline uint qHash(ActionType key, uint seed = 0) noexcept {
    return qHash(static_cast<int>(key), seed);
}

inline uint qHash(UserRole key, uint seed = 0) noexcept {
    return qHash(static_cast<int>(key), seed);
}

class PermissionManager {
public:
    explicit PermissionManager(const QString& policy_path);

    bool hasPermission(UserRole role, ActionType action) const;
    UserRole roleFromString(const QString& s) const;
    QString roleToString(UserRole role) const;

private:
    QHash<UserRole, QSet<ActionType>> role_permissions_;

    void loadPolicy(const QString& policy_path);
};

#endif
