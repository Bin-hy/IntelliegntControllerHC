#include "ui_app/permission_manager.hpp"
#include <QFile>
#include <QTextStream>

PermissionManager::PermissionManager(const QString& policy_path) {
    loadPolicy(policy_path);
}

UserRole PermissionManager::roleFromString(const QString& s) const {
    QString v = s.toLower();
    if (v == "operator") {
        return UserRole::Operator;
    }
    if (v == "maintainer") {
        return UserRole::Maintainer;
    }
    return UserRole::Admin;
}

QString PermissionManager::roleToString(UserRole role) const {
    if (role == UserRole::Operator) {
        return "operator";
    }
    if (role == UserRole::Maintainer) {
        return "maintainer";
    }
    return "admin";
}

bool PermissionManager::hasPermission(UserRole role, ActionType action) const {
    if (!role_permissions_.contains(role)) {
        return false;
    }
    return role_permissions_.value(role).contains(action);
}

void PermissionManager::loadPolicy(const QString& policy_path) {
    QSet<ActionType> operator_actions;
    QSet<ActionType> maintainer_actions;
    QSet<ActionType> admin_actions;

    operator_actions.insert(ActionType::PowerOn);
    operator_actions.insert(ActionType::PowerOff);
    operator_actions.insert(ActionType::StartTask);
    operator_actions.insert(ActionType::StopTask);

    maintainer_actions = operator_actions;
    maintainer_actions.insert(ActionType::ModifyParam);
    maintainer_actions.insert(ActionType::CalibrateSystem);
    maintainer_actions.insert(ActionType::ViewAuthLog);
    maintainer_actions.insert(ActionType::ManageTask);

    admin_actions = maintainer_actions;
    admin_actions.insert(ActionType::DeleteAuthLog);
    admin_actions.insert(ActionType::CreateUser);
    admin_actions.insert(ActionType::ModifyUser);
    admin_actions.insert(ActionType::DeleteUser);
    admin_actions.insert(ActionType::ResetUserPassword);
    admin_actions.insert(ActionType::LockUnlockUser);
    admin_actions.insert(ActionType::DeletePhoto);

    QFile file(policy_path);
    if (file.exists() && file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream in(&file);
        QString current_role;
        while (!in.atEnd()) {
            QString line = in.readLine().trimmed();
            if (line.startsWith('#') || line.isEmpty()) {
                continue;
            }
            if (line.startsWith('[') && line.endsWith(']')) {
                current_role = line.mid(1, line.size() - 2).toLower();
                continue;
            }
            QStringList parts = line.split('=', Qt::SkipEmptyParts);
            if (parts.size() != 2) {
                continue;
            }
            QString key = parts[0].trimmed().toLower();
            QString value = parts[1].trimmed();
            if (key != "actions") {
                continue;
            }
            QStringList actions = value.split(',', Qt::SkipEmptyParts);
            QSet<ActionType>* target = nullptr;
            if (current_role == "operator") {
                target = &operator_actions;
                target->clear();
            } else if (current_role == "maintainer") {
                target = &maintainer_actions;
                target->clear();
            } else if (current_role == "admin") {
                target = &admin_actions;
                target->clear();
            }
            if (!target) {
                continue;
            }
            for (const QString& a : actions) {
                QString aa = a.trimmed().toLower();
                if (aa == "poweron") {
                    target->insert(ActionType::PowerOn);
                } else if (aa == "poweroff") {
                    target->insert(ActionType::PowerOff);
                } else if (aa == "starttask") {
                    target->insert(ActionType::StartTask);
                } else if (aa == "stoptask") {
                    target->insert(ActionType::StopTask);
                } else if (aa == "modifyparam") {
                    target->insert(ActionType::ModifyParam);
                } else if (aa == "calibratesystem") {
                    target->insert(ActionType::CalibrateSystem);
                } else if (aa == "viewauthlog") {
                    target->insert(ActionType::ViewAuthLog);
                } else if (aa == "deleteauthlog") {
                    target->insert(ActionType::DeleteAuthLog);
                } else if (aa == "createuser") {
                    target->insert(ActionType::CreateUser);
                } else if (aa == "modifyuser") {
                    target->insert(ActionType::ModifyUser);
                } else if (aa == "deleteuser") {
                    target->insert(ActionType::DeleteUser);
                } else if (aa == "resetuserpassword") {
                    target->insert(ActionType::ResetUserPassword);
                } else if (aa == "lockunlockuser") {
                    target->insert(ActionType::LockUnlockUser);
                } else if (aa == "managetask") {
                    target->insert(ActionType::ManageTask);
                } else if (aa == "deletephoto") {
                    target->insert(ActionType::DeletePhoto);
                }
            }
        }
        file.close();
    }

    role_permissions_.insert(UserRole::Operator, operator_actions);
    role_permissions_.insert(UserRole::Maintainer, maintainer_actions);
    role_permissions_.insert(UserRole::Admin, admin_actions);
}
