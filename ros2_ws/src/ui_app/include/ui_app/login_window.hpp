#ifndef LOGIN_WINDOW_HPP
#define LOGIN_WINDOW_HPP

#include <QDialog>
#include <memory>
#include "ui_app/auth_manager.hpp"
#include "ui_app/auth_log_manager.hpp"

class QLineEdit;
class QLabel;
class QToolButton;
class QPushButton;

struct UserSession {
    QString username;
    UserRole role;
    QString session_id;
};

class LoginWindow : public QDialog {
    Q_OBJECT
public:
    LoginWindow(std::shared_ptr<AuthManager> auth_manager,
                std::shared_ptr<AuthLogManager> log_manager,
                QWidget* parent = nullptr);

    bool loginSucceeded() const;
    UserSession session() const;

private slots:
    void onLoginClicked();
    void onTogglePassword();

private:
    std::shared_ptr<AuthManager> auth_manager_;
    std::shared_ptr<AuthLogManager> log_manager_;

    QLineEdit* edit_username_;
    QLineEdit* edit_password_;
    QToolButton* btn_toggle_password_;
    QPushButton* btn_login_;
    QLabel* label_status_;
    QLabel* label_fail_count_;
    QLabel* label_lock_info_;

    bool login_succeeded_;
    UserSession session_;
};

#endif
