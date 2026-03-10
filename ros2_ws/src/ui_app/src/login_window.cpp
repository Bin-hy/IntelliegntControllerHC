#include "ui_app/login_window.hpp"
#include "ui_app/i18n_manager.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QToolButton>
#include <QPushButton>
#include <QRandomGenerator>
#include <QDateTime>

LoginWindow::LoginWindow(std::shared_ptr<AuthManager> auth_manager,
                         std::shared_ptr<AuthLogManager> log_manager,
                         QWidget* parent)
    : QDialog(parent),
      auth_manager_(std::move(auth_manager)),
      log_manager_(std::move(log_manager)),
      login_succeeded_(false) {
    setWindowTitle(tr_ui("设备安全登录系统", "Secure Login"));
    resize(400, 260);

    auto* layout = new QVBoxLayout(this);

    auto* row_user = new QHBoxLayout();
    row_user->addWidget(new QLabel(tr_ui("用户名:", "Username:")));
    edit_username_ = new QLineEdit();
    row_user->addWidget(edit_username_);
    layout->addLayout(row_user);

    auto* row_pass = new QHBoxLayout();
    row_pass->addWidget(new QLabel(tr_ui("密  码:", "Password:")));
    edit_password_ = new QLineEdit();
    edit_password_->setEchoMode(QLineEdit::Password);
    row_pass->addWidget(edit_password_);
    btn_toggle_password_ = new QToolButton();
    btn_toggle_password_->setText("👁");
    row_pass->addWidget(btn_toggle_password_);
    layout->addLayout(row_pass);

    btn_login_ = new QPushButton(tr_ui("登录", "Login"));
    layout->addWidget(btn_login_);

    label_status_ = new QLabel();
    layout->addWidget(label_status_);

    label_fail_count_ = new QLabel();
    layout->addWidget(label_fail_count_);

    label_lock_info_ = new QLabel();
    layout->addWidget(label_lock_info_);

    connect(btn_login_, &QPushButton::clicked, this, &LoginWindow::onLoginClicked);
    connect(btn_toggle_password_, &QToolButton::clicked, this, &LoginWindow::onTogglePassword);
}

bool LoginWindow::loginSucceeded() const {
    return login_succeeded_;
}

UserSession LoginWindow::session() const {
    return session_;
}

void LoginWindow::onTogglePassword() {
    if (edit_password_->echoMode() == QLineEdit::Password) {
        edit_password_->setEchoMode(QLineEdit::Normal);
    } else {
        edit_password_->setEchoMode(QLineEdit::Password);
    }
}

void LoginWindow::onLoginClicked() {
    QString username = edit_username_->text().trimmed();
    QString password = edit_password_->text();
    if (username.isEmpty() || password.isEmpty()) {
        label_status_->setText(tr_ui("请输入用户名和密码", "Please enter username and password"));
        return;
    }
    UserInfo info;
    QString reason;
    if (!auth_manager_->verifyPassword(username, password, info, reason)) {
        label_status_->setText(tr_ui("登录失败: ", "Login failed: ") + reason);
        label_fail_count_->setText(tr_ui("失败次数或已锁定", "Too many failures or locked"));
        if (log_manager_) {
            log_manager_->logFailure(username, reason);
        }
        qint64 now = QDateTime::currentSecsSinceEpoch();
        if (info.lock_until > now) {
            qint64 remain = info.lock_until - now;
            int minutes = static_cast<int>((remain + 59) / 60);
            label_lock_info_->setText(tr_ui("账号锁定，剩余约 %1 分钟", "Account locked, ~%1 min remaining").arg(minutes));
        }
        return;
    }
    login_succeeded_ = true;
    session_.username = info.username;
    session_.role = info.role;
    session_.session_id = QString::number(QDateTime::currentMSecsSinceEpoch()) +
                          "_" +
                          QString::number(QRandomGenerator::global()->generate64(), 16);
    label_status_->setText(tr_ui("登录成功", "Login successful"));
    label_fail_count_->clear();
    label_lock_info_->clear();
    if (log_manager_) {
        log_manager_->logSuccess(username);
    }
    accept();
}
