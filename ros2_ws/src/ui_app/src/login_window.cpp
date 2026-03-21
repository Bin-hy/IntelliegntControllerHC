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

    QLabel* label_tips = new QLabel(tr_ui("提示: 初次登录默认密码为 admin@123", "Tip: Default password is admin@123"));
    label_tips->setStyleSheet("color: gray; font-size: 11px;");
    layout->addWidget(label_tips);

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
    UserInfo info{};
    QString reason;
    if (!auth_manager_->verifyPassword(username, password, info, reason)) {
        label_status_->setText(tr_ui("登录失败: ", "Login failed: ") + reason);
        if (log_manager_) {
            log_manager_->logFailure(username, reason);
        }
        qint64 now = QDateTime::currentSecsSinceEpoch();
        if (info.lock_until > now) {
            // Account just got locked or was already locked
            qint64 remain = info.lock_until - now;
            int minutes = static_cast<int>((remain + 59) / 60);
            label_fail_count_->setText(tr_ui("账号已锁定", "Account locked"));
            label_lock_info_->setText(tr_ui("剩余约 %1 分钟后解锁", "Unlocks in ~%1 min").arg(minutes));
        } else {
            // Still within allowed attempts — show remaining chances (threshold = 3)
            constexpr int kMaxAttempts = 3;
            int remaining = kMaxAttempts - info.failed_attempts;
            label_fail_count_->setText(tr_ui("已失败 %1 次，还可尝试 %2 次",
                                             "Failed %1 time(s), %2 attempt(s) left")
                                           .arg(info.failed_attempts)
                                           .arg(remaining > 0 ? remaining : 0));
            label_lock_info_->clear();
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
