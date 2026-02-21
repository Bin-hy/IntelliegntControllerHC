#include <QApplication>
#include <QFile>
#include <QTimer>
#include <QMessageBox>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <cstdlib>
#include "ui_app/ros_node.hpp"
#include "ui_app/app_window.hpp"
#include "ui_app/auth_manager.hpp"
#include "ui_app/auth_log_manager.hpp"
#include "ui_app/permission_manager.hpp"
#include "ui_app/login_window.hpp"
#include "ui_app/idle_logout_watcher.hpp"

int main(int argc, char ** argv) {
  if (geteuid() == 0) {
    QApplication tmp(argc, argv);
    QMessageBox::critical(nullptr,
                          "安全错误",
                          "禁止以 root 权限运行本程序，请使用普通用户。");
    return 1;
  }

  rclcpp::init(argc, argv);
  std::shared_ptr<RosNode> node(new RosNode());
  
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  QApplication app(argc, argv);

  QFile styleFile(":/style.qss");
  if(styleFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
      app.setStyleSheet(styleFile.readAll());
      styleFile.close();
  }

  const char* home_env = std::getenv("HOME");
  QString base_dir = home_env ? QString::fromLocal8Bit(home_env) : ".";
  QString user_file = base_dir + "/.secure/users.dat";
  QString log_dir = base_dir + "/.logs";
  QString policy_file = base_dir + "/.secure/policy.conf";

  std::shared_ptr<AuthManager> auth_manager(new AuthManager(user_file));
  std::shared_ptr<AuthLogManager> auth_log_manager(new AuthLogManager(log_dir));
  PermissionManager permission_manager(policy_file);

  LoginWindow login(auth_manager, auth_log_manager);
  if (login.exec() != QDialog::Accepted || !login.loginSucceeded()) {
      rclcpp::shutdown();
      return 0;
  }
  UserSession session = login.session();

  node->set_user_context(session.username.toStdString(),
                         permission_manager.roleToString(session.role).toStdString(),
                         session.session_id.toStdString());

  AppWindow w(node, auth_manager, auth_log_manager, &permission_manager, session.role);
  w.setWindowTitle("机器臂手上位机");
  w.resize(800, 600);
  w.show();

  IdleLogoutWatcher watcher(10 * 60 * 1000, &app);
  watcher.setLogoutCallback([&]() {
      w.hide();
      LoginWindow dlg(auth_manager, auth_log_manager);
      if (dlg.exec() == QDialog::Accepted && dlg.loginSucceeded()) {
          session = dlg.session();
          node->set_user_context(session.username.toStdString(),
                                 permission_manager.roleToString(session.role).toStdString(),
                                 session.session_id.toStdString());
          w.setCurrentRole(session.role);
          w.show();
      } else {
          app.quit();
      }
  });
  app.installEventFilter(&watcher);
  
  QTimer timer;
  QObject::connect(&timer, &QTimer::timeout, [&exec](){
    exec.spin_some();
  });
  timer.start(10);

  int ret = app.exec();
  
  rclcpp::shutdown();
  
  return ret;
}
