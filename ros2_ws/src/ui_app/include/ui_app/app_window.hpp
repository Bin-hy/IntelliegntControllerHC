#ifndef APP_WINDOW_HPP
#define APP_WINDOW_HPP

#include <QWidget>
#include <memory>
#include "ui_app/ros_node.hpp"
#include "ui_app/robot_viz_widget.hpp"
#include "ui_app/task_widget.hpp"
#include "ui_app/auth_manager.hpp"
#include "ui_app/auth_log_manager.hpp"
#include "ui_app/permission_manager.hpp"

class QLabel;
class QTextEdit;
class QTimer;
class QPushButton;
class QDoubleSpinBox;
class QSpinBox;
class QComboBox;
class QCheckBox;
class QTabWidget;
class QTableWidget;
class QLineEdit;
class QDateTimeEdit;
class QToolButton;
class QMenu;
class PointCloudWidget;
class UDEGloveSDK;

class AppWindow : public QWidget {
public:
  AppWindow(std::shared_ptr<RosNode> node,
            std::shared_ptr<AuthManager> auth_manager,
            std::shared_ptr<AuthLogManager> log_manager,
            PermissionManager* permission_manager,
            UserRole current_role,
            QWidget *parent = nullptr);
  ~AppWindow();

  void setCurrentRole(UserRole role);
  void setUserSession(const QString& username, UserRole role, const QString& session_id, bool logged_in);

private:
  QWidget* createControlTab();
  void buildDashboard();
  QWidget* createMoveTab();
  QWidget* createIOTab();
  QWidget* createLHandTab();
  QWidget* createGloveTab();
  void updateGloveData();
  QWidget* createCameraTab();
  QWidget* createTaskTab();
  QWidget* createAdminTab();
  QWidget* createVideoWidget(const QString& title, QLabel*& label_ptr, std::function<void()> save_callback);
  void refreshCameraList();
  void onCameraConfigChanged();

  void updateUI();
  void rebuildAdminTab();
  void applyPermissionToControls();

  void showDeviceInfoDialog();
  void showTaskDialog();
  void onLogin();
  void onLogout();
  void onPermission();
  void updateUserModule();
  std::shared_ptr<RosNode> node_;
  std::shared_ptr<AuthManager> auth_manager_;
  std::shared_ptr<AuthLogManager> auth_log_manager_;
  PermissionManager* permission_manager_;
  UserRole current_role_;

  QLabel * label_count_;
  QTextEdit * text_robot_state_;
  QLabel * label_user_name_;
  QLabel * label_user_role_;
  QToolButton * btn_avatar_;
  QMenu * menu_user_;
  bool logged_in_;
  QString current_user_;
  QString session_id_;
  QTimer * timer_;
  QTabWidget* tabs_;
  QWidget* admin_tab_;
  bool left_hand_loaded_;
  bool right_hand_loaded_;
  
  QPushButton * btn_power_on_;
  QPushButton * btn_enable_;
  QPushButton * btn_disable_;
  QPushButton * btn_power_off_;

  QDoubleSpinBox* spin_joints_[7];
  QDoubleSpinBox* spin_cart_[6];
  QDoubleSpinBox* spin_vel_;
  QDoubleSpinBox* spin_acc_;

  QComboBox* combo_io_type_;
  QSpinBox* spin_io_port_;
  QCheckBox* chk_io_value_;

  QComboBox* combo_camera_;
  QComboBox* combo_pc_topic_;
  QPushButton* btn_scan_;
  
  QCheckBox* check_color_;
  QCheckBox* check_depth_;
  QCheckBox* check_ir_left_;
  QCheckBox* check_ir_right_;
  QCheckBox* check_point_cloud_;

  QWidget* widget_color_;
  QWidget* widget_depth_;
  QWidget* widget_ir_left_;
  QWidget* widget_ir_right_;

  QLabel* label_color_stream_;
  QLabel* label_depth_stream_;
  QLabel* label_ir_left_stream_;
  QLabel* label_ir_right_stream_;
  PointCloudWidget* widget_point_cloud_;
  QWidget* container_video_;

  QTableWidget* admin_user_table_;
  QTableWidget* admin_log_table_;
  QLineEdit* admin_log_user_filter_;
  QDateTimeEdit* admin_log_from_;
  QDateTimeEdit* admin_log_to_;
  QCheckBox* admin_log_success_only_;
  QCheckBox* admin_log_failure_only_;

  QSpinBox* spin_lhand_pos_[6];
  QSpinBox* spin_lhand_vel_;
  QPushButton* btn_lhand_enable_;
  QPushButton* btn_lhand_disable_;
  QPushButton* btn_lhand_home_;
  QPushButton* btn_lhand_move_;
  QPushButton* lhand_joint_buttons_[6];
  QPushButton* btn_lhand_set_vel_;

  RobotVizWidget* robot_viz_;

  // Motion capture glove
  std::unique_ptr<UDEGloveSDK> glove_sdk_;
  QTimer* glove_timer_ = nullptr;
  QSpinBox* spin_glove_port_ = nullptr;
  QPushButton* btn_glove_start_ = nullptr;
  QPushButton* btn_glove_stop_ = nullptr;
  QLabel* label_glove_status_ = nullptr;
  QLabel* label_glove_role_ = nullptr;
  QTableWidget* table_glove_left_ = nullptr;
  QTableWidget* table_glove_right_ = nullptr;
  QLabel* label_glove_ctrl_[12] = {};
};

#endif // APP_WINDOW_HPP
