#ifndef APP_WINDOW_HPP
#define APP_WINDOW_HPP

#include <QWidget>
#include <memory>
#include "ui_app/ros_node.hpp"
#include "ui_app/robot_viz_widget.hpp"
#include "ui_app/selectable_image_label.hpp"
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
class QGridLayout;
class QLineEdit;
class QDateTimeEdit;
class QToolButton;
class QMenu;
class PointCloudWidget;
class UDEGloveSDK;
struct Vector3Float;

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
  QWidget* createCalibrationTab();
  void updateGloveData();
  QWidget* createCameraTab();
  QWidget* createTaskTab();
  QWidget* createAdminTab();
  QWidget* createVideoWidget(const QString& title, QLabel*& label_ptr, std::function<void()> save_callback);
  void refreshCameraList();
  void onCameraConfigChanged();
  void onCollisionCameraChanged();

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

  // IO Control Panel
  static constexpr int IO_MAX_PORTS = 16;
  QComboBox* combo_io_group_;
  QGridLayout* io_grid_;
  QWidget* io_rows_container_;
  QLabel* do_status_labels_[IO_MAX_PORTS];
  QLabel* di_status_labels_[IO_MAX_PORTS];
  QPushButton* do_on_buttons_[IO_MAX_PORTS];
  QPushButton* do_off_buttons_[IO_MAX_PORTS];
  QLabel* io_name_labels_[IO_MAX_PORTS];
  int io_visible_count_ = 0;

  // Lift Platform Control - Position Mode Only
  QDoubleSpinBox* spin_lift_pos_target_ = nullptr;  // cm
  QSpinBox* spin_lift_pos_speed_ = nullptr;
  QPushButton* btn_lift_enable_ = nullptr;
  QPushButton* btn_lift_pos_stop_ = nullptr;
  QLabel* label_lift_pos_status_ = nullptr;

  // Lift position polling
  QTimer* timer_lift_pos_poll_ = nullptr;
  int lift_poll_last_pos_ = INT_MIN;
  int lift_poll_stable_count_ = 0;
  int lift_poll_elapsed_ = 0;
  int lift_poll_target_pulses_ = 0;
  int lift_op_seq_ = 0;  // 操作序号，Stop时递增使挂起操作失效
  void startLiftPositionPoll(int target_pulses);
  void stopLiftPositionPoll(const QString& final_status);
  void liftAutoStop();

  void rebuildIORows(int group_index);
  void refreshIOGroup();
  void setDoOutput(int io_type, int port, bool value, int idx);
  void readDoStatus(int io_type, int port, int idx);
  void readDiStatus(int io_type, int port, int idx);

  QComboBox* combo_camera_;
  QComboBox* combo_pc_topic_;
  QComboBox* combo_collision_camera_ = nullptr;
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
  QLabel* label_depth_measure_ = nullptr;

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

  // Glove → Hand control
  QCheckBox* chk_glove_hand_ctrl_ = nullptr;
  QComboBox* combo_glove_hand_side_ = nullptr;
  QLabel* label_glove_hand_status_ = nullptr;
  bool glove_hand_request_in_flight_ = false;
  std::array<int, 6> glove_hand_last_sent_ = {-1, -1, -1, -1, -1, -1};

  // Vision calibration tab (debug)
  QComboBox* combo_calib_camera_ = nullptr;
  QDoubleSpinBox* spin_calib_tf_[6] = {};
  QDoubleSpinBox* spin_grasp_offset_[3] = {};
  QSpinBox* spin_calib_hand_[6] = {};
  QLabel* label_calib_status_ = nullptr;
  QLabel* label_grasp_target_ = nullptr;
  QPushButton* btn_grasp_ = nullptr;
  QString calib_yaml_path_;   // path to hand_eye.yaml currently loaded

  void loadCalibYaml(const QString& path);   // populate spinboxes from yaml

  // Video target selection (event filter on label_color_stream_)
  QPointF video_selected_point_;
  bool has_video_selection_ = false;
  QSize video_image_size_;
  bool eventFilter(QObject* obj, QEvent* event) override;

  // Mapping helpers
  int gloveAngleToPosition(double deg, double min_deg, double max_deg) const;
  void sendGloveToHand(const std::vector<Vector3Float>& finger_data);
};

#endif // APP_WINDOW_HPP
