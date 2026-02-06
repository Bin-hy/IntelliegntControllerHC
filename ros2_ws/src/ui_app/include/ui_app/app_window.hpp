#ifndef APP_WINDOW_HPP
#define APP_WINDOW_HPP

#include <QWidget>
#include <memory>
#include "ui_app/ros_node.hpp"

class QLabel;
class QTextEdit;
class QTimer;
class QPushButton;
class QDoubleSpinBox;
class QSpinBox;
class QComboBox;
class QCheckBox;

class AppWindow : public QWidget {
public:
  explicit AppWindow(std::shared_ptr<RosNode> node, QWidget *parent = nullptr);

private:
  QWidget* createControlTab();
  QWidget* createMoveTab();
  QWidget* createIOTab();
  QWidget* createLHandTab();
  QWidget* createCameraTab();
  QWidget* createVideoWidget(const QString& title, QLabel*& label_ptr, std::function<void()> save_callback);
  void refreshCameraList();
  void onCameraConfigChanged();

  void updateUI();

  std::shared_ptr<RosNode> node_;
  QLabel * label_count_;
  QTextEdit * text_robot_state_;
  QTimer * timer_;
  
  QPushButton * btn_power_on_;
  QPushButton * btn_enable_;
  QPushButton * btn_disable_;
  QPushButton * btn_power_off_;

  // Move Tab UI
  QDoubleSpinBox* spin_joints_[7];
  QDoubleSpinBox* spin_cart_[6];
  QDoubleSpinBox* spin_vel_;
  QDoubleSpinBox* spin_acc_;

  // IO Tab UI
  QComboBox* combo_io_type_;
  QSpinBox* spin_io_port_;
  QCheckBox* chk_io_value_;

  // Camera Tab UI
  QComboBox* combo_camera_;
  QPushButton* btn_scan_;
  
  QCheckBox* check_color_;
  QCheckBox* check_depth_;
  QCheckBox* check_ir_left_;
  QCheckBox* check_ir_right_;

  QWidget* widget_color_;
  QWidget* widget_depth_;
  QWidget* widget_ir_left_;
  QWidget* widget_ir_right_;

  QLabel* label_color_stream_;
  QLabel* label_depth_stream_;
  QLabel* label_ir_left_stream_;
  QLabel* label_ir_right_stream_;
  QWidget* container_video_;

  // LHand UI
  QSpinBox* spin_lhand_pos_[6];
  QSpinBox* spin_lhand_vel_;
  QPushButton* btn_lhand_enable_;
  QPushButton* btn_lhand_disable_;
  QPushButton* btn_lhand_home_;
  QPushButton* btn_lhand_move_;
};

#endif // APP_WINDOW_HPP
