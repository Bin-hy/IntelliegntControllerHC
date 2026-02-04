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
  QWidget* createCameraTab();
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
  QLabel* label_color_stream_;
  QLabel* label_depth_stream_;
  QPushButton* btn_save_image_;
};

#endif // APP_WINDOW_HPP
