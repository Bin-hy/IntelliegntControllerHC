#ifndef TASK_DIALOGS_HPP
#define TASK_DIALOGS_HPP

#include <QDialog>
#include <memory>
#include "common_msgs/msg/task_config.hpp"
#include "common_msgs/msg/task_device_check.hpp"
#include "common_msgs/msg/task_step.hpp"
#include "ui_app/ros_node.hpp"
#include "ui_app/task_record_manager.hpp"

class QLineEdit;
class QComboBox;
class QSpinBox;
class QDoubleSpinBox;
class QTableWidget;
class QListWidget;
class QLabel;
class QProgressBar;
class RobotVizWidget;
class PointCloudWidget;
class QPushButton;

// 1. Device Add Dialog
class DeviceAddDialog : public QDialog {
    Q_OBJECT
public:
    explicit DeviceAddDialog(std::shared_ptr<RosNode> node, QWidget *parent = nullptr);
    common_msgs::msg::TaskDeviceCheck getDeviceCheck() const;

private slots:
    void onTypeChanged(const QString& type);
    void onScan();
    void onDeviceIndexChanged(int index);

private:
    std::shared_ptr<RosNode> node_;
    QComboBox* combo_type_;
    QLineEdit* edit_usage_;
    QComboBox* combo_device_select_; // Populated by scan
    QLineEdit* edit_sn_; // Or auto-filled
    QLineEdit* edit_model_;
    QPushButton* btn_scan_;
};

// 2. Step Add Dialog
class StepAddDialog : public QDialog {
    Q_OBJECT
public:
    explicit StepAddDialog(std::shared_ptr<RosNode> node, const std::vector<common_msgs::msg::TaskDeviceCheck>& devices, QWidget *parent = nullptr);
    common_msgs::msg::TaskStep getStep() const;
    void setStep(const common_msgs::msg::TaskStep& step);

private slots:
    void onTypeChanged(const QString& type);
    void onCaptureCurrent();
    void onArmCommandChanged(const QString& cmd);
    void onControlTargetChanged(const QString& target);

private:
    std::shared_ptr<RosNode> node_;
    QComboBox* combo_type_;
    QComboBox* combo_device_;
    std::vector<common_msgs::msg::TaskDeviceCheck> devices_;
    
    // Stacked Widget or just hide/show based on type
    QWidget* widget_arm_;
    QWidget* widget_hand_;
    QWidget* widget_camera_;
    QWidget* widget_io_;

    // Lift UI
    QWidget* widget_lift_;
    QComboBox* combo_lift_command_;
    QSpinBox* spin_lift_speed_rpm_;
    QSpinBox* spin_lift_target_pulses_;
    QSpinBox* spin_lift_accel_ms_;
    QSpinBox* spin_lift_decel_ms_;

    // Arm UI
    QComboBox* combo_arm_command_;       // "movej" / "movel"
    QWidget* widget_arm_movej_;          // joint angle inputs
    QWidget* widget_arm_movel_;          // cartesian inputs
    QDoubleSpinBox* spin_arm_pos_[6];    // joint angles in degrees (MoveJ)
    QDoubleSpinBox* spin_arm_cart_[6];   // cartesian [X,Y,Z,RX,RY,RZ] (MoveL)
    QDoubleSpinBox* spin_arm_velocity_;  // MoveJ: deg/s, MoveL: mm/s
    QDoubleSpinBox* spin_arm_accel_;     // accel

    // Hand UI
    QSpinBox* spin_hand_pos_[6];

    // Camera UI
    QComboBox* combo_camera_type_;

    // IO UI (属于 duco arm 设备)
    QComboBox* combo_io_group_;   // Standard 1-8 / 9-16 / Tool
    QComboBox* combo_io_port_;    // Port within group
    QComboBox* combo_io_value_;   // HIGH / LOW
    void onIOGroupChanged(int index);

    // Control UI (type=control)
    QWidget*   widget_control_;
    QComboBox* combo_control_target_;   // arm / lhand / rhand / lift
    QComboBox* combo_control_command_;  // poweron/enable/disable/poweroff/home

    // Delay UI
    QSpinBox* spin_delay_ms_;

    QLineEdit* edit_name_;
};

// 3. Task Config Dialog
class TaskConfigDialog : public QDialog {
    Q_OBJECT
public:
    explicit TaskConfigDialog(std::shared_ptr<RosNode> node, QWidget *parent = nullptr);
    void setTask(const common_msgs::msg::TaskConfig& task);
    common_msgs::msg::TaskConfig getTask() const;
    void setAllTasks(const std::vector<common_msgs::msg::TaskConfig>& tasks);

private slots:
    void onAddDevice();
    void onDeleteDevice();
    void onAddStep();
    void onEditStep();
    void onDeleteStep();
    void onImportSteps();
    void onExportSteps();
    void onImportFromFile();

private:
    static QJsonObject stepToJson(const common_msgs::msg::TaskStep& step);
    static common_msgs::msg::TaskStep stepFromJson(const QJsonObject& obj);
    std::shared_ptr<RosNode> node_;
    QLineEdit* edit_name_;
    QSpinBox* spin_rounds_;
    QTableWidget* table_devices_;
    QListWidget* list_steps_;

    std::vector<common_msgs::msg::TaskDeviceCheck> devices_;
    std::vector<common_msgs::msg::TaskStep> steps_;
    std::vector<common_msgs::msg::TaskConfig> all_tasks_;

    void updateDeviceTable();
    void updateStepList();
};

// 4. Task Run Dialog
class TaskRunDialog : public QDialog {
    Q_OBJECT
public:
    explicit TaskRunDialog(std::shared_ptr<RosNode> node,
                           const common_msgs::msg::TaskConfig& task,
                           std::shared_ptr<TaskRecordManager> record_manager = nullptr,
                           QWidget *parent = nullptr);

private slots:
    void onStart();
    void onPause();
    void onStop();
    void onEdit(); // Added declaration
    
    // Action Callbacks
    void onTaskResult(const rclcpp_action::ClientGoalHandle<common_msgs::action::ExecuteTask>::WrappedResult& result);
    void onTaskFeedback(const std::shared_ptr<const common_msgs::action::ExecuteTask::Feedback> feedback);

private:
    std::shared_ptr<RosNode> node_;
    std::shared_ptr<TaskRecordManager> record_manager_;
    common_msgs::msg::TaskConfig task_;
    TaskExecutionRecord current_record_;

    QLabel* label_status_;
    QProgressBar* progress_steps_;

    QPushButton* btn_start_;
    QPushButton* btn_pause_;
    QPushButton* btn_stop_;

    RobotVizWidget* robot_viz_;
    RobotVizWidget* lhand_viz_;

    void checkDevices();
    bool devices_ready_;
};

#endif // TASK_DIALOGS_HPP
