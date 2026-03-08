#ifndef TASK_WIDGET_HPP
#define TASK_WIDGET_HPP

#include <QWidget>
#include <memory>
#include <vector>
#include "ui_app/ros_node.hpp"
#include "ui_app/task_record_manager.hpp"
#include "common_msgs/msg/task_config.hpp"

class QListWidget;
class QPushButton;
class QTableWidget;
class QLabel;
class QProgressBar;

class TaskWidget : public QWidget {
    Q_OBJECT
public:
    explicit TaskWidget(std::shared_ptr<RosNode> node, QWidget *parent = nullptr);

private slots:
    void onAddTask();
    void onEditTask();
    void onDeleteTask();
    void onRunTask();
    void onShowHistory();
    void onExportCSV();

private:
    std::shared_ptr<RosNode> node_;
    std::shared_ptr<TaskRecordManager> record_manager_;
    QListWidget* task_list_;
    std::vector<common_msgs::msg::TaskConfig> tasks_;

    void loadTasks();
    void saveTasks();
    void updateTaskList();
};

#endif // TASK_WIDGET_HPP
