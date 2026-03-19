#include "ui_app/app_window.hpp"
#include <QVBoxLayout>

QWidget* AppWindow::createTaskTab() {
    auto * widget = new QWidget();
    auto * layout = new QVBoxLayout();
    
    auto * task_widget = new TaskWidget(node_, current_user_, current_role_);
    layout->addWidget(task_widget);
    
    widget->setLayout(layout);
    return widget;
}
