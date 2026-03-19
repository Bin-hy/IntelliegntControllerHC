#include "ui_app/idle_logout_watcher.hpp"
#include <QEvent>

IdleLogoutWatcher::IdleLogoutWatcher(int timeout_ms, QObject* parent)
    : QObject(parent) {
    timer_.setInterval(timeout_ms);
    timer_.setSingleShot(true);
    QObject::connect(&timer_, &QTimer::timeout, [this]() {
        if (logout_callback_) {
            logout_callback_();
        }
    });
}

void IdleLogoutWatcher::setLogoutCallback(std::function<void()> cb) {
    logout_callback_ = std::move(cb);
    timer_.start();
}

bool IdleLogoutWatcher::eventFilter(QObject* obj, QEvent* event) {
    Q_UNUSED(obj);
    switch (event->type()) {
    case QEvent::MouseButtonPress:
    case QEvent::MouseMove:
    case QEvent::KeyPress:
    case QEvent::Wheel:
        if (timer_.isActive()) {
            timer_.start();
        }
        break;
    default:
        break;
    }
    return false;
}
