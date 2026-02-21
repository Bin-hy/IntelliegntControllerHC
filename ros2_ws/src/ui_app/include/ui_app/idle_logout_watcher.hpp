#ifndef IDLE_LOGOUT_WATCHER_HPP
#define IDLE_LOGOUT_WATCHER_HPP

#include <QObject>
#include <QTimer>
#include <functional>

class IdleLogoutWatcher : public QObject {
    Q_OBJECT
public:
    explicit IdleLogoutWatcher(int timeout_ms, QObject* parent = nullptr);

    void setLogoutCallback(std::function<void()> cb);

protected:
    bool eventFilter(QObject* obj, QEvent* event) override;

private:
    QTimer timer_;
    std::function<void()> logout_callback_;
};

#endif
