#ifndef __STATE_MONITOR_HPP
#define __STATE_MONITOR_HPP

#include <QtWidgets>
#include <map>
#include <string>

class StateMonitor: public QMainWindow
{
    Q_OBJECT
public:
    StateMonitor(int id);
    void update_state(int s);
private:
    std::map<int, QLabel*> state_labels_;
    int state_;
    int id_;
};

#endif
