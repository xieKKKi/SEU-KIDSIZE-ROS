#include "team_monitor.hpp"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    TeamMonitor foo;
    foo.show();
    return app.exec();
}