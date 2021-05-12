#include <QApplication>
#include <fstream>
#include "action_debuger.hpp"
#include <seumath/math.hpp>

using namespace std;
using namespace robot;
using namespace seumath;
using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_debuger");
    ros::NodeHandle node;
    ros::service::waitForService("/debug/action/run");
    QApplication app(argc, argv);
    glutInit(&argc, argv);
    ActionDebuger foo(node);
    foo.show();
    return app.exec();
}