#include "image_debuger.hpp"
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <common/datadef.hpp>
#include <seuimage/ransac.hpp>

using namespace cv;
using namespace std;
using namespace seuimage;

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    ImageDebuger foo;
    foo.show();
    return app.exec();
}