#include "depth2laser.h"

Depth2Laser d2l;

void frameCallback(const sensor_msgs::Image::ConstPtr& frame){


...



d2l.setImage(the image);
d2l.compute();
std::vector<float> ranges = d2l.ranges();

...

scan.ranges = ranges;

}
