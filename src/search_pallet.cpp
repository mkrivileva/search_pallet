#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>

const double PALLET_WIDTH = 0.39;
const double PALLET_LENGTH = 0.41;
const double PALLET_DIAG = pow((pow(PALLET_WIDTH, 2) + pow(PALLET_LENGTH,2) ), 0.5);
const double MAX_RANGE = 2.0;
const double TOLERANCE = 0.02;
cv::Mat img;
int image_height = 0;
int image_width = 0;
std::string image_encoding;
vector <double> ranges;
double angle_max;
double angle_increment;

//     image_transport::Subscriber img_sub;
//     image_transport::Publisher img_pub;
//     ros::Subscriber scan_sub;

//     
//     self.image = None;
//     ros::Rate loop_rate(20);
// public:
//         SearchImage(ros::NodeHandle % nh, image_transport::ImageTransport &it)
//         {

//         }
// };

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    image_height = msg->height;
    image_width = msg->width;
    image_encoding = msg->encoding.c_str();
    img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("view", img);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ranges = msg->ranges;
  angle_max = msg->angle_max;
  angle_increment = msg->angle_increment;
}

std::pair <int, int> xy_of_dot(int index, double dist_value)
{
  double angle = angle_of_dot(index);
  return (dist_value * sin(angle), dist_value * cos(angle));
}

def angle_of_dot(self, index):
    return self.angle_increment * (index - len(self.ranges) // 2)

def distance_between_dots(self, first, second):
    x1, y1 = self.xy_of_dot(first, self.ranges[first])
    x2, y2 = self.xy_of_dot(second, self.ranges[second])
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def search_pallet(self):
  for i in range(0, len(self.ranges), 5):
      dist = self.ranges[i]
      if dist > self.MAX_RANGE:
          continue
      #angle_increment = msg.angle_increment
      if dist < self.PALLET_DIAG:
          beta = self.angle_max
      else:
          beta = math.asin(self.PALLET_DIAG / dist)
      min_i = int(max(0, i - beta // self.angle_increment))
      max_i = int(min(len(self.ranges), i + beta // self.angle_increment))
      width_candidates = []
      length_candidates = []
      diag_candidates = []
      for j in range (min_i, max_i):
          s = self.distance_between_dots(i, j)
          if abs(s - self.PALLET_WIDTH) <= self.tolerance:
              width_candidates.append(j)
          if abs(s - self.PALLET_LENGTH) <= self.tolerance:
              length_candidates.append(j)
          if abs(s - self.PALLET_DIAG) <= self.tolerance:
              diag_candidates.append(j)
      candidates = []
      errors = []
      for w in width_candidates:
          for l in length_candidates:
              for d in diag_candidates:
                  s1 = abs(self.distance_between_dots(w, d) - self.PALLET_LENGTH)
                  s2 = abs(self.distance_between_dots(l, d)  - self.PALLET_WIDTH)
                  s3 = abs(self.distance_between_dots(l, w) - self.PALLET_DIAG)
                  if (s1 <= self.tolerance and s2 <= self.tolerance and s3 <= self.tolerance):
                      # return (i, w, l, d)
                      candidates.append((i, w, l, d))
                      errors.append((s1 ** 2 + s2 ** 2 + s3 ** 2) ** 0.5)
  if len(errors) == 0:
      return (-1, -1, -1, -1)
  min_value = min(errors)
  min_index=errors.index(min_value)
  return candidates[min_index]

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  ros::rate loop_rate(5);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);
  ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);
  while (ros::ok())
  {
    if (ranges.size())
    {

    }
  }
  cv::destroyWindow("view");
}