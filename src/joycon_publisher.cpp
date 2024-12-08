#include <rclcpp/rclcpp.hpp>
#include <libevdev-1.0/libevdev/libevdev.h>
#include <fcntl.h>


class TimerNode : public rclcpp::Node
{
public:
  TimerNode() : Node("timer_node")
  {
    dev = libevdev_new();
    fd = open("/dev/input/event10", O_RDONLY|O_NONBLOCK);
    int err = libevdev_set_fd(dev, fd);
    if (err < 0) {
      RCLCPP_INFO(this->get_logger(), "Failed to init libevdev: %s", strerror(-err));
    }

    // RCLCPP_INFO(this->get_logger(), "Input device name: \"%s\"\n", libevdev_get_name(dev));
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TimerNode::timer_callback, this));
  }

private:
  int fd;
  struct libevdev *dev;
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Timer callback triggered");
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimerNode>());
  rclcpp::shutdown();
  return 0;
}