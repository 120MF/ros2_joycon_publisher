#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <libevdev/libevdev.h>
#include <std_msgs/msg/string.hpp>

class DevicePublisherNode : public rclcpp::Node {
public:
  DevicePublisherNode() : Node("JoyCon_Publisher") {
    publisher_imu_ = this->create_publisher<std_msgs::msg::String>("joycon_imu_events", 10);
    publisher_keys_ = this->create_publisher<std_msgs::msg::String>("joycon_keys_events", 10);
    search_joycon_evdev_device();
    // std::thread thread_imu([this]() {
    //   read_joycon(dev_imu, publisher_imu_);
    // });
    // std::thread thread_keys([this]() {
    //   read_joycon(dev_keys, publisher_keys_);
    // });
    // thread_imu.join();
    // thread_keys.join();
    read_joycon(dev_imu, publisher_imu_);
  }

  ~DevicePublisherNode() override {
    libevdev_free(dev_imu);
    libevdev_free(dev_keys);
  }

private:
  void search_joycon_evdev_device() {
    std::string name;
    for (int i = 0; i <= 25; i++) {
      struct libevdev *dev = nullptr;
      dev = libevdev_new();
      char file_name[20];
      sprintf(file_name, "/dev/input/event%d", i);
      const int fd = open(file_name, O_RDONLY | O_NONBLOCK);
      if (const int err = libevdev_set_fd(dev, fd); err < 0) {
        RCLCPP_INFO(this->get_logger(), "Failed init evdev device %d: %s", i, strerror(-err));
        continue;
      }
      name = libevdev_get_name(dev);
      if (name == "Joy-Con (L)" && dev_keys == nullptr) {
        RCLCPP_INFO(this->get_logger(), "Joy-Con key input device found: \"%s\", in /dev/input/event%d\n", name.c_str(),
                    i);
        dev_keys = libevdev_new();
        dev_keys = dev;
      } else if (name == "Joy-Con (L) (IMU)" && dev_imu == nullptr) {
        RCLCPP_INFO(this->get_logger(), "Joy-Con IMU input device found: \"%s\", in /dev/input/event%d\n", name.c_str(),
                    i);
        dev_imu = libevdev_new();
        dev_imu = dev;
      } else {
        libevdev_free(dev);
      }
    }
    if (!dev_imu || !dev_keys) {
      RCLCPP_FATAL(this->get_logger(), "Can not find valid Joy-Con input device. node exiting......");
      rclcpp::shutdown();
    }
  }

  void read_joycon(struct libevdev *dev, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher) const {
    int rc = 0;
    do {
      // if (rc == LIBEVDEV_READ_STATUS_SYNC)
      // RCLCPP_INFO(this->get_logger(), "Sync drop happened.");
      struct input_event ev;
      rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
      // RCLCPP_INFO(this->get_logger(), "%d\n", rc);
      if (rc == 0) {
        // RCLCPP_INFO(this->get_logger(), "%d\n", rc);
        auto message = std_msgs::msg::String();
        message.data = "Event: " + std::string(libevdev_event_type_get_name(ev.type)) + " " +
                       std::string(libevdev_event_code_get_name(ev.type, ev.code)) + " " +
                       std::to_string(ev.value);
        // RCLCPP_INFO(this->get_logger(), "%s", message.data.c_str());
        publisher->publish(message);
      }
    } while (rc == 1 || rc == 0 || rc == -EAGAIN);
  }

  struct libevdev *dev_imu, *dev_keys = nullptr;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_imu_, publisher_keys_;
};

int main(const int argc, char **argv) {
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<DevicePublisherNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
