# How to add code in package with cpp
[Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)에서 주로 xml, Cmake를 작성하는 방법을 다룰 것입니다.

## 세팅
모든 ROS 2 작업을 하기 앞서 다음 bash를 실행합니다.
```bash
source /opt/ros/foxy/setup.bash
```


패키지를 만듭니다. 앞서 사용했던 workspace를 사용합니다.   
위치 : `~/ros2_ws/src`

```bash
ros2 pkg create --build-type ament_cmake cpp_pubsub
```

다음 위치로 옮겨 퍼블리셔가 될 빈 cpp 파일을 받습니다.  
위치 : `~/ros2_ws/src/cpp_pubsub/src`
```bash
wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_publisher/member_function.cpp
```

그러면 해당 위치에 `publisher_member_function.cpp`파일이 생성되었을 것입니다. 여기에 들어가면 텅 비어있는데, 다음과 같은 코드를 붙여넣습니다.

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

또 subscriber가 될 빈 cpp 파일을 받습니다.  
위치 : `~/ros2_ws/src/cpp_pubsub/src`

```bash
wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_subscriber/member_function.cpp
```

그곳에 다음 코드를 붙여넣습니다.

```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

## package.xml 설정
`~/ros2_ws/src/cpp_pubsub`이 위치에 `package.xml`
여기에선 디펜던시를 설정해주어야 합니다.

처음에 들어가면 다음과 같은 상태입니다.

```xml
<?xml version="1.0"?>
<?xml-model
   href="http://download.ros.org/schema/package_format3.xsd"
   schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
 <name>my_package</name>
 <version>0.0.0</version>
 <description>TODO: Package description</description>
 <maintainer email="user@todo.todo">user</maintainer>
 <license>TODO: License declaration</license>

 <buildtool_depend>ament_cmake</buildtool_depend>

 <test_depend>ament_lint_auto</test_depend>
 <test_depend>ament_lint_common</test_depend>

 <export>
   <build_type>ament_cmake</build_type>
 </export>
</package>
```

여기에 `<description>`, `<maintainer>`, `<license>`와 같은 tag들을 다음과 같이 채워줍니다.

```xml
<description>Examples of minimal publisher/subscriber using rclcpp</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

`ament_cmake` buildtool dependency를 설정하는 줄 뒤에 다음 줄을 추가해줍니다.

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

이를 통해 이 패키지는 안의 코드가 build되거나 execute될 때 `rclcpp`와 `std_msgs`를 필요로함을 선언할 수 있습니다. 

다 하면 다음과 같이 될 것입니다.

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>cpp_pubsub</name>
  <version>0.0.0</version>
  <description>Examples fo minimal publisher/subscriber using rclcpp</description>
  <maintainer email="jinu1423@naver.com">jinoo</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <!--여기에 디펜던시로서 필요한 요소들을 추가한 모습-->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```


## CMakeLists.txt 설정

이는 처음에 다음과 같을 것입니다.

```cmake
cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

우선 디펜던시를 설정합니다.  
`find_package(ament_cmake REQUIRED)` 밑에 다음과 같이 디펜던시를 추가해줍니다. 

```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```

`publisher_member_function.cpp`로부터 `talker`라는 이름의 executable을 추가합니다. 또한, `subscriber_member_function.cpp`로부터 `listener`라는 이름의 executable을 추가합니다. 이는 `ament_package()`위에 추가하면 됩니다.

```cmake
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
```

위에서 추가한 executable들이 어디에 install될 지 지정하여 `ros2 run`을 했을 때 executable들을 찾아 실행할 수 있도록 합니다. 이것도 `ament_package()`위에 추가하면 됩니다.

```cmake
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

그러면 다음과 같이 완성됩니다.

```cmake
cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# 여기에 필요한 디펜던시를 추가해준다.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# 또한 executable을 추가하고 이름을 talker로 설정한다. 
# 그러면 ros2 run으로 노드를 사용할 수 있을 것이다.
add_executable(talker src/publisher_member_function.cpp)
# 여기에서 talker는 rclcpp와 std_msg를 사용하므로 
# ament_target_dependencies를 이용하여 디펜던시를 명시한다.
ament_target_dependencies(talker rclcpp std_msgs)
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)


# 마지막으로, install(TARGETS...)을 사용하여 ros2 run을 통해 
# executable을 찾을 수 있도록 합니다.
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()

```

## build & run

디펜던시 체크를 해줍니다.  
위치 : `~/ros2_ws`

```bash
rosdep install -i --from-path src --rosdistro foxy -y
```

새로 만든 패키지를 build해줍니다.
```bash
colcon build --packages-select cpp_pubsub
```

새로운 터미널을 띄우고 적절한 환경을 만들어줍니다.

```bash
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws
. install/setup.bash
```

이제 노드를 실행하여 잘 동작하는지 확인합니다.

터미널 1 :
```bash
ros2 run cpp_pubsub talker
```

터미널 2 :
```bash
ros2 run cpp_pubsub listener
```