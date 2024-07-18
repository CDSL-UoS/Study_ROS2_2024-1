# 09 Node

4가지 예시를 실행해볼 것이다.

## Node의 생성 후 Log를 출력하는 예제

```bash
ros2 run cpp_first_pkg simple_node
```

```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {

  // rclcpp initalizer
  rclcpp::init(argc, argv);

  // rclcpp의 Node의 make_shared로 Node 생성 : make_shared 안에 node의 이름이 들어간다.
  auto node = rclcpp::Node::make_shared("simple_node");

  // Console에 log를 출력한다. 노드를 생성하지 않는다면 rclcpp::get_logger로 해야 한다.
  RCLCPP_INFO(node->get_logger(), "Logger Test");

  // ROS2 활경을 종료한다.
  rclcpp::shutdown();
  return 0;
}
```

ROS1과 달리 ROS2는 포인터를 이용한다.

## 주기적으로 node를 동작시키는 예제

```bash
ros2 run cpp_first_pkg simple_loop_node
```

```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {

  // 초기화
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_loop_node");

  // WallRate는 주기에 해당한다. 일정한 주기를 갖는 rate instance를 만든다.
  rclcpp::WallRate rate(2);  // Hz

  // ok() 상태를 넣으면, 우리가 강제적으로 종료하지 않는 한 while문은 계속된다.
  while (rclcpp::ok()) {
    RCLCPP_INFO(node->get_logger(), "Simple Loop Node");

    // spin_some 안에 node를 넣어주면 한 번 실행시킨다.
    rclcpp::spin_some(node);

    // 호출한다. 주기만큼 반복한다.
    rate.sleep();
  }

  // 종료
  rclcpp::shutdown();
  return 0;
}
```

## 상속을 통한 Node 생성

```bash
ros2 run cpp_first_pkg simple_oop_node
```

```cpp
#include <memory>
#include "rclcpp/rclcpp.hpp"

// 핵심!! rclcpp::Node를 상속받습니다.
class Talker : public rclcpp::Node
{
private:
	// Node를 주기적으로 실행시켜 줄 timer 입니다.
  rclcpp::TimerBase::SharedPtr m_timer;

  // Member 변수로 m_count를 넣는다.
  size_t m_count;
	
  // 실질적으로 실행될 함수입니다.
  void timer_callback()
  {
    // 카운트를 올린다.
    m_count++;
		// Log 출력한다.
    RCLCPP_INFO(this->get_logger(), "I am Simple OOP Example, count : %d", m_count);
  }

public:
	// 생성자 함수의 인자로 이름을 넣어주어 노드를 생성한다.
  Talker() : Node("simple_oop_node")
  {
		// create_wall_timer 함수에 주기와 실행 대상을 인자로 넣는다. -> 주기적으로 실행한다.
		// this->는 굳이 명시하지 않아도 됩니다.
    m_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Talker::timer_callback, this));
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

	// spin은 Node내부에 정해진 timer에 따라 Node를 주기적으로 동작, 갱신시킨다.
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
```

## Lifecycle

```bash
anthony@anthony-B760M-AORUS-ELITE:~/gcamp_ros2_ws$ ros2 run cpp_first_pkg lifecycle_node

// Node Constructor가 가장 먼저 실행된다.
[WARN] [1712792135.195308434] [simple_oop_node]: Node Constructor

// Timer Callback이 실행된다.
[INFO] [1712792135.695519824] [simple_oop_node]: I am Simple OOP Example, count : 1
[INFO] [1712792136.195520190] [simple_oop_node]: I am Simple OOP Example, count : 2
[INFO] [1712792136.695551740] [simple_oop_node]: I am Simple OOP Example, count : 3
[INFO] [1712792137.195536855] [simple_oop_node]: I am Simple OOP Example, count : 4
[INFO] [1712792137.695551270] [simple_oop_node]: I am Simple OOP Example, count : 5
[INFO] [1712792138.195553282] [simple_oop_node]: I am Simple OOP Example, count : 6
[INFO] [1712792138.695549050] [simple_oop_node]: I am Simple OOP Example, count : 7
[INFO] [1712792139.195567812] [simple_oop_node]: I am Simple OOP Example, count : 8
[INFO] [1712792139.695579495] [simple_oop_node]: I am Simple OOP Example, count : 9
[INFO] [1712792140.195583433] [simple_oop_node]: I am Simple OOP Example, count : 10

// 강제 종료를 한 시점
^C[INFO] [1712792140.641371251] [rclcpp]: signal_handler(signal_value=2)

// spin을 벗어나서 Spin Done
[INFO] [1712792140.641732090] [simple_oop_node]: ==== Spin Done ====

// rclcpp::shutdown(); 이후 
==== After Shutdown ====

// rclcpp가 stutdown된 이후 제일 마지막에 Node Destructor가 실행된다.
[WARN] [1712792140.642664455] [simple_oop_node]: Node Destructor
```

```cpp
#include <memory>
#include "rclcpp/rclcpp.hpp"

class Talker : public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr m_timer;
  size_t m_count;

  void timer_callback() {
    m_count++;

    // console 1
    RCLCPP_INFO(this->get_logger(), "I am Simple OOP Example, count : %d",
                m_count);
  }

public:
  Talker() : Node("simple_oop_node"), m_count(0) {

    // console 1
    RCLCPP_WARN(this->get_logger(), "Node Constructor");

    m_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                      std::bind(&Talker::timer_callback, this));
  }

  ~Talker() {
    // publisher
    // console 3
    RCLCPP_WARN(this->get_logger(), "Node Destructor");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto talker = std::make_shared<Talker>();
  rclcpp::spin(talker);

  // console 4
  RCLCPP_INFO(talker->get_logger(), "==== Spin Done ====");

  rclcpp::shutdown();

  // console 5
  std::cout << "==== After Shutdown ====" << std::endl;

  return 0;
}
```
