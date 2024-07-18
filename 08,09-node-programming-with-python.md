# Node programming with python
코드 분석을 위주로 진행합니다.

다음 코드들은 시뮬레이션을 띄운 상태

```bash
ros2 launch gcamp_gazebo gcamp_world.launch.py 
```

에서 해당 노드를 띄우면 결과를 볼 수 있습니다. 

이 코드의 주석으로 코드 분석을 작성하였습니다. import -> main -> class 순서로 보는 것을 추천합니다. 
## Publisher Node
시뮬레이션 안의 로봇이 자리를 계속 돌도록 하는 메시지를 전달하는 publisher를 가진 node의 코드입니다. (`ctrl + c`로 프로세스를 멈췄을 때 로봇도 멈출 수 있도록 수정했습니다.)

```python
# !/usr/bin/env/ python3

import sys

# geometry_msgs/msg/Twist 메시지를 사용하기 위해 import
# 대문자 소문자 등의 오타를 조심해야한다.
from geometry_msgs.msg import Twist

# node를 만들 때 언제나 import해야하는 Node class! 
import rclpy
from rclpy.node import Node 

# 새로운 node를 생성하는 class. Node라는 class를 상속한다.
class CmdVelPublisher(Node):

    def __init__(self):
        # 상속하는 부모 class 초기화.
        # 여기에선 이 node의 이름을 정해준다.
        super().__init__('endless_cmd_vel_pub_node') 

        # publisher을 설정한다.
        # 이 publisher은 Twist Message를 사용하고, 'skidbot/cmd_vel'이라는 토픽을 사용하며,
        # Queue의 크기를 10으로 가진다.
        # 이때 Queue는 subscriber가 충분히 빠른 속도로 정보를 받지 못했을 때
        # 메시지를 최대 얼마나 쌓아 놓을 지를 결정하는 것이다.
        self.publisher = self.create_publisher(Twist, 'skidbot/cmd_vel', 10)

        # 이것을 0.5초로 설정하여 
        # 0.5초의 간격으로 publish_callback함수가 실행된다.
        timer_period = 0.5  # seconds

        # create_timer함수가 timer_period의 주기로 publish_callback함수를 실행한다.
        self.timer = self.create_timer(timer_period, self.publish_callback)

        # 다음과 같은 문자열 로그를 터미널에 표시한다.
        self.get_logger().info(
            'DriveForward Node Started, move forward endless \n'
        )

    def publish_callback(self):
        # Twist 메시지 객체 생성.
        twist_msg = Twist()

        # Twist 메시지 내용 설정.
        twist_msg.linear.x = 0.5
        twist_msg.angular.z = 1.0

        # twist_msg를 publish
        self.publisher.publish(twist_msg)

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)


def main(args=None):
    # ROS Node가 동작하기 위해 우선 현 상태등을 알아야하는 등의 정보가 필요하다.
    # 따라서 rclpy class의 초기화를 해준다.
    rclpy.init(args=args)

    # 노드를 생성한다.
    cmd_vel_publisher = CmdVelPublisher()

    try:
        # rclpy에게 이 node를 반복(= spin)해서 실행하라고 전달한다.
        rclpy.spin(cmd_vel_publisher)
    except KeyboardInterrupt:
        # ROS에서 제공하는 get_logger()이라는 함수이다.
        # 콘솔에 로그를 찍고 싶을 때 사용한다.
        cmd_vel_publisher.get_logger().info('==== Server stopped cleanly ====')
    except BaseException:
	    # 예외 발생시 에러 메시지를 로그에 띄운다.
        cmd_vel_publisher.get_logger().info('!! Exception in server:', file=sys.stderr)
        raise
    finally:
        cmd_vel_publisher.stop_robot() # 로봇의 모든 속도를 0으로 만드는 메시지를 publish한다.
        # (optional - Done automatically when node is garbage collected)
        # 아래 코드는 어차피 가비지 컬렉터가 수행할 것들이다.
        # 따라서 하든 안 하든 상관 없지만, 노드가 많다면 하는 편이 좋겠다.
        cmd_vel_publisher.destroy_node() 
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()
```

## Subscriber Node
시뮬레이션 안의 로봇이 가져오는 라이다 데이터를 터미널에 띄우는 노드입니다.

```python
# !/usr/bin/env/ python3

# 노드를 만들기 위한 import
import rclpy
from rclpy.node import Node

# 메시지 형식 import (여기에선 LaserScan을 사용한다.)
from sensor_msgs.msg import LaserScan

# import한 Node class를 상속하여 원하는 Node class를 만든다.
class LaserSubscriber(Node):

    def __init__(self):
	    # 노드 이름 설정
        super().__init__('laser_sub_node')
        # queue사이즈 결정
        queue_size = 10
        # Subscriber가 받을 메시지 자료형, 사용할 토픽, 
        # 자료를 받아올 콜백 함수 ,큐 사이즈를 설정하며
        # subscriber을 만든다.
        self.subscriber = self.create_subscription(
            LaserScan, 'skidbot/scan', self.sub_callback, queue_size
        )
        # self.subscriber  # prevent unused variable warning
        
	# callback함수로 메시지를 받아온다.
    def sub_callback(self, msg):
	    # 받아온 메시지를 로그에 띄운다.
        self.get_logger().info(f'Raw Laser Data : {msg.ranges}')


def main(args=None):
	# 노드를 위한 init
    rclpy.init(args=args)
	# 노드 생성
    laser_subscriber = LaserSubscriber()
    
	# 노드를 반복하여 실행한다.
    rclpy.spin(laser_subscriber)
    # 한번만 실행하고싶으면 rclpy.spin_once(<원하는 Node>)

	# 노드가 차지하는 리소스를 반환한다
    laser_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


```

## Subscriber, Publisher둘 다 쓰는 Node
이 노드는 라이다 데이터를 바탕으로 로봇이 직진하도록 합니다. 그러나, 로봇 앞의 장애물이 특정 거리 미만으로 줄어들었을 때 정지하도록 로봇에 메시지를 보냅니다. (전체적인 코드를 더 객체지향적으로 수정하였습니다.)

```python
# !/usr/bin/env/ python3

import sys

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ParkingNode(Node):

    def __init__(self):
        super().__init__('parking_node')
        
        self.publisher = self.create_publisher(Twist, 'skidbot/cmd_vel', 10)

        self.subscriber = self.create_subscription(
            LaserScan, 'skidbot/scan', self.sub_callback, 10
        )

        self.get_logger().info('==== Parking Node Started ====\n')

    def sub_callback(self, msg):
        self.ranges = msg.ranges
    
    def get_distance_forward(self):
        return self.ranges[360]
    
    def stop(self):
        stop_twist_msg = Twist()
        stop_twist_msg.linear.x = 0.0
        self.publisher.publish(stop_twist_msg)

    def drive_forward(self):
        drive_twist_msg = Twist()
        drive_twist_msg.linear.x = 0.5
        self.publisher.publish(drive_twist_msg)

    def park(self):
        while(True):
            rclpy.spin_once(self)
            if self.get_distance_forward() > 0.5:
                self.get_logger().info(f'Distance from Front Object : {self.get_distance_forward()}')
                self.drive_forward()
            else:
                self.get_logger().info('==== Parking Done!!! ====\n')
                self.stop()
                break
    

def main(args=None):
    rclpy.init(args=args)
    parking_node = ParkingNode()

    try:
        parking_node.park()
    except KeyboardInterrupt:
        parking_node.get_logger().info('==== Server stopped cleanly ====')
    except BaseException:
        parking_node.get_logger().info('!! Exception in server:', file=sys.stderr)
        raise
    finally:
        parking_node.stop()
        parking_node.get_logger().info('==== stopping Done ====\n')
        parking_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```

## 정리
### Node의 코드 구조
위의 코드를 통해 파이썬에서 Node는 다음과 같은 구조로 설계함을 알 수 있습니다.

1. 노드를 만들기 위해 `Node`, `rcply`를 import해준다.
2. 사용할 메시지 유형에 따라 관련 파이썬 파일을 import해준다.
3. Node class를 상속하여 새로운 노드 class를 선언한다.
4. 새 노드 class의 init함수에 함수 `super().__init__('<노드 이름>')`를 사용하여 그 노드의 이름을 설정한다.
5. init함수에 통신을 위한 subscriber, publisher등을 생성한다.
6. 이 노드의 기능들을 담당하는 함수를 class의 method로써 작성한다.  
   (그런데, main에서 그 기능을 추가해도 동작하는 데엔 상관없는 것같다.)

### Subscriber, Publisher 비교.
두가지 기능 모두 사용할 메시지 구조, 토픽 이름, queue size를 설정해야 합니다.

그러나 subscriber은 메시지를 받아올 때 callback함수와, `rclpy.spin()`또는 `rclpy.spin_once()`를 꼭 사용해야 합니다.  


### 이외의 함수
#### timer
만약 어떤 함수를 특정 주기로 실행하고 싶다면 `create_timer(<주기>,<함수>)` 와 `rclpy.spin()`를 사용하여 반복할 수 있다.

### 의문점
1. 만든 Node class를 다른 python file에서 사용할 수 있을까?
2. 하나의 python file에서 여러개의 노드를 실행시킬 수 있을까?

# Ref
[ROS 공식 Documents](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
[8, 9강 노트 링크](https://puzzling-cashew-c4c.notion.site/Topic-python-abf265c0af7b45c38264914d164d7349)

