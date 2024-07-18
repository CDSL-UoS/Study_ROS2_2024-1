# Service programming with python
## client node 만들기 1
이 client node는 skidbot 하나를 원하는 위치에 소환하기 위해 시뮬레이션 서버에 server call을 하는 노드이다.
```python
# !/usr/bin/env/ python3
import os
# 패키지 디렉토리 내부 파일에 접근하기 위해(?) import.
# 이를 통해 패키지의 파일 시스템에 접근할 수 있다. 
# 여기에선 'gcamp_gazebo/urdf'에 접근한다.
from ament_index_python.packages import get_package_share_directory

# client가 사용할 SpawnEntity라는 srv 타입을 import
from gazebo_msgs.srv import SpawnEntity

# node를 만들기 위한 imports
import rclpy
from rclpy.node import Node

# node class 상속
class SpawnRobot(Node):
    # 생성자 선언
    def __init__(self):
        # 부모 class 생성자 사용 - 노드 이름 설정
        super().__init__('gazebo_model_spawner')
        # client 생성. 입력으로 'srv 타입 class', 'request를 보낼 Service 이름'을 사용한다.
        self.client = self.create_client(SpawnEntity, '/spawn_entity')

        # Server가 답이 없을 때 기다릴 시간을 지정한다.
        # 지정한 시간동안 서비스의 답장이 없으면 False를 반환하나보다.
        # server을 실행하지 않았을 때 1초마다 반복된다.
        while not self.client.wait_for_service(timeout_sec=1.0):
            # 다 기다렸는데 안 오면 다음과 같은 에러 메시지를 로그에 띄운다.
            self.get_logger().error('service not available, waiting again...')

        # 등장시킬 로봇의 외형을 담은 파일 경로 접근.
        # Get urdf path
        self.urdf_file_path = os.path.join(
            get_package_share_directory('gcamp_gazebo'),
            'urdf',
            'skidbot2.urdf',
        )

        # 보낼 SpawnEntitiy srv 형식을 가진 request 객체 선언
        self.req = SpawnEntity.Request()

    # request 내용 설정하는 함수
    def send_req(self):
        # request의 각 매개변수를 설정한다.
        # urdf 파일 경로와 로봇을 소환할 위치정보 등등
        self.req.name = 'skidbot2'
        self.req.xml = open(self.urdf_file_path, 'r').read()
        self.req.robot_namespace = 'skidbot2'
        self.req.initial_pose.position.x = 1.0
        self.req.initial_pose.position.y = 1.0
        self.req.initial_pose.position.z = 0.3
        
        # request call 이후 상태를 반환하는 future을 예약받는다.
        self.get_logger().debug('==== Sending service request to `/spawn_entity` ====')
        # future이란, 해당 작업이 완료될 것임의 '약속'이다.
        # 이건 main에서 더 알아보자.
        self.future = self.client.call_async(self.req)

        # future 반환
        return self.future


def main(args=None):
    # rclpy 초기화
    rclpy.init(args=args)
    
    # request하는 node 객체 생성.
    robot_spawn_node = SpawnRobot()
    # request를 보내고, future 받아온다.
    # 이 robot_spqwn_node.send_req()는 반드시 끝날 것을 약속하므로 
    future = robot_spawn_node.send_req()
    # 약속을 지킬 때까지 robot_spawn_node를 spin하고 있을 것이라는 의미이다.
    rclpy.spin_until_future_complete(robot_spawn_node, future)

    # request cll, response가 끝난 상황.
    # 이때, futrue에는 response가 있다.
    if future.done():
        try:
            response = future.result()
        except Exception:
            # 예외 처리
            raise RuntimeError(
                'exception while calling service: %r' % future.exception()
            )
        else: # try가 정상 작동하면 실행하는 것.
            # 성공하였음을 표시하고 받은 메시지를 보인다.
            robot_spawn_node.get_logger().info('==== Service Call Done ====')
            robot_spawn_node.get_logger().info(f'Status_message : {response.status_message}')
        finally:
            # 노드를 끈다.
            robot_spawn_node.get_logger().warn('==== Shutting down node. ====')
            robot_spawn_node.destroy_node()
            # rclpy 종료.
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## client node 만들기 2
Turning Server에 request를 보내 원하는 값으로 움직이도록 하는 client node이다.
```python
# !/usr/bin/env/ python3
# 예제를 위해 만들어진 custom srv
from custom_interfaces.srv import TurningControl

# 노드를 만들기 위한 필수 import
import rclpy
from rclpy.node import Node

# class 상속
class RobotTurnClient(Node):

    def __init__(self):
        super().__init__('robot_turn_client')
        self.client = self.create_client(TurningControl, 'turn_robot')  # CHANGE

        # 서비스가 사용 가능할 때까지 기다린다. 
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # 보낼 형식의 srv 객체를 선언한다.
        self.req = TurningControl.Request()
        self.get_logger().info('==== Robot Turn Service Client ====')

    def send_request(self):
        # 보낼 srv 내용을 작성한다.
        while True:
            try:
                td = input('> Type turning time duration: ')
                vel_x = input('> Type turning linear velocity: ')
                vel_z = input('> Type turning angular velocity: ')

                if float(vel_z) > 1.5707 or float(vel_x) > 3:
                    raise ArithmeticError('Velocity too high !!')

                self.req.time_duration = int(td)

                self.req.linear_vel_x = float(vel_x)
                self.req.angular_vel_z = float(vel_z)
                break
            except ArithmeticError as e:
                self.get_logger().warn(e)
            except Exception as e:
                self.get_logger().warn(e)
                self.get_logger().warn('Not a number, PLZ Type number Again')

        # request를 보내고, 약속(future)을 받는다.
        self.future = self.client.call_async(self.req)
        self.get_logger().info(
            f'linear_x : {self.req.linear_vel_x} / angular_z : {self.req.angular_vel_z}'
        )
        self.get_logger().info(' Request Sended ')
        # future을 return
        return self.future


def main(args=None):
    rclpy.init(args=args)

    # client node 선언, request를 보내고 future이 올 때까지 기다린다.
    robot_turn_client = RobotTurnClient()
    future = robot_turn_client.send_request()
    rclpy.spin_until_future_complete(robot_turn_client, future)

    # future이 안 끝나면 spin을 못 나오는 거 아닌가? 왜 if 문을 사용한 것일까?
    if future.done():
        # result를 받아온다.
        try:
            response = future.result()
        # 예외 사항이 발생하면 예외 상황을 표시한다.
        except Exception:
            raise RuntimeError(
                'exception while calling service: %r' % future.exception()
            )
        # 제대로 받아왔다면 성공했다고 표시한다.
        else:
            robot_turn_client.get_logger().info('==== Service Call Done ====')
            robot_turn_client.get_logger().info(
                f"Result Message : {'Success' if response.success == True else 'Fail'}"
            )
        # 마지막으로 노드를 종료한다.
        finally:
            robot_turn_client.get_logger().warn('==== Shutting down node ====')
            robot_turn_client.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

```

## Service 만들기
이 service는 client로부터 로봇의 속도를 받아 시뮬레이션에 topic을 사용하여 전달(Publish)한다.
```python
# !/usr/bin/env/ python3

# srv type import
from custom_interfaces.srv import TurningControl
# msg type import
from geometry_msgs.msg import Twist
# import for node
import rclpy
from rclpy.node import Node


# srv 내용
# uint32 time_duration
# float64 angular_vel_z
# float64 linear_vel_x
# ---
# bool success


class RobotTurnServer(Node):

    def __init__(self):
        super().__init__('robot_turn_server')
        # 퍼블리셔 생성
        self.publisher = self.create_publisher(Twist, 'skidbot/cmd_vel', 10)
        # 서비스 생성. srv type, srv name, cllback함수 를 인풋으로 받는다.
        self.srv = self.create_service(
            TurningControl, 'turn_robot', self.robot_turn_callback
        )
        # topic으로 보낼 메시지 객체 생성
        self.twist_msg = Twist()
        # 현재 시간을 초로 받아 저장한다.
        self.start_time = self.get_clock().now().to_msg().sec

        # 서버가 시작되어 request를 기다림을 알린다.
        self.get_logger().info('==== Robot Turning Server Started, Waiting for Request ====')

    # 로봇을 움직이도록 속도 정보(Twist)를 publish하는 함수
    def move_robot(self, seconds=1, linear_x=0.0, angular_z=0.0):
        # 속도 지정
        self.twist_msg.linear.x = linear_x
        self.twist_msg.angular.z = angular_z

        # 현재 시각 받아온다.
        clock_now = self.get_clock().now().to_msg().sec
        # 로봇이 어떻게 움직이는 지 알린다.
        self.get_logger().info('Robot Moves')
        self.get_logger().info(f'Move Commands = linear_x : {linear_x} / angular_z : {angular_z}')
        # 지정한 속도로 움직이도록 publish한다.
        self.publisher.publish(self.twist_msg)
        # 지정했던 시간동안 이 함수에서 머물도록 한다. 
        while (clock_now - self.start_time) < seconds:
            clock_now = self.get_clock().now().to_msg().sec
            

    # 로봇이 멈추도록 하는 함수
    def stop_robot(self):
        # 모든 속도가 0인 Twist_msg로 만든다.
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0

        # publish하여 멈추도록 한다.
        self.publisher.publish(self.twist_msg)
        # 로봇이 멈췄음을 알린다.
        self.get_logger().info('Robot Stop')

    # 서비스를 요청받았을 때 실행할 함수
    def robot_turn_callback(self, request, response):
        # 현재 시간을 받는다.
        self.start_time = self.get_clock().now().to_msg().sec

        # 로봇이 지정한 시간, 속도로 움직이도록 한다.
        self.move_robot(
            request.time_duration, request.linear_vel_x, request.angular_vel_z
        )
        # 로봇이 멈추도록 한다.
        self.stop_robot()

        # response를 True로 설정하여 제대로 동작했음을 표시한다.
        response.success = True
        self.get_logger().info('Servie Process Done...')

        return response


def main(args=None):
    rclpy.init(args=args)

    # 노드 생성
    robot_turn_server = RobotTurnServer()

    # 노드 실행
    rclpy.spin(robot_turn_server)

    # 노드 제거
    robot_turn_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```


## 요약
### Client
#### class 작성
##### `__init__` 작성
1. client 객체 생성.  
   `self.client = self.create_client(<srv type>, <srv name>)`

2. 이때 init함수에서 서비스를 기다리기 위해 다음과 같은 명령을 사용하면 좋다.  
   ```python
	  # Server가 답이 없을 때 기다릴 시간을 지정한다.
      # 지정한 시간동안 서비스의 답장이 없으면 False를 반환하나보다.
      # server을 실행하지 않았을 때 1초마다 반복된다.
	  while not self.client.wait_for_service(timeout_sec=1.0):
          self.get_logger().info('service not available, waiting again...')
	```

##### send_request() 작성
1. request 객체 생성  
   `self.req = <srv type>.Request()`

 2. request 작성  
    `self.req.<req 내용> = <값>`

3.  service call  
	`self.future = self.client.call_async(<Node>.req)`  
	future 변수에 response가 도착하였는지에 대한 정보와, response 내용 등이 들어있거나 들어있을 예정이다.

4. response 사용
   `self.futrure.result()`의 return이 response이다.
#### 사용
1.  다른 함수에서 사용할 때  
	   ```python
   	   # client node 선언, request를 보내고 future이 올 때까지 기다린다.
	    <객체 변수> = <Node>()
	    future = <객체 변수>.send_request()
	    rclpy.spin_until_future_complete(<객체 변수>, future)
	```
	
2. `try`문 사용하면 좋음
### Service
1. Node class 내 `__init__()`에서 서비스 객체 생성  
   `self.srv = self.create_service(<srv type>,<srv name>,<callback function>)`
2. callback 함수는 인풋으로 `request`, `response`를 받는다.
   `request`에서 request 정보를 받고, `response`(객체임)를 편집하여 return한다.
   `response.success = True`로 해주어야 다 됐는지 알 수 있다.
   ```
	   callback(self, request, response):
			   <사용> = request.<어쩌구>
			   response.<저쩌구> = <편집>
			   response.success = True
	```
3. 실행   
   `rclpy.spin(<Node>)`