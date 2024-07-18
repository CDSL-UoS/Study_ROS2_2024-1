# [ROS 2 Study] Ch. 15. Action

## Action이란?
- ROS 2의 3가지 주요 통신 방식(topic, service, action) 중 하나   
- Action은 service와 기본적으로 유사하게 동작하지만, service에는 없는 feedback의 개념이 추가됨. ![Diagram for action](https://docs.ros.org/en/foxy/_images/Action-SingleActionClient.gif) 
  - **Action client** : Server에게 request를 보내고 result를 받음.
  - **Action server** : Client에게 result를 보내고, 그 과정에서 지속적으로 feedback topic을 전송함. (이를 위해 multi-threading의 개념이 도입되어야 할 수 있음.)
- 개념적으로 service와는 달리 action client는 action goal을 전송한 이후에는 다른 작업을 수행할 수 있음. 
  - 이는 로봇 제어 알고리즘, 차량 경로 생성 알고리즘 등 계층적인 구조 도입이 필요한 상황에서 유용함. 
  - 예를 들어 로봇 팔을 특정 위치로 제어하는 문제에서, 로봇이 추종해야 할 지령 정보(reference)를 action client가 생성하여 action server에 넘겨주고, action server에 있는 제어기가 로봇 팔에 장착된 모터를 움직이는 방식으로 구현 가능.
- (https://design.ros2.org/articles/actions.html) 
- (https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)



## Example 1 : Fibonacci (Customized by Road-Balance)

- Fibonacci 수열의 결과를 계산하는 간단한 예제
- 자세한 설명은 강의 교안의 내용 참고 (https://puzzling-cashew-c4c.notion.site/ROS-2-Action-e8d4f184344c4a9fb41b0b6a1b6cf10c)

## Example 2 : Turtlesim

- Turtlesim은 가장 잘 알려진 action 예제 중 하나(https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- Terminal 2개를 열어서 turtlesim_node와 turtle_teleop_key를 각각 실행시켰을 때, turtle_teleop_key를 실행시킨 터미널에서는 아래의 메세지가 출력됨
    ```console
    $ ros2 run turtlesim turtle_teleop_key
    Reading from keyboard
    ---------------------------
    Use arrow keys to move the turtle.
    Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
    'Q' to quit.
    ```
- 만약 turtle_teleop_key에서 rotation과 관련된 key 중 하나를 입력하였다면, turtlesim_node 화면에서는 turtlebot이 회전하고 있으며 동시에 터미널에는 다음이 출력됨
  ```console
  [INFO] [1713382448.026251352] [turtlesim]: Rotation goal completed successfully
  ```
- 실제로 이러한 과정은 rotate_absolute라는 action을 통해서 구현되는데, 실제로 action list를 통해서 확인해볼 수 있음.
    ```console
    $ ros2 action list
    /turtle1/rotate_absolute
    ```
- 조금 더 구체적으로 살펴보자면, 현재의 시뮬레이션에서 /turtlesim과 /teleop_key 라는 node들은 각각 action server와 action client로 동작.
  - **/turtlesim**
    ```console
    $ ros2 node info /turtlesim
        /turtlesim
            Subscribers:
                ... # 생략
            Publishers:
                ... # 생략
            Service Servers:
                ... # 생략
            Service Clients:

            Action Servers:
                /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
            Action Clients:
    ```
  - **/teleop_turtle**
    ```console
    $ ros2 node info /teleop_turtle
        /teleop_turtle
        Subscribers:
            ... # 생략
        Publishers:
            ... # 생략
        Service Servers:
            ... # 생략
        Service Clients:

        Action Servers:

        Action Clients:
            /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
    ```
  - 위의 상황은 action info 명령을 통해 확인 가능
      ```console
      $ ros2 action info /turtle1/rotate_absolute
          Action: /turtle1/rotate_absolute
          Action clients: 1
              /teleop_turtle
          Action servers: 1
              /turtlesim
      ```
- Action interface는 크게 *goal request*, *feedback*, *result*로 구성되어 있으며, 아래와 같이 확인 가능 (각각은 --- 를 통해 구분)
    ```console
    $ ros2 interface show turtlesim/action/RotateAbsolute
        # The desired heading in radians
        float32 theta
        ---
        # The angular displacement in radians to the starting position
        float32 delta
        ---
        # The remaining rotation in radians
        float32 remaining
    ```
    - **`theta`** : Action client(=/teleop_turtle)가 처음 action server(=/turtlesim)에게 회전을 요청할 때(즉, goal request할 때) 사용되는 값.
    - **`delta`** : 시작점으로부터 얼마나 회전했는지를 action server(=/turtlesim)이 실시간으로 계산한 결과
    - **`remaining`** : Action이 종료된 이후 회전의 결과를 출력 
- Package에서 제공하는 함수를 활용하는 것 이외에도, terminal에서 아래와 같이 `action send_goal` 함수를 사용하여 action goal을 action server에게 요청할 수 있음. (여기서 `<value>`는 YAML 포맷으로 작성되어야 함)
    ```console
    $ ros2 action send_goal <action_name> <action_type> <value>
    ```
    Turtlesim 예제에서는 다음의 명령어를 terminal에 입력하여 수행. (실제 turtlesim 화면에서도 1.57 rad만큼 회전하는 것을 확인 가능)
    ```console
    $ ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
        Waiting for an action server to become available...
        Sending goal:
            theta: 1.57

        Goal accepted with ID: d32ef1d4b1fc4ed3a8ac67e49bbcfa07

        Result:
            delta: -1.5520002841949463

        Goal finished with status: SUCCEEDED
    ```