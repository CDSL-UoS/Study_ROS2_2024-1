# [Ch. 6] ROS 2 Launch

## 참고 자료

[ROS 2 Launch, launch file 작성](https://www.notion.so/ROS-2-Launch-launch-file-55c2125808ef4b64bade278852b37d6e?pvs=21)

## Launch File이란?

- ROS 2 node를 포함한 다수의 실행파일을 동시에 실행시킬 수 있도록 하는 파일
- ROS 1에서는 xml 문법으로 작성되었지만, ROS 2에서는 Python 기반으로 작성
- `<파일명>.launch.py` 형식의 파일명을 가짐

## 작성 방식 1 : ExecuteProcess 방식

- `ExecuteProcess()` 명령을 나열하여 작성하는 방식
- 아래 예시처럼 작성
    
    ```python
    ExecuteProcess(
                    cmd=[ "ros2", "service", "call", "/spawn_entity", "gazebo_msgs/SpawnEntity", spwan_args ],
                    output="screen",
                )
    ```
    

## 작성 방식 2 : Node 방식

- 다수 node를 생성하는 방식
- 아래 예시처럼 node를 생성할 때 필요한 package, parameter, argument 등을 지정해줄 수 있다.
    
    ```python
        turtlesim_node = Node(
            package='turtlesim',
            executable='turtlesim_node',
            parameters=[],
            arguments=[],
            output="screen",
        )
    ```
    
- 기본 문법
    - **`Node`** : 하나의 node를 실행
    - `**package**` : 실행하는 node가 포함된 package
    - `**executable**` : c++ 문법에서 colcon build할 때 생성되는 exe 파일에 대응되는 인자
    - `**parameters**` : node 생성에 필요한 추가 매개변수

## 매개변수 관리

- ROS 2 시스템에서는 `.yml` 파일 안에 구동에 사용되는 여러 매개변수들을 정리하고, 이를 launch 파일에서 불러올 수 있다.
- yaml 파일에서 매개변수를 선언하는 방식은 다음과 같다.
    
    ```yaml
    nanosaur_camera:
      ros__parameters:
    
        frame_id: 'camera_optical_frame'
    
        camera:
          device: "0"
          frameRate: 60.0
          width: 1280
          height: 720
    ```
    
- Launch 파일에서는 `LaunchConfiguration` 함수를 이용하여 camera.yml 파일의 경로를 지정해주고, node 생성 시 parameters에서 이를 활용한다.
    
    ```python
    camera_dir = LaunchConfiguration(
        'camera_dir',
        default=os.path.join(pkg_camera, 'param', 'camera.yml'))
    
    camera_node = launch_ros.actions.Node(
        package='nanosaur_camera',
        executable='nanosaur_camera',
        name='nanosaur_camera',
        parameters=[camera_dir],
        output='screen'
    )
    
    ```