# How to make ROS 2 Workspace

참고자료 : [공식 Documentation](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
사실상 번역 수준..
## 우선 필요한 것들
- ROS 2 설치
- [colcon installation](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- git & git 사용법 숙지
- [tertlesim installation](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
- [rosdep installation](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Rosdep.html)
- 리눅스 사용법 숙지
- 텍스트 에디터(vs code 등)

## tasks
### ROS 2 환경 설정 적용

```bash
source /opt/ros/foxy/setup.bash
```


### 새 작업환경 디렉토리 생성

어디든 무슨 이름이든 디렉토리를 생성하면 됩니다.  
여기에선 `ros2_ws`라는 이름의 디렉토리를 home(`~`)에 추가합니다.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### git clone tutorial_package 

튜토리얼 패키지를 깃을 통해 가져옵니다.   
(위치 : `~/ros2_ws/src`)

```bash
git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
```

여기서 `-b`옵션은 다음에 오는 문자열의 이름을 가진 브랜치에서 클론을 하겠다는 옵션입니다.

### 디펜던시 확인
이 워크스페이스를 완성하기 전에 필요한 요소들을 다음 명령어를 통해 확인하고, 설치할 수 있습니다. 어떤 패키지를 받아서 build하기 전에 이 명령어를 실행하는 것이 정신건강에 좋을겁니다..

이 명령어는 `src`에서 실행하지 말고 반드시 해당 워크스페이스 디렉토리에서 실행하세요.  
(위치 : `~/ros2_ws`)
```bash
rosdep install -i --from-path src --rosdistro foxy -y
```

저같은 경우에는 다음 에러가 발생했습니다.

```bash
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
turtlesim: Cannot locate rosdep definition for [qtbase5-dev]
```

이 문제는 다음과 같은 명령어를 통해 해결할 수 있었습니다. (chat GPT를 사용하면 잘 나왔습니다.)

```bash
sudo rosdep init
rosdep update
```

 `sudo rdsdep init`에서는 에러가 났지만, `rosdep update`이후 잘 작동하는 것을 알 수 있었습니다.

이는 `rosdep`이 필요로하는 시스템 종속성을 해결할 수 없었고, 이중 `qtbase5-dev` 패키지를 찾지 못해 발생한 문제였다. 따라서 `rosdep`을 위와같이 최신 상태로 업데이트 하여 해결이 되었습니다.

잘 되면 다음과 같이 뜹니다!
```bash
#All required rosdeps installed successfully
```


### colcon을 통해 workspace build하기.
위치 : `~/ros2_ws`

다음과 같은 명령어를 입력합니다.

```bash
colcon build
```

그럼 다음과 같이 메시지가 뜨면서

```bash
Starting >>> turtlesim
Finished <<< turtlesim [5.49s]

Summary: 1 package finished [5.58s]
```

다음과 같이 환경이 만들어집니다. (위치 : `~/ros2_ws`)

```bash
ls
build  install  log  src
```

이때 `colcon build`에는 다음과 같은 옵션이 있으니 적절하게 사용하면 좋을 것입니다.

> [!colcon build options]
> - `--packages-up-to` : 원하는 패키지를 build합니다. 그리고 그 패키지의 모든 디펜던시도 같이 build합니다. 하지만, 전체 패키지를 가지고 workspace를 build하진 않습니다. 따라서 build하는 시간을 줄일 수 있습니다.
> - `--symlink-install` : python scripts를 수정할 때마다 rebuild하는 수고를 덜어줍니다.
> - `--event-handlers console_direct+` : building하는 도중 콘솔에 나오는 출력을 보여줍니다.

### Source the overlay
우선 "Source the overlay" 이 단어가 생소합니다. 이게 무엇을 의미하는 단어 집합일까요?  
이는 bash를 두 개 이상 실행(Source)한다(겹친다)는 뜻입니다. 또한, 처음 실행한 bash 파일은 underlay, 다음 실행한 bash 파일은 overlay로 지칭합니다.

build를 진행한 같은 터미널에서 Sourcing an overlay하는 것은 복잡한 문제를 야기할 수 있습니다. 따라서 새로운 터미널을 띄워서 다음 동작을 실행합니다. overlay 환경을 구성합니다.

```bash
source /opt/ros/foxy/setup.bash

cd ~/ros2_ws
source install/local_setup.bash
```

그리고 overlay환경에서 `turtlesim` 패키지를 실행해봅시다.

```bash
ros2 run turtlesim turtlesim_node
```

이것의 장점은 overlay 환경에서 build한 패키지와 underlay 환경에서 build 했던 패키지를 분리하여 사용할 수 있다는 것입니다. 

overlay 환경에서 `~/ros2_ws/src/ros_tutorials/turtlesim/src`에 있는 `turtle_frame.cpp`에서 52번째 줄에 있는 `setWindowTitle("TurtleSim");`를 `setWindowTitle("MyTurtleSim");`으로 바꾸면 다음과 같이 각각 실행하였을 때 turtlesim 창의 이름이 다르게 떠있는 것을 볼 수 있습니다.
![Screenshot from 2024-05-04 18-55-22.png](/img/Screenshot%20from%202024-05-04%2018-55-22.png)

작은 패키지를 가지고 workspace를 만들고 사용할 때 overlay환경에서 사용하는 것을 추천합니다. 따라서 모든 것을 같은 workspace안에 넣어 아주 큰 workspace를 기능을 추가할 때마다 rebuild할 필요가 없습니다.