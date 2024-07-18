# How to add code in package with python
[Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)에서 주로, xml, setup.py를 작성하는 방법을 다룹니다.

## 세팅
모든 ROS 2 작업을 하기 앞서 다음 bash를 실행합니다.
```bash
source /opt/ros/foxy/setup.bash
```


패키지를 만듭니다. 앞서 사용했던 workspace를 사용합니다.   
위치 : `~/ros2_ws/src`

```bash
ros2 pkg create --build-type ament_python py_pubsub
```

다음 위치로 옮겨 publisher와 subscriber py 파일을 받습니다.  
위치 : `ros2_ws/src/py_pubsub/py_pubsub`

```bash
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py

wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```

cpp에서와 다르게 코드가 안에 들어있을 것입니다. 파일 이름과 경로는 다음과 같습니다.
- `/ros2_ws/src/py_pubsub/py_pubsub/publisher_member_function.py`  
- `/ros2_ws/src/py_pubsub/py_pubsub/subscriber_member_function.py`


## package.xml 설정
`ros2_ws/src/py_pubsub`의 `package.xml`을 편집기로 들어갑니다.

들어가면, 다음과 같이 되어있을 것입니다.

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>py_pubsub</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">root</maintainer>
  <license>TODO: License declaration</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

여기에 `<description>`, `<maintainer>`, `<license>`와 같은 tag들을 다음과 같이 채워줍니다.

```xml
<description>Examples of minimal publisher/subscriber using rclcpp</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

그 다음 줄에 코드가 실행될 때 필요한 패키지들을 다음과 같이 추가합니다.

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

그러면 최종적으로 다음과 같이 될 것입니다.

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>py_pubsub</name>
  <version>0.0.0</version>
  <description>Examples of minimal publisher/subscriber using rclpy</description>
  <maintainer email="jinu1423@naver.com">root</maintainer>
  <license>Apache License 2.0</license>

  <!--여기에 실행할 때 필요한 디펜던시 추가-->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

```


## setup.py 설정

이 파일은 처음에 다음과 같이 되어있을 것입니다.

```python
from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='you@email.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

xml 파일에서 했던것과 같이 이름, 이메일, description, license를 채워줍니다.

```python
maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
```

다음과 같이 `entry_points`내용도 채워줍니다.

```python
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
        ],
    },
```

채우는 내용은 아마도 `'<명령어에 쓸 노드 이름>' ='<패키지 이름>.<사용할 코드 파일 이름>:<이 코드를 실행할 때 무엇으로써 실행할 것인가>'` 이 아닐까 생각합니다. 결국, 노드와 코드를 매칭시키는 부분으로 보입니다. 

모두 작성하였다면 다음과 같이 되었을 것입니다.

```python
from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinoo',
    maintainer_email='jinu1423@naver.com',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache Licence 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
        ],
    },
)

```

## setup.cfg

```cfg
[develop]
script-dir=$base/lib/py_pubsub
[install]
install-scripts=$base/lib/py_pubsub
```

이 파일은 자동으로 생성되어 편집할 것은 없습니다.  

이 파일은 생성한 executable들을 `lib` 디렉토리에 넣을 것을 지시합니다. 따라서 `ros2 run`이 그 executable들을 사용할 수 있게 됩니다.


## build & run

디펜던시 체크를 합니다.  
위치 : `/ros2_ws`

```bash
rosdep install -i --from-path src --rosdistro foxy -y
```

새로 만든 패키지를 build합니다.

```bash
colcon build --packages-select py_pubsub
```

새 터미널을 띄우고 적절한 환경을 만듭니다.

```bash
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws
. install/setup.bash
```

노드를 실행하여 잘 동작하는지 확인합니다.

터미널 1 : 
```bash
ros2 run py_pubsub talker
```


터미널 2 :

```bash
ros2 run py_pubsub listener
```