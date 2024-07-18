# How to make a ROS 2 package
[ROS 2 documentation- Creating a package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
패키지는 CMake(cpp)와 python으로 만들 수 있습니다. 
이를 진행하기 앞서, workspace 만들면서 사용한 디렉토리를 사용합니다.

```bash
cd ~/ros2_ws/src
```

## 패키지 최소 요구 내용물

패키지는 다음과 같은 최소한의 파일들이 있어야 합니다.

**CMake** : 
- `CMakeLists.txt` : 패키지 안의 code들을 어떻게 build하는지 알리는 파일.
- `include/<package_name>` : 패키지의 public header들을 가지고 있는 디렉토리.
- `package.xml` : 패키지에 대한 메타 정보를 가지는 파일.
- `src` : 패키지를 위한 소스 코드를 가지고있는 디렉토리.

**Python** : 
- `package.xml` : 패키지에 대한 메타 정보를 가지는 파일.
- `resource/<package_name>` : 패키지 마커(?) 파일
- `setup.cfg` : `ros2 run` 명령어를 사용할 때 패키지에서 executable을 찾을 수 있도록 하는 파일
- `setup.py` : 패키지를 어떻게 install 하는지에 대한 instruction을 포함하고 있는 파일.
- `<package_name>` : 패키지와 같은 이름을 가진 디렉토리. ROS 2 tool들이 이 패키지를 찾을 때 쓰인다. 이 디렉토리는 `__init__.py`를 가진다.

그렇다면 제일 간단한 패키지는 다음과 같은 파일 구조를 가질 것입니다.

**CMake** : 
```bash
my_package/
     CMakeLists.txt
     include/my_package/
     package.xml
     src/
```

**Python** : 
```bash
my_package/
      package.xml
      resource/my_package
      setup.cfg
      setup.py
      my_package/
```

## Packages in a workspace
하나의 workspace는 여러개의 패키지들을 원하는 만큼 포함할 수 있습니다. 
그 친구들은 모두 workspace 안의 src 디렉토리에 있게 됩니다.

따라서 workspace는 다음과 같은 구조를 가질 것입니다.
```bash
workspace_folder/
    src/
      cpp_package_1/
          CMakeLists.txt
          include/cpp_package_1/
          package.xml
          src/

      py_package_1/
          package.xml
          resource/py_package_1
          setup.cfg
          setup.py
          py_package_1/
      ...
      cpp_package_n/
          CMakeLists.txt
          include/cpp_package_n/
          package.xml
          src/
```
## Tasks
### Create a package
위치 : `~/ros2_ws/src`

새로운 패키지를 만드는 명령어는 다음과 같습니다.

**CMake** : 
```bash
ros2 pkg create --build-type ament_cmake <package_name>
```

**Python** :
```bash
ros2 pkg create --build-type ament_python <package_name>
```

하지만, 이 튜토리얼에서는 `--node-name`이라는 옵션을 사용하여 간단한 Hello World 타입의 executable을 패키지 안에 생성할 것입니다.

**CMake** : 
```bash
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

**Python** :
```bash
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

이러면, `src`디렉토리 안에 `my_package`가 있을 것입니다. 여기에서 자동으로 생성된 패키지 파일들을 볼 수 있습니다.

### Build a package
패키지를 build하는 건 간단합니다.
workspace 디렉토리에서 
```bash
colcon build
```
를 사용하면 됩니다. 이걸 사용하면 workspace안에 있는 모든 패키지가 build됩니다. 만약 몇몇 패키지를 골라서 build하고 싶으면 다음 옵션을 사용하여 하면 됩니다.

여기에선 새로 만든 패키지가 `my_package`이므로 이걸 build하겠습니다.
```bash
colcon build --packages-select my_package
```

### Source the setup file
새로 만든 패키지를 사용하려면, 새로운 터미널에서 main ROS 2 installation bash를 실행합니다. 
```bash
source /opt/ros/foxy/setup.bash
```

다음 `ros2_ws`디렉토리에서 다음 명령어를 실행합니다.
```bash
source install/local_setup.bash
```

이러면, 이 workspace는 경로로 추가되어 새로 만든 패키지의 executable들을 사용할 수 있게 된 것입니다.


### Use the package
`--node-name`이라는 옵션을 사용하여 생성한 패키지의 executable을 실행하려면, 다음 명령어를 입력합니다.

```bash
ros2 run my_package my_node
```

그러면, 다음과 같은 출력을 볼 수 있습니다.

**CMake** : 
```bash
hello world my_package package
```

**Python** :
```bash
Hi from my_package.
```
### Examine package contents
`ros2_ws/src/my_package`에서 `ros2 pkg create`라는 명령어를 통해 자동으로 생성된 다음과 같은 파일과 디렉토리를 볼 수 있을 것입니다.

**CMake** : 
```bash
CMakeLists.txt  include  package.xml  src
```
src 디렉토리 내에는 `my_node.cpp`가 있습니다. 이것이 나중에 만들 노드가 되는 것입니다.

**Python** :
```bash
my_package  package.xml  resource  setup.cfg  setup.py  test
```
src 디렉토리 내에는 `my_node.py`가 있습니다. 이것이 나중에 만들 노드가 되는 것입니다.


### Customize package.xml
패키지를 만들 때 description이나 license를 봤을 것입니다. 이는 package description과 license declaration이 자동으로 되지 않기 때문입니다. 하지만 이런 것들은 패키지를 배포하고 싶으면 필요한 것들입니다. 또한 maintainer 란도 채워져야 합니다.

여기에선 `ros2_ws/src/my_package`에 `package.xml`파일을 편집하면 됩니다.
처음 들어가면 다음과 같이 되어있을 것입니다.

**CMake** : 
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

**Python** :
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

 <test_depend>ament_copyright</test_depend>
 <test_depend>ament_flake8</test_depend>
 <test_depend>ament_pep257</test_depend>
 <test_depend>python3-pytest</test_depend>

 <export>
   <build_type>ament_python</build_type>
 </export>
</package>
```

1. `maintainer` 태그 안에 이름과 email을 입력합니다.
2. `description` 태그 안에 패키지 설명하는 간단한 문장을 적습니다.
3. `license` 태그 안을 채웁니다. 오픈 소스 라이선스는 다음을 참고하세요.[오픈 소스 라이선스](https://opensource.org/license)
4. `*_depend` 태그 안에는 colcon으로 찾아볼 의존하는 패키지들의 리스트가 들어갑니다.

Python에만 있는 `setup.py`에는 `package.xml`처럼 채워야할 여러 정보가 들어갑니다. 따라서 그것들을 다음과 같이 모두 동일하게 채워 넣어야 합니다.

```python
from setuptools import setup

package_name = 'my_py_pkg'

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
 maintainer='TODO',
 maintainer_email='TODO',
 description='TODO: Package description',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'my_node = my_py_pkg.my_node:main'
     ],
   },
)
```