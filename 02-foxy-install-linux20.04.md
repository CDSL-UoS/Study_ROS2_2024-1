# 2강. ros2 foxy install
[강의 노트 링크](https://puzzling-cashew-c4c.notion.site/ROS-2-Foxy-Linux20-04-58f0c6f2537e498eb8fe163ad1f13ce5)
리눅스를 설치하는 건 넘어가겠습니다. 

## 편의성 프로그램 설치
리눅스 20.04를 깔고 다음 프로그램들을 깔면 조금 더 편하게 진행할 수 있습니다.
1. terminator
   ```bash
   sudo apt update
   sudo apt install terminator -y
	```

2. vscode
	https://code.visualstudio.com/Download
	이 링크에서 vscode를 다운받아 설치하는 것을 추천합니다. 리눅스용으로 받아야 하며, 확장자명이 `.deb`인 설치 파일을 받아야 합니다.


## ROS 2 Foxy 설치
[ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

1. locale 설정  
   What is locale?   
   Locale is basically **a set of environmental variables that defines the user's language, region, and any special variant preferences that the user wants to see in their Linux interface**
    
	``` bash
	locale  # check for UTF-8

	sudo apt update && sudo apt install locales
	sudo locale-gen en_US en_US.UTF-8
	sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
	export LANG=en_US.UTF-8

	locale  # verify settings
	```

2. Setup Sources
	```bash
	# First ensure that the Ubuntu Universe repository is enabled.
	sudo apt install software-properties-common
	sudo add-apt-repository universe

	# Now add the ROS 2 GPG key with apt.
	sudo apt update && sudo apt install curl -y
	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

	# Then add the repository to your sources list.
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	```


3. Install ROS 2 packages
	```bash
	sudo apt update
	sudo apt upgrade
	
	# Desktop Install (Recommended): ROS, RViz, demos, tutorials.
	sudo apt install ros-foxy-desktop python3-argcomplete
	
	```

4. 추가 패키지 설치
	```bash
	sudo apt install ros-foxy-rqt*
	sudo apt install ros-foxy-image-view
	sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup 
	sudo apt install ros-foxy-joint-state-publisher-gui
	sudo apt install ros-foxy-xacro
	```

5. ROS가 동작하는지 test  
   터미널 두 개를 켜고 각각 다음 명령어를 입력합니다.  
   
   터미널 1 :   
	```bash
	source /opt/ros/foxy/setup.bash
	ros2 run demo_nodes_cpp talker
	```
	터미널 2:  
	```bash
	source /opt/ros/foxy/setup.bash
	ros2 run demo_nodes_py listener
	```
	이렇게 하면 터미널 하나가 메시지를 publishing하면 다른 터미널이 메시지를 받는 것을 실시간으로 볼 수 있습니다.
## Gazebo 11 설치
```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt update

sudo apt install gazebo11 libgazebo11-dev -y

sudo apt install ros-foxy-gazebo-ros-pkgs -y
```

이후 Gazebo 실행은 다음 명령어로 합니다.
```bash
gazebo
```


## .bashrc 
홈 디렉토리에 있는 `.bashrc`파일에 `alias`를 설정해 놓으면 터미널에서 명령어를 간단하게 입력할 수 있습니다.

`.bashrc`파일의 윗부분은 건드리지 말고 아래에 추가로 작성하면 됩니다.

```bash
alias eb='gedit ~/.bashrc'
alias sb='source ~/.bashrc'

alias cba='colcon build --symlink-install'
alias cbp='colcon build --symlink-install --packages-select'
alias killg='killall -9 gzserver && killall -9 gzclient && killall -9 rosmaster'

alias rosfoxy='source /opt/ros/foxy/setup.bash && source ~/gcamp_ros2_ws/install/local_setup.bash'

source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/gcamp_ros2_ws
```