# Service
![](./img/service2.gif) 출처 : [링크](https://docs.ros.org/en/foxy/Tutorials/Services/Understanding-ROS2-Services.html)

## 특징
- Client가 Server에게 Request를 하면(이걸 Server Call이라 한다.) Server에서 그 Client에게 Response를 하는 구조.  
- 1:1 통신이다.
- 분명한 요청의 주체가 있고, 빠르게 동작이 완료되는 경우에 쓰인다.
- Server는 한번에 하나의 Request만 처리 가능하다. Request 두 개가 들어오면 나중에 온 하나는 기다려야 한다.
- Topic은 `msg`타입을 사용한다면 Service는 `srv`라는 타입을 사용한다.
## 서비스 정보 찾는 명령어
- Service list 보기
	```bash
	ros2 service list
	```

- Service의 srv 타입 알아내기
	```bash
	ros2 service type <서비스 이름>
	```

-  특정 srv 타입을 사용하는 Service 찾기
	```bash
	ros2 service find <srv 타입>
	```

- 특정 srv 타입의 정보 보기
	```bash
	ros2 interface show <srv 타입>
	```

- Service call 하기 예시
	```bash
	ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'skidbot'}"
	```

