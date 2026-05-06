================================================================================
 ur3_4040_urdf - 4040 알루미늄 모바일 매니퓰레이터 SLAM 자율 주행 패키지
================================================================================

  4륜 차동 구동 모바일 베이스 + UR3 매니퓰레이터 로봇(L_AND_F)에 대한
  ROS 2 Jazzy 기반의 자율 매핑 / 자율 주행 / 외부 미션 인터페이스 패키지.

  Gazebo Harmonic 시뮬레이션(warehouse_world)에서 검증되었으며,
  Gazebo 부분만 교체하면 동일 nav2 설정으로 실물 로봇에서도 동작 가능.


--------------------------------------------------------------------------------
 1. 환경 요구사항
--------------------------------------------------------------------------------

  - Ubuntu 24.04 (noble)
  - ROS 2 Jazzy
  - Gazebo Harmonic (gz sim)
  - Python 3.12

  추가 ROS 2 패키지 (apt 설치):
    ros-jazzy-slam-toolbox
    ros-jazzy-nav2-bringup
    ros-jazzy-ros-gz-bridge
    ros-jazzy-ros-gz-sim
    ros-jazzy-robot-state-publisher
    ros-jazzy-xacro

  Python 라이브러리:
    opencv-python  (clean_map.py 에서 사용)

  외부 ROS 패키지 (소스 빌드):
    m-explore-ros2  (https://github.com/robo-friends/m-explore-ros2)
      → ~/ur3/src/m-explore-ros2/  로 clone

  환경변수:
    export ROS_DOMAIN_ID=28


--------------------------------------------------------------------------------
 2. 처음 세팅 (조원이 새 PC 에서 GitHub 으로 받아와 시작할 때)
--------------------------------------------------------------------------------

  GitHub repo: https://github.com/innhyung/capstone

  아래 0~8 단계를 순서대로. 0번은 그 PC 에서 평생 한 번이면 됨.


  ── (0) 사전 요구사항 (OS / 시스템 패키지) ────────────────────────────────────

  Ubuntu 24.04 (noble) 에서 검증됨. ROS / Gazebo 가 이미 깔려있다면 스킵.

  # ROS 2 Jazzy
  #   공식 문서: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
  sudo apt install ros-jazzy-desktop

  # Gazebo Harmonic 연동
  sudo apt install ros-jazzy-ros-gz

  # 기타 도구
  sudo apt install python3-pip git python3-colcon-common-extensions python3-rosdep


  ── (1) 워크스페이스 clone ───────────────────────────────────────────────────

  git clone https://github.com/innhyung/capstone.git ~/ur3
  cd ~/ur3


  ── (2) third-party 패키지 (m-explore-ros2) 별도 clone ───────────────────────

  m-explore-ros2 는 라이선스/사이즈 이유로 .gitignore 에 들어있어서 별도 clone:

  git clone https://github.com/robo-friends/m-explore-ros2.git src/m-explore-ros2


  ── (3) ROS 2 환경 source + rosdep DB 초기화 ─────────────────────────────────

  source /opt/ros/jazzy/setup.bash
  sudo rosdep init      # "already initialized" 떠도 OK, 무시하면 됨
  rosdep update


  ── (4) ROS 의존성 자동 설치 ─────────────────────────────────────────────────

  package.xml 의 모든 exec_depend (slam_toolbox, nav2_bringup, ros_gz_bridge,
  robot_state_publisher 등) 가 이 한 줄로 다 깔림:

  cd ~/ur3
  rosdep install --from-paths src --ignore-src -r -y

  rosdep 옵션 의미:
    --from-paths src  : src/ 아래 모든 package.xml 스캔
    --ignore-src      : 워크스페이스 내부 패키지(예: m-explore-ros2)는 apt 로 안 받음
                        → 이미 src/ 에 있으니 colcon build 단계에서 빌드됨
    -r                : 일부 의존성 해결 실패해도 멈추지 말고 계속
    -y                : apt prompt 자동 yes

  참고: explore_lite 는 rosdep DB 에 없어 "explore_lite: not found" 경고가 떠도
        --ignore-src 로 무시되니 정상.


  ── (5) Python 라이브러리 (clean_map.py 용) ──────────────────────────────────

  pip install opencv-python


  ── (6) 워크스페이스 빌드 ────────────────────────────────────────────────────

  cd ~/ur3
  colcon build --symlink-install

  처음 빌드는 5~15분 정도. UR3 / m-explore-ros2 등이 같이 컴파일됨.


  ── (7) 환경 source + ROS_DOMAIN_ID ──────────────────────────────────────────

  매 새 터미널마다 아래 세 줄이 필요:

  source /opt/ros/jazzy/setup.bash
  source ~/ur3/install/setup.bash
  export ROS_DOMAIN_ID=28

  자주 새 터미널 여는데 매번 치기 귀찮으면 ~/.bashrc 끝에 한 번 박아두면 자동:

  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
  echo "source ~/ur3/install/setup.bash" >> ~/.bashrc
  echo "export ROS_DOMAIN_ID=28"          >> ~/.bashrc


  ── (8) 동작 확인 ────────────────────────────────────────────────────────────

  ros2 launch ur3_4040_urdf map_building.launch.py

  - Gazebo + RViz + SLAM + Nav2 + explore 가 다 떠야 정상.
  - 14초 자동 회전 후 robot 이 자율 매핑 시작.
  - 종료는 launch 터미널에서 Ctrl+C.


  ── 자주 막히는 지점 ─────────────────────────────────────────────────────────

    rosdep update 가 멈춤
        → 인터넷 + GitHub 도달 가능한지 확인.
    colcon build 에서 explore_lite 못 찾음
        → (2) 단계 m-explore-ros2 clone 누락. clone 후 다시 colcon build.
    ros2 launch 가 패키지 못 찾음
        → (7) 단계 source ~/ur3/install/setup.bash 안 한 새 터미널.
    Gazebo 안 뜨거나 모델 로드 실패
        → ros-jazzy-ros-gz 미설치 ((0) 단계 누락).
    controller "collision ahead!" 반복
        → nav2_params.yaml 의 obstacle_min_range 확인. 6번 섹션 FAQ 참고.


--------------------------------------------------------------------------------
 3. 사용법
--------------------------------------------------------------------------------

 [STEP A] 자율 매핑 -----------------------------------------------------------

  ros2 launch ur3_4040_urdf map_building.launch.py

    - Gazebo 시뮬레이터가 뜨고 robot이 (-3, -3, 0.15) 에 spawn
    - 14초간 자동 회전(부트스트랩) 후 explore_lite 가 자율 탐험 시작
    - SLAM Toolbox 가 /map 토픽으로 점진적으로 매핑 결과 발행
    - 한 번의 실행으로 warehouse_world 전체(30 x 24 m) 매핑 가능


 [STEP B] 매핑 결과 저장 -------------------------------------------------------

  cd ~/ur3/src/ur3_4040_urdf/maps
  ros2 run nav2_map_server map_saver_cli -f my_capstone_map \
    --ros-args -p map_subscribe_transient_local:=true

    - my_capstone_map.pgm  + my_capstone_map.yaml  생성
    - SLAM Toolbox 의 /map 토픽 QoS 호환을 위해 -p 옵션 필수


 [STEP C] 저장된 맵의 stale 점 제거 (라이다 self-hit 후처리) ------------------

  ros2 run ur3_4040_urdf clean_map.py \
    ~/ur3/src/ur3_4040_urdf/maps/my_capstone_map.pgm

    - opencv connected components 로 작은 occupied 군집(default <20px)을 free로
    - 큰 군집(진짜 벽)은 보존
    - 자동 .bak 백업 생성
    - 옵션: -t N (threshold), -o OUT (다른 파일로 저장), --no-backup


 [STEP D] 자율 주행 (RViz 사용) -----------------------------------------------

  ros2 launch ur3_4040_urdf navigate.launch.py

    - Gazebo + Nav2(map_server + AMCL + planner/controller/bt) + RViz 가 뜸
    - AMCL 은 set_initial_pose 로 자동 초기화 (사용자가 2D Pose Estimate 누를 필요 없음)
    - RViz 의 "2D Goal Pose" 버튼으로 클릭+드래그하여 미션 전달
       (클릭 = 위치, 드래그 방향 = 도착 yaw)


 [STEP E] 외부 토픽으로 미션 전달 (Headless) ----------------------------------

  ros2 launch ur3_4040_urdf mission.launch.py

    - Gazebo + Nav2 + goal_relay_node (RViz 없음)
    - 외부에서 /simple_goal (geometry_msgs/PoseStamped) 토픽 발행하면 자동 미션
    - 결과는 launch 터미널에 logger 로 출력 + /navigation_result(String) topic publish

  다른 터미널에서 goal 발행 예시 (map(5,2), yaw 90도):

    ros2 topic pub --once /simple_goal geometry_msgs/PoseStamped \
      "{header: {frame_id: 'map'},
        pose: {position: {x: 5.0, y: 2.0, z: 0.0},
               orientation: {x: 0, y: 0, z: 0.707, w: 0.707}}}"

  결과 모니터:

    ros2 topic echo /navigation_result
    (값: SUCCEEDED / ABORTED / CANCELED / REJECTED)


 [대안] CLI로 직접 nav2 action 호출 -------------------------------------------

  goal_relay_node 거치지 않고 nav2 action 을 직접 호출해도 됨:

    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'},
             pose: {position: {x: 5.0, y: 2.0, z: 0.0},
                    orientation: {x: 0, y: 0, z: 0.707, w: 0.707}}}}" \
    --feedback

    - --feedback 으로 거리/ETA/recovery 횟수 실시간 출력
    - 종료 시 SUCCEEDED/ABORTED 자동 표시


--------------------------------------------------------------------------------
 4. 패키지 구조
--------------------------------------------------------------------------------

  ur3_4040_urdf/
  |
  +-- launch/
  |   +-- gazebo.launch.py        : Gazebo + robot spawn + ros_gz_bridge
  |   +-- map_building.launch.py  : 자율 매핑 (SLAM + Nav2 + explore_lite + RViz)
  |   +-- navigate.launch.py      : 자율 주행 (저장된 맵 + AMCL + Nav2 + RViz)
  |   +-- mission.launch.py       : Headless 미션 (Nav2 + goal_relay_node)
  |   +-- nav2.launch.py          : Nav2 만 실행 (디버깅용)
  |   +-- display.launch.py       : robot_state_publisher + RViz (URDF 시각화)
  |
  +-- urdf/
  |   +-- full_robot.urdf.xacro   : robot 전체 조립 (mobile_base + ur3_arm)
  |   +-- mobile_base.xacro       : 모바일 베이스 link/joint
  |   +-- mobile_base_gazebo.xacro: 모바일 베이스 Gazebo plugin (lidar/imu/diff_drive)
  |   +-- ur3_arm.xacro           : UR3 매니퓰레이터
  |
  +-- config/
  |   +-- nav2_params.yaml        : Nav2 전체 파라미터 (AMCL, costmap, planner, RPP, ...)
  |   +-- ekf.yaml                : (미사용) robot_localization EKF 설정
  |   +-- joint_names_*.yaml      : (참고용) joint 이름 매핑
  |
  +-- scripts/
  |   +-- clean_map.py            : 저장된 pgm 의 stale obstacle 제거
  |   +-- goal_relay_node.py      : /simple_goal -> nav2 action 변환 노드
  |
  +-- maps/
  |   +-- my_capstone_map.pgm     : 매핑된 occupancy grid (이미지)
  |   +-- my_capstone_map.yaml    : 맵 메타데이터 (resolution, origin)
  |
  +-- meshes/                     : robot 외관 STL
  +-- textures/                   : Gazebo 머티리얼
  +-- docs/
  |   +-- autonomous_exploration_debug_report.md  : 디버깅 상세 보고서
  |   +-- paper_draft.md          : 논문 초안
  |
  +-- CMakeLists.txt
  +-- package.xml
  +-- read_me.txt                 : (이 파일)


--------------------------------------------------------------------------------
 5. 핵심 설정 요약 (config/nav2_params.yaml)
--------------------------------------------------------------------------------

  AMCL:
    set_initial_pose: true
    initial_pose: (0, 0, 0, 0)        # map frame 기준! (world 와 다름)
    tf_broadcast: true

  Costmap (local + global):
    footprint: [[0.20,0.27],[0.20,-0.27],[-0.20,-0.27],[-0.20,0.27]]
    inflation_radius: 0.25
    obstacle_min_range: 0.6           # chassis self-hit 차단
    raytrace_min_range: 0.6

  Planner:
    GridBased.use_astar: true
    GridBased.tolerance: 0.5
    GridBased.allow_unknown: true

  Controller (FollowPath):
    plugin: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
    desired_linear_vel: 0.35
    lookahead_dist: 0.4
    max_lookahead_dist: 0.6
    use_rotate_to_heading: true

  Goal checker:
    xy_goal_tolerance: 0.15
    yaw_goal_tolerance: 0.25          # navigate 시. 매핑 시에는 RewrittenYaml 로
                                      #  3.14 override (explore_lite frontier 도달 인정)


--------------------------------------------------------------------------------
 6. 자주 발생하는 문제와 해결
--------------------------------------------------------------------------------

  Q1. launch 재시작 했는데 robot 이 이상한 자세로 시작
       -> Gazebo 가 이전 세션 그대로 떠있는 상태. 다음으로 완전 종료:
          pkill -9 -f "gz sim|ros2 launch|nav2|slam|rviz|explore|parameter_bridge"
          sleep 3
          ros2 launch ur3_4040_urdf <launch>

  Q2. 매핑 후 navigate 시 controller 가 collision ahead! 만 외치고 못 감
       -> 라이다 self-hit 으로 live costmap 에 stale 점이 매번 박힘.
          obstacle_min_range / raytrace_min_range 가 0.6 인지 nav2_params.yaml 에서 확인.

  Q3. RViz 의 Map 디스플레이가 비어있음 (Width=0, Height=0)
       -> map_server lifecycle 활성화 race condition. localization_launch 와
          navigation_launch 가 시간차(5s, 12s)로 호출되도록 launch 가 구성됨.

  Q4. AMCL 위치/yaw 가 robot 실제와 어긋남
       -> ros_gz_bridge 의 use_sim_time 이 True 인지 확인. False 이면
          wall clock stamp 와 sim_time stamp 가 안 맞아 TF chain 보간 실패.

  Q5. goal 보냈는데 도착 후 yaw 가 반영 안 됨
       -> general_goal_checker.yaw_goal_tolerance 가 작은 값(0.25)인지 확인.
          매핑 시에는 RewrittenYaml 로 3.14 로 자동 override 됨.

  Q6. 매핑이 robot 주위 작은 영역만 되고 explore 가 일찍 종료됨
       -> explore_lite 의 return_to_init=false, min_frontier_size=0.1 확인.
          GridBased.tolerance=0.5, allow_unknown=true 도 필요.

  Q7. ros2 run ur3_4040_urdf clean_map.py 가 not found
       -> colcon build 후 source ~/ur3/install/setup.bash 다시 했는지 확인.


--------------------------------------------------------------------------------
 7. 실물 로봇으로 이전 시 변경점
--------------------------------------------------------------------------------

  - launch/gazebo.launch.py 제거 또는 미사용
  - 라즈베리 파이에서 실제 라이다 driver (rplidar_ros 등) 와 IMU driver(bno055) 실행
  - 모터 드라이버가 /odom 발행 + /cmd_vel 구독하도록 설정
  - 노트북-Pi 같은 ROS_DOMAIN_ID=28, 같은 LAN/WiFi 에서 자동 발견
  - AMCL set_initial_pose: false 로 두고 시작 시 정해진 도킹 위치에서 초기 pose publish
  - mission.launch.py 형태가 headless 운영에 그대로 활용 가능


--------------------------------------------------------------------------------
 8. 참고 문서
--------------------------------------------------------------------------------

  docs/autonomous_exploration_debug_report.md : 단계별 디버깅 결과
  docs/paper_draft.md                          : 논문 초안 (A4 절반 분량)


--------------------------------------------------------------------------------
 작성: 2026-05-03
--------------------------------------------------------------------------------
