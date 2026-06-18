#!/bin/bash
# 사용법: bash scripts/run_waypoints.sh [right|center|left]
# 기본값: right

TARGET=${1:-right}

case $TARGET in
  right)
    JOINTS="[-0.6168230215655726, -2.0829302273192347, -2.4906070232391357, -0.6784708660892029, 0.4651188850402832, 0.6663665771484375]"
    ;;
  center)
    JOINTS="[-0.022162739430562794, -2.162271638909811, -2.4077365398406982, -0.9789505761912842, 0.5077733993530273, 0.8893018364906311]"
    ;;
  left)
    JOINTS="[0.6906053423881531, -1.4578069981983681, 2.349764887486593, -2.2809258900084437, -0.5541742483722132, -3.316235367451803]"
    ;;
  *)
    echo "사용법: bash scripts/run_waypoints.sh [right|center|left]"
    exit 1
    ;;
esac

echo "타겟: $TARGET"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=77

ros2 control switch_controllers \
    --activate scaled_joint_trajectory_controller 2>/dev/null || true

ros2 action send_goal \
  /scaled_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
      points: [
        {positions: [-1.724648300801412, -0.3419717115214844, -1.4476462602615356, -2.2719394169249476, 1.6401886940002441, -1.6181018988238733], time_from_start: {sec: 5,  nanosec: 0}},
        {positions: $JOINTS, time_from_start: {sec: 10, nanosec: 0}}
      ]
    }
  }"
