from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    # planning_scene_monitor 명시 활성화 (2026-06-17): 기본 move_group 이 /planning_scene 토픽 diff 를
    #   적용 안 하고 /apply_planning_scene 서비스도 없어 collision object(본체 바닥 장애물)가 무효였음.
    #   → 모니터 publish/apply 켜서 장애물이 scene 에 반영되게(접근 MoveGroup 이 본체 프레임 피하도록).
    moveit_config = (
        MoveItConfigsBuilder("ur3_control", package_name="ur3_moveit_config_custom")
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
