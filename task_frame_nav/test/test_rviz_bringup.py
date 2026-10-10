import math
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description import LaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RVIZ_CONFIG = PACKAGE_ROOT / 'config' / 'task_frame_nav.rviz'
NAV2_PARAMS = PACKAGE_ROOT / 'config' / 'nav2_task_frame.yaml'
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'task_frame_nav.launch.py'


def _collect_displays(displays):
    for display in displays:
        yield display
        yield from _collect_displays(display.get('Displays', []))


def _launch_default(argument):
    value = argument.default_value
    if isinstance(value, list) and len(value) == 1:
        value = value[0]
    return value.perform(None) if hasattr(value, 'perform') else value


def test_rviz_config_shows_navigation_data_in_the_task_frame():
    config = yaml.safe_load(RVIZ_CONFIG.read_text())
    manager = config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'task_frame'

    displays = list(_collect_displays(manager['Displays']))
    topics = {
        display.get('Topic', {}).get('Value')
        for display in displays
    }
    assert {
        '/Laser_map',
        '/cloud_registered_body',
        '/global_costmap/costmap',
        '/local_costmap/costmap',
        '/plan',
        '/local_plan',
        '/local_costmap/published_footprint',
    } <= topics

    tool_classes = {tool['Class'] for tool in manager['Tools']}
    assert 'nav2_rviz_plugins/GoalTool' not in tool_classes
    assert 'rviz_default_plugins/SetGoal' not in tool_classes


def test_nav2_projects_clouds_in_the_captured_base_plane():
    params = yaml.safe_load(NAV2_PARAMS.read_text())
    assert params['bt_navigator']['ros__parameters']['global_frame'] == 'task_frame'
    assert params['local_costmap']['local_costmap']['ros__parameters'][
        'global_frame'] == 'task_frame'
    assert params['global_costmap']['global_costmap']['ros__parameters'][
        'global_frame'] == 'task_frame'
    assert params['behavior_server']['ros__parameters']['global_frame'] == 'task_frame'
    assert params['bt_navigator']['ros__parameters']['robot_base_frame'] == 'base_link'
    assert params['local_costmap']['local_costmap']['ros__parameters'][
        'robot_base_frame'] == 'base_link'
    assert params['global_costmap']['global_costmap']['ros__parameters'][
        'robot_base_frame'] == 'base_link'
    assert params['behavior_server']['ros__parameters']['robot_base_frame'] == 'base_link'
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_map = params['global_costmap']['global_costmap']['ros__parameters']
    assert local['plugins'] == ['obstacle_layer', 'inflation_layer']
    for layer, topic in (
        (local['obstacle_layer'], '/cloud_registered_body'),
        (global_map['obstacle_layer'], '/Laser_map'),
    ):
        assert layer['plugin'] == 'nav2_costmap_2d::ObstacleLayer'
        assert layer['min_obstacle_height'] == 0.10
        assert layer['max_obstacle_height'] == 1.2
        source = layer['cloud'] if topic == '/cloud_registered_body' else layer['map_cloud']
        assert source['topic'] == topic
        assert source['min_obstacle_height'] == 0.10
        assert source['max_obstacle_height'] == 1.2


def test_nav2_does_not_accept_a_large_planar_goal_error():
    params = yaml.safe_load(NAV2_PARAMS.read_text())
    controller = params['controller_server']['ros__parameters']
    goal_checker = controller['general_goal_checker']
    follow_path = controller['FollowPath']

    assert goal_checker['xy_goal_tolerance'] <= 0.05
    assert goal_checker['stateful'] is False
    assert follow_path['xy_goal_tolerance'] <= 0.05
    assert follow_path['trans_stopped_velocity'] <= 0.05
    assert follow_path['min_vel_y'] == 0.0
    assert follow_path['max_vel_y'] == 0.0
    assert follow_path['vy_samples'] == 1
    smoother = params['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'][1] == 0.0
    assert smoother['min_velocity'][1] == 0.0


def test_combined_launch_passes_task_frame_to_task_node():
    spec = spec_from_file_location('task_frame_nav_launch', LAUNCH_FILE)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)

    description = module.generate_launch_description()
    task_nodes = [
        entity for entity in description.entities
        if isinstance(entity, Node)
        and entity.node_package == 'airobot_action'
        and entity.node_executable == 'task_frame_nav_node'
    ]
    assert len(task_nodes) == 1
    leveling_nodes = [
        entity for entity in description.entities
        if isinstance(entity, Node)
        and entity.node_executable == 'static_transform_publisher'
    ]
    assert len(leveling_nodes) == 1
    assert leveling_nodes[0]._Node__arguments[-1] == 'base_link'
    robot_include = next(
        entity for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
        and 'base_frame' in dict(entity.launch_arguments)
    )
    assert dict(robot_include.launch_arguments)['base_frame'] == 'base_link'
    assert 'robot_type' in dict(robot_include.launch_arguments)
    assert 'invert_angular_z' in dict(robot_include.launch_arguments)
    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert _launch_default(arguments['task_frame']) == 'task_frame'
    assert _launch_default(arguments['robot_type']) == 'r20_fwd'
    assert _launch_default(arguments['invert_angular_z']) == 'false'
    pitch = float(_launch_default(arguments['body_to_base_pitch']))
    x = float(_launch_default(arguments['body_to_base_x']))
    z = float(_launch_default(arguments['body_to_base_z']))
    assert math.isclose(pitch, -math.radians(46.55), abs_tol=1e-12)
    assert math.isclose(x, 0.20 * math.sin(-pitch), abs_tol=1e-8)
    assert math.isclose(z, -0.20 * math.cos(-pitch), abs_tol=1e-8)


def test_combined_launch_rewrites_nav2_global_frames_with_task_frame_override():
    spec = spec_from_file_location('task_frame_nav_launch', LAUNCH_FILE)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)

    description = module.generate_launch_description()
    nav2_include = next(
        entity for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
        and dict(entity.launch_arguments).get('container_name') == 'nav2_container'
    )
    rewritten_params = dict(nav2_include.launch_arguments)['params_file']
    assert isinstance(rewritten_params, RewrittenYaml)

    context = LaunchContext()
    context.launch_configurations.update({
        'nav2_params': str(NAV2_PARAMS),
        'task_frame': 'captured_base',
    })
    rewritten_path = Path(rewritten_params.perform(context))
    params = yaml.safe_load(rewritten_path.read_text())

    assert params['bt_navigator']['ros__parameters']['global_frame'] == 'captured_base'
    assert params['local_costmap']['local_costmap']['ros__parameters'][
        'global_frame'] == 'captured_base'
    assert params['global_costmap']['global_costmap']['ros__parameters'][
        'global_frame'] == 'captured_base'
    assert params['behavior_server']['ros__parameters']['global_frame'] == 'captured_base'


def test_combined_launch_rewrites_rviz_fixed_frame_with_task_frame_override():
    spec = spec_from_file_location('task_frame_nav_launch', LAUNCH_FILE)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)

    description = module.generate_launch_description()
    rviz_node = next(
        entity for entity in description.entities
        if isinstance(entity, Node) and entity.node_package == 'rviz2'
    )
    rewritten_config = rviz_node._Node__arguments[1]
    assert isinstance(rewritten_config, RewrittenYaml)

    context = LaunchContext()
    context.launch_configurations.update({
        'rviz_config': str(RVIZ_CONFIG),
        'task_frame': 'captured_base',
    })
    config = yaml.safe_load(Path(rewritten_config.perform(context)).read_text())
    assert config['Visualization Manager']['Global Options']['Fixed Frame'] == 'captured_base'


def test_combined_launch_starts_rviz_by_default_and_allows_disabling_it():
    spec = spec_from_file_location('task_frame_nav_launch', LAUNCH_FILE)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)

    description = module.generate_launch_description()
    assert isinstance(description, LaunchDescription)
    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert _launch_default(arguments['use_rviz']) == 'true'
    assert _launch_default(arguments['rviz_config']).endswith(
        'task_frame_nav/config/task_frame_nav.rviz')

    rviz_nodes = [
        entity for entity in description.entities
        if isinstance(entity, Node) and entity.node_package == 'rviz2'
    ]
    assert len(rviz_nodes) == 1
    assert rviz_nodes[0].node_executable == 'rviz2'
    assert isinstance(rviz_nodes[0].condition, IfCondition)

    context = LaunchContext()
    context.launch_configurations['use_rviz'] = 'false'
    assert not rviz_nodes[0].condition.evaluate(context)
    context.launch_configurations['use_rviz'] = 'true'
    assert rviz_nodes[0].condition.evaluate(context)


def test_nav2_composition_flag_is_a_python_boolean_for_nav2_expressions():
    spec = spec_from_file_location('task_frame_nav_launch', LAUNCH_FILE)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)

    description = module.generate_launch_description()
    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    nav2_include = next(
        entity for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
        and dict(entity.launch_arguments).get('container_name') ==
        'nav2_container'
    )
    forwarded_composition = dict(nav2_include.launch_arguments)[
        'use_composition']

    values = {
        _launch_default(arguments['use_composition']): 'False',
        'false': 'False',
        'False': 'False',
        'true': 'True',
        'True': 'True',
    }
    for value, expected in values.items():
        context = LaunchContext()
        context.launch_configurations['use_composition'] = value
        actual = perform_substitutions(context, [forwarded_composition])
        assert actual == expected
