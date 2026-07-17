"""Unit tests for the ideal directional range-sensing geometry."""

import math

from field_rover_sim.range_sensor import (
    calculate_beam_direction,
    calculate_beam_world_angle,
    DEFAULT_RANGE_SENSOR_CONFIG,
    intersect_ray_with_boundary,
    intersect_ray_with_circle,
    measure_all_beams,
    measure_beam_range,
    RangeSensorConfig,
)
from field_rover_sim.world_model import CircleObstacle, Pose2D, WorldModel

import pytest


SMALL_WORLD = WorldModel(
    width=10.0,
    height=10.0,
    rover_radius=0.4,
    obstacles=(
        CircleObstacle(name='test_rock', x=5.0, y=5.0, radius=1.0),
    ),
)


def test_front_beam_at_zero_yaw_points_along_positive_x():
    """Confirm the front beam direction is (1, 0) when yaw and angle are zero."""
    world_angle = calculate_beam_world_angle(rover_yaw=0.0, relative_angle=0.0)
    direction_x, direction_y = calculate_beam_direction(world_angle)

    assert direction_x == pytest.approx(1.0)
    assert direction_y == pytest.approx(0.0, abs=1e-9)


def test_positive_relative_angle_rotates_counter_clockwise():
    """Confirm a positive relative angle points toward the rover's left."""
    world_angle = calculate_beam_world_angle(
        rover_yaw=0.0, relative_angle=math.pi / 2.0,
    )
    direction_x, direction_y = calculate_beam_direction(world_angle)

    assert direction_x == pytest.approx(0.0, abs=1e-9)
    assert direction_y == pytest.approx(1.0)


def test_rover_yaw_rotates_all_beams():
    """Confirm a quarter-turn yaw carries a zero-angle beam to face +y."""
    world_angle = calculate_beam_world_angle(
        rover_yaw=math.pi / 2.0, relative_angle=0.0,
    )
    direction_x, direction_y = calculate_beam_direction(world_angle)

    assert direction_x == pytest.approx(0.0, abs=1e-9)
    assert direction_y == pytest.approx(1.0)


def test_direct_forward_ray_hits_circle_at_surface_distance():
    """Confirm the reported distance stops at the circle surface, not centre."""
    obstacle = CircleObstacle(name='rock', x=5.0, y=0.0, radius=1.0)

    distance = intersect_ray_with_circle(0.0, 0.0, 1.0, 0.0, obstacle)

    assert distance == pytest.approx(4.0)


def test_ray_missing_circle_returns_none():
    """Confirm a ray that passes well clear of a circle reports no hit."""
    obstacle = CircleObstacle(name='rock', x=5.0, y=5.0, radius=1.0)

    distance = intersect_ray_with_circle(0.0, 0.0, 1.0, 0.0, obstacle)

    assert distance is None


def test_circle_behind_ray_is_ignored():
    """Confirm a circle located behind the ray origin is not reported."""
    obstacle = CircleObstacle(name='rock', x=-5.0, y=0.0, radius=1.0)

    distance = intersect_ray_with_circle(0.0, 0.0, 1.0, 0.0, obstacle)

    assert distance is None


def test_tangent_ray_returns_expected_distance():
    """Confirm a tangent ray still returns the single expected distance."""
    obstacle = CircleObstacle(name='rock', x=5.0, y=1.0, radius=1.0)

    distance = intersect_ray_with_circle(0.0, 0.0, 1.0, 0.0, obstacle)

    assert distance == pytest.approx(5.0)


def test_ray_starting_inside_circle_returns_forward_exit_distance():
    """Confirm a ray cast from inside a circle returns the forward exit hit."""
    obstacle = CircleObstacle(name='rock', x=0.0, y=0.0, radius=2.0)

    distance = intersect_ray_with_circle(0.0, 0.0, 1.0, 0.0, obstacle)

    assert distance == pytest.approx(2.0)


def test_nearest_of_two_obstacles_is_selected():
    """Confirm the closer of two colinear obstacles determines the range."""
    world = WorldModel(
        width=20.0,
        height=20.0,
        rover_radius=0.4,
        obstacles=(
            CircleObstacle(name='near_rock', x=13.0, y=10.0, radius=0.5),
            CircleObstacle(name='far_rock', x=18.0, y=10.0, radius=0.5),
        ),
    )
    pose = Pose2D(x=10.0, y=10.0, yaw=0.0)

    distance = measure_beam_range(
        pose, 0.0, world, min_range=0.1, max_range=8.0,
    )

    assert distance == pytest.approx(2.5)


def test_obstacle_beyond_max_range_reports_no_detection_value():
    """Confirm an obstacle further than max_range yields the max_range reading."""
    world = WorldModel(
        width=50.0,
        height=50.0,
        rover_radius=0.4,
        obstacles=(
            CircleObstacle(name='distant_rock', x=30.0, y=10.0, radius=0.5),
        ),
    )
    pose = Pose2D(x=10.0, y=10.0, yaw=0.0)

    distance = measure_beam_range(
        pose, 0.0, world, min_range=0.1, max_range=8.0,
    )

    assert distance == pytest.approx(8.0)


def test_valid_hit_exactly_at_max_range_is_reported():
    """Confirm a hit exactly at max_range is treated as a valid detection."""
    world = WorldModel(
        width=50.0,
        height=50.0,
        rover_radius=0.4,
        obstacles=(
            CircleObstacle(name='edge_rock', x=18.0, y=10.0, radius=0.5),
        ),
    )
    pose = Pose2D(x=10.0, y=10.0, yaw=0.0)

    distance = measure_beam_range(
        pose, 0.0, world, min_range=0.1, max_range=7.5,
    )

    assert distance == pytest.approx(7.5)


def test_hit_below_min_range_saturates_to_min_range():
    """Confirm a hit closer than min_range clamps up to the min_range value."""
    world = WorldModel(
        width=50.0,
        height=50.0,
        rover_radius=0.4,
        obstacles=(
            CircleObstacle(name='close_rock', x=0.05, y=0.0, radius=0.02),
        ),
    )
    pose = Pose2D(x=0.0, y=0.0, yaw=0.0)

    distance = measure_beam_range(
        pose, 0.0, world, min_range=0.1, max_range=8.0,
    )

    assert distance == pytest.approx(0.1)


def test_direct_right_wall_hit():
    """Confirm a ray aimed at +x hits the right wall at the expected distance."""
    distance = intersect_ray_with_boundary(5.0, 5.0, 1.0, 0.0, SMALL_WORLD)

    assert distance == pytest.approx(5.0)


def test_direct_left_wall_hit():
    """Confirm a ray aimed at -x hits the left wall at the expected distance."""
    distance = intersect_ray_with_boundary(5.0, 5.0, -1.0, 0.0, SMALL_WORLD)

    assert distance == pytest.approx(5.0)


def test_direct_top_wall_hit():
    """Confirm a ray aimed at +y hits the top wall at the expected distance."""
    distance = intersect_ray_with_boundary(5.0, 5.0, 0.0, 1.0, SMALL_WORLD)

    assert distance == pytest.approx(5.0)


def test_direct_bottom_wall_hit():
    """Confirm a ray aimed at -y hits the bottom wall at the expected distance."""
    distance = intersect_ray_with_boundary(5.0, 5.0, 0.0, -1.0, SMALL_WORLD)

    assert distance == pytest.approx(5.0)


def test_diagonal_wall_hit_reaches_nearest_corner_wall():
    """Confirm a 45-degree ray reaches whichever wall is geometrically nearer."""
    world = WorldModel(width=10.0, height=6.0, rover_radius=0.4, obstacles=())
    direction = (math.sqrt(2.0) / 2.0, math.sqrt(2.0) / 2.0)

    distance = intersect_ray_with_boundary(
        2.0, 2.0, direction[0], direction[1], world,
    )

    assert distance == pytest.approx(math.hypot(4.0, 4.0))


def test_horizontal_ray_does_not_raise_division_by_zero():
    """Confirm a purely horizontal ray is handled without a math error."""
    distance = intersect_ray_with_boundary(5.0, 5.0, 1.0, 0.0, SMALL_WORLD)

    assert distance is not None


def test_vertical_ray_does_not_raise_division_by_zero():
    """Confirm a purely vertical ray is handled without a math error."""
    distance = intersect_ray_with_boundary(5.0, 5.0, 0.0, 1.0, SMALL_WORLD)

    assert distance is not None


def test_nearest_wall_is_selected_when_multiple_are_reachable():
    """Confirm the closer wall wins when a diagonal ray could reach two walls."""
    world = WorldModel(width=8.0, height=10.0, rover_radius=0.4, obstacles=())
    direction = (math.sqrt(2.0) / 2.0, math.sqrt(2.0) / 2.0)

    distance = intersect_ray_with_boundary(
        4.0, 5.0, direction[0], direction[1], world,
    )

    assert distance == pytest.approx(4.0 * math.sqrt(2.0))


def test_obstacle_nearer_than_wall_is_selected():
    """Confirm the beam reports the obstacle when it is closer than any wall."""
    world = WorldModel(
        width=20.0,
        height=20.0,
        rover_radius=0.4,
        obstacles=(
            CircleObstacle(name='near_rock', x=13.0, y=10.0, radius=0.5),
        ),
    )
    pose = Pose2D(x=10.0, y=10.0, yaw=0.0)

    distance = measure_beam_range(
        pose, 0.0, world, min_range=0.1, max_range=8.0,
    )

    assert distance == pytest.approx(2.5)


def test_wall_nearer_than_obstacle_is_selected():
    """Confirm the beam reports the wall when it is closer than any obstacle."""
    world = WorldModel(
        width=5.0,
        height=20.0,
        rover_radius=0.4,
        obstacles=(
            CircleObstacle(name='far_rock', x=8.0, y=10.0, radius=0.05),
        ),
    )
    pose = Pose2D(x=0.5, y=10.0, yaw=0.0)

    distance = measure_beam_range(
        pose, 0.0, world, min_range=0.1, max_range=8.0,
    )

    assert distance == pytest.approx(4.5)


def test_five_beam_configuration_produces_five_measurements():
    """Confirm the default configuration yields exactly five readings."""
    pose = Pose2D(x=5.0, y=5.0, yaw=0.0)

    readings = measure_all_beams(pose, SMALL_WORLD, DEFAULT_RANGE_SENSOR_CONFIG)

    assert len(readings) == 5


def test_five_beam_result_ordering_matches_declared_beam_order():
    """Confirm reading order matches the configured beam declaration order."""
    pose = Pose2D(x=5.0, y=5.0, yaw=0.0)

    readings = measure_all_beams(pose, SMALL_WORLD, DEFAULT_RANGE_SENSOR_CONFIG)
    names = [name for name, _ in readings]

    assert names == [
        'front_far_right',
        'front_right',
        'front',
        'front_left',
        'front_far_left',
    ]


def test_measurement_values_never_exceed_max_range():
    """Confirm no beam ever reports a distance greater than max_range."""
    pose = Pose2D(x=5.0, y=5.0, yaw=0.7)

    readings = measure_all_beams(pose, SMALL_WORLD, DEFAULT_RANGE_SENSOR_CONFIG)

    for _, distance in readings:
        assert distance <= DEFAULT_RANGE_SENSOR_CONFIG.max_range


def test_measurement_values_never_fall_below_min_range():
    """Confirm no beam ever reports a distance below the configured min_range."""
    tight_config = RangeSensorConfig(
        beams=DEFAULT_RANGE_SENSOR_CONFIG.beams,
        min_range=0.5,
        max_range=8.0,
        field_of_view=0.05,
    )
    world_with_close_rock = WorldModel(
        width=10.0,
        height=10.0,
        rover_radius=0.4,
        obstacles=(
            CircleObstacle(name='close_rock', x=5.2, y=5.0, radius=0.05),
        ),
    )
    pose = Pose2D(x=5.0, y=5.0, yaw=0.0)

    readings = measure_all_beams(pose, world_with_close_rock, tight_config)

    for _, distance in readings:
        assert distance >= tight_config.min_range
