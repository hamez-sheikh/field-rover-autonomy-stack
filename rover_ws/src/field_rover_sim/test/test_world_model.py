"""Unit tests for the static two-dimensional world model."""

from field_rover_sim.world_model import (
    CircleObstacle,
    collides_with_boundary,
    collides_with_obstacle,
    DEFAULT_WORLD,
    INITIAL_ROVER_POSE,
    is_pose_in_collision,
    Pose2D,
)
import pytest


def test_initial_pose_is_in_free_space():
    """Confirm the configured initial rover pose is collision-free."""
    assert not is_pose_in_collision(INITIAL_ROVER_POSE, DEFAULT_WORLD)


def test_pose_overlapping_obstacle_is_in_collision():
    """Confirm a rover overlapping an obstacle is in collision."""
    pose = Pose2D(x=6.0, y=4.0, yaw=0.0)

    assert is_pose_in_collision(pose, DEFAULT_WORLD)


def test_touching_obstacle_counts_as_collision():
    """Confirm exact contact between circular footprints is a collision."""
    obstacle = CircleObstacle(name='test_rock', x=5.0, y=5.0, radius=0.6)
    touching_pose = Pose2D(x=4.0, y=5.0, yaw=0.0)

    assert collides_with_obstacle(
        touching_pose,
        rover_radius=0.4,
        obstacle=obstacle,
    )


@pytest.mark.parametrize(
    'pose',
    [
        Pose2D(x=0.3, y=7.5, yaw=0.0),
        Pose2D(x=10.0, y=0.3, yaw=0.0),
        Pose2D(x=19.7, y=7.5, yaw=0.0),
        Pose2D(x=10.0, y=14.7, yaw=0.0),
    ],
    ids=['left', 'bottom', 'right', 'top'],
)
def test_crossing_each_world_boundary_is_a_collision(pose):
    """Confirm crossing any wall with the rover footprint is a collision."""
    assert collides_with_boundary(pose, DEFAULT_WORLD)


def test_pose_near_wall_without_touching_is_valid():
    """Confirm a rover near a wall remains valid when a gap still exists."""
    pose = Pose2D(x=0.4001, y=7.5, yaw=0.0)

    assert not collides_with_boundary(pose, DEFAULT_WORLD)
    assert not is_pose_in_collision(pose, DEFAULT_WORLD)
