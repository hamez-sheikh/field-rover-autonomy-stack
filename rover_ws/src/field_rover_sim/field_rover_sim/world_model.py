"""Pure Python data model for the static two-dimensional rover world."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    """Represent a ground-truth position and heading in the map frame."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class CircleObstacle:
    """Represent a named circular obstacle in the world."""

    name: str
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class WorldModel:
    """Describe the world boundaries, rover footprint, and obstacles."""

    width: float
    height: float
    rover_radius: float
    obstacles: tuple[CircleObstacle, ...]


DEFAULT_WORLD = WorldModel(
    width=20.0,
    height=15.0,
    rover_radius=0.4,
    obstacles=(
        CircleObstacle(name='rock_alpha', x=6.0, y=4.0, radius=1.0),
        CircleObstacle(name='rock_beta', x=12.0, y=9.0, radius=1.5),
        CircleObstacle(name='rock_gamma', x=16.0, y=5.0, radius=0.8),
    ),
)

INITIAL_ROVER_POSE = Pose2D(x=2.0, y=2.0, yaw=0.0)


def collides_with_obstacle(
    pose: Pose2D,
    rover_radius: float,
    obstacle: CircleObstacle,
) -> bool:
    """Return whether the rover overlaps or touches one obstacle."""
    centre_distance = math.hypot(pose.x - obstacle.x, pose.y - obstacle.y)
    return centre_distance <= rover_radius + obstacle.radius


def collides_with_boundary(pose: Pose2D, world: WorldModel) -> bool:
    """Return whether the rover footprint reaches or crosses a world wall."""
    return (
        pose.x - world.rover_radius <= 0.0
        or pose.y - world.rover_radius <= 0.0
        or pose.x + world.rover_radius >= world.width
        or pose.y + world.rover_radius >= world.height
    )


def is_pose_in_collision(pose: Pose2D, world: WorldModel) -> bool:
    """Return whether the rover collides with any obstacle or boundary."""
    if collides_with_boundary(pose, world):
        return True

    return any(
        collides_with_obstacle(pose, world.rover_radius, obstacle)
        for obstacle in world.obstacles
    )
