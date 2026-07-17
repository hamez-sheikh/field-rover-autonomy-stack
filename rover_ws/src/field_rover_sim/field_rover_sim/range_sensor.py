"""Pure Python ray-casting geometry for ideal directional range sensing."""

from dataclasses import dataclass
import math

from field_rover_sim.world_model import CircleObstacle, Pose2D, WorldModel


ZERO_DIVISION_TOLERANCE = 1e-9


@dataclass(frozen=True)
class BeamDefinition:
    """Describe one fixed directional beam relative to the rover body."""

    name: str
    relative_angle: float


@dataclass(frozen=True)
class RangeSensorConfig:
    """Store the configured beam layout and sensing limits."""

    beams: tuple[BeamDefinition, ...]
    min_range: float
    max_range: float
    field_of_view: float

    def __post_init__(self):
        """Reject a configuration that cannot produce valid readings."""
        if not self.beams:
            raise ValueError('beams must contain at least one entry.')
        if self.min_range < 0.0:
            raise ValueError('min_range must be non-negative.')
        if self.max_range <= self.min_range:
            raise ValueError('max_range must be greater than min_range.')
        if self.field_of_view <= 0.0:
            raise ValueError('field_of_view must be positive.')


DEFAULT_RANGE_SENSOR_CONFIG = RangeSensorConfig(
    beams=(
        BeamDefinition(name='front_far_right', relative_angle=-math.pi / 3.0),
        BeamDefinition(name='front_right', relative_angle=-math.pi / 6.0),
        BeamDefinition(name='front', relative_angle=0.0),
        BeamDefinition(name='front_left', relative_angle=math.pi / 6.0),
        BeamDefinition(name='front_far_left', relative_angle=math.pi / 3.0),
    ),
    min_range=0.1,
    max_range=8.0,
    field_of_view=0.05,
)


def calculate_beam_world_angle(rover_yaw: float, relative_angle: float) -> float:
    """Rotate a beam's body-relative angle into the world map frame."""
    return rover_yaw + relative_angle


def calculate_beam_direction(beam_world_angle: float) -> tuple[float, float]:
    """Return the unit direction vector for a beam's world-frame angle."""
    return math.cos(beam_world_angle), math.sin(beam_world_angle)


def intersect_ray_with_circle(
    origin_x: float,
    origin_y: float,
    direction_x: float,
    direction_y: float,
    obstacle: CircleObstacle,
) -> float | None:
    """Return the nearest non-negative distance to a circular obstacle."""
    # Ray-circle quadratic: for m = origin - centre and unit direction d,
    # the roots t^2 + 2*b*t + c_term = 0 give the crossing distances,
    # with b = dot(m, d) and c_term = dot(m, m) - radius^2.
    m_x = origin_x - obstacle.x
    m_y = origin_y - obstacle.y

    b = m_x * direction_x + m_y * direction_y
    c_term = (m_x * m_x + m_y * m_y) - obstacle.radius * obstacle.radius
    discriminant = b * b - c_term

    if discriminant < 0.0:
        return None

    sqrt_discriminant = math.sqrt(discriminant)
    near_t = -b - sqrt_discriminant
    far_t = -b + sqrt_discriminant

    if near_t >= 0.0:
        return near_t
    if far_t >= 0.0:
        return far_t
    return None


def intersect_ray_with_boundary(
    origin_x: float,
    origin_y: float,
    direction_x: float,
    direction_y: float,
    world: WorldModel,
) -> float | None:
    """Return the nearest non-negative distance to a rectangular wall."""
    # Checks the four walls x=0, x=width, y=0, y=height, keeping only
    # forward intersections (t >= 0) whose other coordinate falls on the
    # finite wall segment. A ray parallel to a wall pair skips that pair
    # instead of dividing by a near-zero direction component.
    candidate_distances = []

    if abs(direction_x) > ZERO_DIVISION_TOLERANCE:
        for wall_x in (0.0, world.width):
            t = (wall_x - origin_x) / direction_x
            if t < 0.0:
                continue
            hit_y = origin_y + t * direction_y
            if 0.0 <= hit_y <= world.height:
                candidate_distances.append(t)

    if abs(direction_y) > ZERO_DIVISION_TOLERANCE:
        for wall_y in (0.0, world.height):
            t = (wall_y - origin_y) / direction_y
            if t < 0.0:
                continue
            hit_x = origin_x + t * direction_x
            if 0.0 <= hit_x <= world.width:
                candidate_distances.append(t)

    if not candidate_distances:
        return None
    return min(candidate_distances)


def measure_beam_range(
    pose: Pose2D,
    relative_angle: float,
    world: WorldModel,
    min_range: float,
    max_range: float,
) -> float:
    """Cast one beam from the rover centre and return its clamped range."""
    # A hit closer than min_range saturates to min_range (sensor
    # saturation), and no hit within max_range reports max_range as the
    # no-detection value, so downstream code never sees infinity.
    beam_world_angle = calculate_beam_world_angle(pose.yaw, relative_angle)
    direction_x, direction_y = calculate_beam_direction(beam_world_angle)

    candidate_distances = []

    for obstacle in world.obstacles:
        distance = intersect_ray_with_circle(
            pose.x, pose.y, direction_x, direction_y, obstacle,
        )
        if distance is not None:
            candidate_distances.append(distance)

    boundary_distance = intersect_ray_with_boundary(
        pose.x, pose.y, direction_x, direction_y, world,
    )
    if boundary_distance is not None:
        candidate_distances.append(boundary_distance)

    valid_distances = [d for d in candidate_distances if d <= max_range]

    if not valid_distances:
        return max_range

    nearest_distance = min(valid_distances)
    return max(nearest_distance, min_range)


def measure_all_beams(
    pose: Pose2D,
    world: WorldModel,
    config: RangeSensorConfig,
) -> tuple[tuple[str, float], ...]:
    """Return (beam_name, range_metres) for every configured beam, in order."""
    return tuple(
        (
            beam.name,
            measure_beam_range(
                pose,
                beam.relative_angle,
                world,
                config.min_range,
                config.max_range,
            ),
        )
        for beam in config.beams
    )
