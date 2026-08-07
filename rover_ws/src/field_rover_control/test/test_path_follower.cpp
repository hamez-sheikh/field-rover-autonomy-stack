// Unit tests for the pure field_rover_control path-following math. None of
// these tests touch ROS: they exercise path_follower.hpp/.cpp directly.
#include <cmath>
#include <limits>
#include <vector>

#include "field_rover_control/path_follower.hpp"
#include "gtest/gtest.h"

namespace field_rover_control
{
namespace
{

constexpr double kPi = M_PI;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

PathFollowerConfig make_default_config()
{
  return PathFollowerConfig{};
}

// ---------------------------------------------------------------------
// Configuration validation
// ---------------------------------------------------------------------

TEST(Config, DefaultsAreValid)
{
  EXPECT_TRUE(is_valid_config(make_default_config()));
}

TEST(Config, InvalidControlRate)
{
  auto config = make_default_config();
  config.control_rate_hz = 0.0;
  EXPECT_FALSE(is_valid_config(config));
  config.control_rate_hz = -5.0;
  EXPECT_FALSE(is_valid_config(config));
  config.control_rate_hz = kNan;
  EXPECT_FALSE(is_valid_config(config));
}

TEST(Config, InvalidLookaheadDistance)
{
  auto config = make_default_config();
  config.lookahead_distance_m = 0.0;
  EXPECT_FALSE(is_valid_config(config));
  config.lookahead_distance_m = -1.0;
  EXPECT_FALSE(is_valid_config(config));
}

TEST(Config, InvalidGoalTolerance)
{
  auto config = make_default_config();
  config.goal_tolerance_m = 0.0;
  EXPECT_FALSE(is_valid_config(config));
  config.goal_tolerance_m = kInf;
  EXPECT_FALSE(is_valid_config(config));
}

TEST(Config, InvalidSpeedLimits)
{
  auto config = make_default_config();
  config.max_linear_speed_mps = 0.0;
  EXPECT_FALSE(is_valid_config(config));

  config = make_default_config();
  config.max_angular_speed_radps = -0.1;
  EXPECT_FALSE(is_valid_config(config));
}

TEST(Config, InvalidGains)
{
  auto config = make_default_config();
  config.linear_gain = 0.0;
  EXPECT_FALSE(is_valid_config(config));

  config = make_default_config();
  config.angular_gain = -1.0;
  EXPECT_FALSE(is_valid_config(config));
}

TEST(Config, InvalidTurnThreshold)
{
  auto config = make_default_config();
  config.turn_in_place_threshold_rad = 0.0;
  EXPECT_FALSE(is_valid_config(config));
  config.turn_in_place_threshold_rad = kPi + 0.01;
  EXPECT_FALSE(is_valid_config(config));
  config.turn_in_place_threshold_rad = kPi;
  EXPECT_TRUE(is_valid_config(config));
}

TEST(Config, InvalidLocalizationTimeout)
{
  auto config = make_default_config();
  config.localization_timeout_s = 0.0;
  EXPECT_FALSE(is_valid_config(config));
  config.localization_timeout_s = -0.5;
  EXPECT_FALSE(is_valid_config(config));
}

TEST(Config, EmptyFrames)
{
  auto config = make_default_config();
  config.map_frame = "";
  EXPECT_FALSE(is_valid_config(config));

  config = make_default_config();
  config.base_frame = "";
  EXPECT_FALSE(is_valid_config(config));
}

// ---------------------------------------------------------------------
// Math helpers
// ---------------------------------------------------------------------

TEST(MathHelpers, NormalizeAngleZero)
{
  EXPECT_NEAR(normalize_angle(0.0), 0.0, 1e-9);
}

TEST(MathHelpers, NormalizeAnglePositiveWrapping)
{
  EXPECT_NEAR(normalize_angle(3.0 * kPi), -kPi, 1e-9);
  EXPECT_NEAR(normalize_angle(2.0 * kPi + 0.5), 0.5, 1e-9);
}

TEST(MathHelpers, NormalizeAngleNegativeWrapping)
{
  EXPECT_NEAR(normalize_angle(-3.0 * kPi), -kPi, 1e-9);
  EXPECT_NEAR(normalize_angle(-2.0 * kPi - 0.5), -0.5, 1e-9);
}

TEST(MathHelpers, NormalizeAngleNearPi)
{
  EXPECT_NEAR(normalize_angle(kPi - 1e-6), kPi - 1e-6, 1e-9);
  EXPECT_NEAR(normalize_angle(kPi), -kPi, 1e-9);
}

TEST(MathHelpers, NormalizeAngleNearNegativePi)
{
  EXPECT_NEAR(normalize_angle(-kPi + 1e-6), -kPi + 1e-6, 1e-9);
  EXPECT_NEAR(normalize_angle(-kPi), -kPi, 1e-9);
}

TEST(MathHelpers, CircularHeadingError)
{
  EXPECT_NEAR(circular_heading_error(0.1, 0.0), 0.1, 1e-9);
  EXPECT_NEAR(circular_heading_error(-kPi + 0.1, kPi - 0.1), 0.2, 1e-9);
  EXPECT_NEAR(circular_heading_error(0.0, kPi), -kPi, 1e-9);
}

TEST(MathHelpers, EuclideanDistance)
{
  EXPECT_NEAR(euclidean_distance(Point2D{0.0, 0.0}, Point2D{3.0, 4.0}), 5.0, 1e-9);
  EXPECT_NEAR(euclidean_distance(Point2D{1.0, 1.0}, Point2D{1.0, 1.0}), 0.0, 1e-9);
}

TEST(MathHelpers, FiniteValueChecks)
{
  EXPECT_TRUE(is_finite_value(1.0));
  EXPECT_FALSE(is_finite_value(kNan));
  EXPECT_FALSE(is_finite_value(kInf));
  EXPECT_TRUE(is_finite_point(Point2D{1.0, 2.0}));
  EXPECT_FALSE(is_finite_point(Point2D{kNan, 2.0}));
  EXPECT_TRUE(is_finite_pose(Pose2D{1.0, 2.0, 0.5}));
  EXPECT_FALSE(is_finite_pose(Pose2D{1.0, 2.0, kInf}));
}

TEST(MathHelpers, ClampBehaviour)
{
  EXPECT_NEAR(clamp_value(5.0, 0.0, 10.0), 5.0, 1e-9);
  EXPECT_NEAR(clamp_value(-5.0, 0.0, 10.0), 0.0, 1e-9);
  EXPECT_NEAR(clamp_value(15.0, 0.0, 10.0), 10.0, 1e-9);
}

// ---------------------------------------------------------------------
// Path validation
// ---------------------------------------------------------------------

TEST(PathValidation, EmptyPath)
{
  EXPECT_FALSE(is_path_valid({}));
}

TEST(PathValidation, OnePointPath)
{
  EXPECT_TRUE(is_path_valid({Point2D{1.0, 1.0}}));
}

TEST(PathValidation, MultiplePointPath)
{
  EXPECT_TRUE(is_path_valid({Point2D{0.0, 0.0}, Point2D{1.0, 0.0}, Point2D{2.0, 0.0}}));
}

TEST(PathValidation, NonFinitePoint)
{
  EXPECT_FALSE(is_path_valid({Point2D{0.0, 0.0}, Point2D{kNan, 1.0}}));
  EXPECT_FALSE(is_path_valid({Point2D{kInf, 0.0}}));
}

TEST(PathValidation, ReplacedPath)
{
  PathFollowerState state;
  reset_path(state, {Point2D{0.0, 0.0}, Point2D{5.0, 0.0}});
  state.current_path_index = 1;
  state.goal_reached = true;

  reset_path(state, {Point2D{9.0, 9.0}});

  EXPECT_TRUE(state.path_active);
  EXPECT_EQ(state.current_path_index, 0u);
  EXPECT_FALSE(state.goal_reached);
  ASSERT_EQ(state.path.size(), 1u);
  EXPECT_NEAR(state.path[0].x, 9.0, 1e-9);
}

TEST(PathValidation, ResetWithEmptyPathClears)
{
  PathFollowerState state;
  reset_path(state, {Point2D{0.0, 0.0}, Point2D{5.0, 0.0}});

  reset_path(state, {});

  EXPECT_FALSE(state.path_active);
  EXPECT_TRUE(state.path.empty());
}

// ---------------------------------------------------------------------
// Nearest point and progress
// ---------------------------------------------------------------------

class NearestPointTest : public ::testing::Test
{
protected:
  std::vector<Point2D> path = {
    Point2D{0.0, 0.0}, Point2D{1.0, 0.0}, Point2D{2.0, 0.0},
    Point2D{3.0, 0.0}, Point2D{4.0, 0.0},
  };
};

TEST_F(NearestPointTest, NearestFirstPoint)
{
  EXPECT_EQ(find_nearest_index(path, Point2D{0.1, 0.0}, 0), 0u);
}

TEST_F(NearestPointTest, NearestMiddlePoint)
{
  EXPECT_EQ(find_nearest_index(path, Point2D{2.05, 0.0}, 0), 2u);
}

TEST_F(NearestPointTest, NearestFinalPoint)
{
  EXPECT_EQ(find_nearest_index(path, Point2D{4.2, 0.0}, 0), 4u);
}

TEST_F(NearestPointTest, ProgressNeverDecreases)
{
  const std::size_t first = find_nearest_index(path, Point2D{2.0, 0.0}, 0);
  // Rover drifts slightly behind, but the search only looks forward from
  // the saved progress index, so the result cannot regress.
  const std::size_t second = find_nearest_index(path, Point2D{1.9, 0.0}, first);
  EXPECT_GE(second, first);
}

TEST_F(NearestPointTest, ProgressAdvancesAfterRoverMoves)
{
  const std::size_t first = find_nearest_index(path, Point2D{0.0, 0.0}, 0);
  const std::size_t second = find_nearest_index(path, Point2D{3.0, 0.0}, first);
  EXPECT_GT(second, first);
}

TEST(NearestPoint, DuplicatePathPoints)
{
  const std::vector<Point2D> path = {
    Point2D{1.0, 1.0}, Point2D{1.0, 1.0}, Point2D{2.0, 2.0},
  };
  EXPECT_EQ(find_nearest_index(path, Point2D{1.0, 1.0}, 0), 0u);
}

TEST(NearestPoint, RoverFarFromPath)
{
  const std::vector<Point2D> path = {Point2D{0.0, 0.0}, Point2D{10.0, 0.0}};
  EXPECT_EQ(find_nearest_index(path, Point2D{100.0, 100.0}, 0), 1u);
}

TEST(NearestPoint, StartIndexClampedToPathEnd)
{
  const std::vector<Point2D> path = {Point2D{0.0, 0.0}, Point2D{1.0, 0.0}};
  // A stale/out-of-range start_index must not read past the path.
  EXPECT_EQ(find_nearest_index(path, Point2D{0.0, 0.0}, 5), 1u);
}

// ---------------------------------------------------------------------
// Lookahead selection
// ---------------------------------------------------------------------

TEST(LookaheadSelection, FirstPointBeyondLookahead)
{
  const std::vector<Point2D> path = {
    Point2D{0.0, 0.0}, Point2D{0.5, 0.0}, Point2D{1.0, 0.0}, Point2D{2.0, 0.0},
  };
  EXPECT_EQ(select_lookahead_index(path, Point2D{0.0, 0.0}, 0, 0.9), 2u);
}

TEST(LookaheadSelection, FinalPointFallback)
{
  const std::vector<Point2D> path = {Point2D{0.0, 0.0}, Point2D{0.1, 0.0}, Point2D{0.2, 0.0}};
  EXPECT_EQ(select_lookahead_index(path, Point2D{0.0, 0.0}, 0, 5.0), 2u);
}

TEST(LookaheadSelection, OnePointPath)
{
  const std::vector<Point2D> path = {Point2D{3.0, 3.0}};
  EXPECT_EQ(select_lookahead_index(path, Point2D{0.0, 0.0}, 0, 0.6), 0u);
}

TEST(LookaheadSelection, ExactLookaheadBoundary)
{
  const std::vector<Point2D> path = {Point2D{0.0, 0.0}, Point2D{0.6, 0.0}, Point2D{2.0, 0.0}};
  EXPECT_EQ(select_lookahead_index(path, Point2D{0.0, 0.0}, 0, 0.6), 1u);
}

TEST(LookaheadSelection, NoOutOfRangeIndex)
{
  const std::vector<Point2D> path = {Point2D{0.0, 0.0}, Point2D{1.0, 0.0}};
  const std::size_t index = select_lookahead_index(path, Point2D{0.0, 0.0}, 0, 100.0);
  EXPECT_LT(index, path.size());
}

TEST(LookaheadSelection, ProgressIndexNearPathEnd)
{
  const std::vector<Point2D> path = {Point2D{0.0, 0.0}, Point2D{1.0, 0.0}, Point2D{2.0, 0.0}};
  EXPECT_EQ(select_lookahead_index(path, Point2D{2.0, 0.0}, 2, 0.6), 2u);
}

// ---------------------------------------------------------------------
// Heading control
// ---------------------------------------------------------------------

TEST(HeadingControl, TargetStraightAhead)
{
  auto config = make_default_config();
  const auto command = compute_control_command(
    Pose2D{0.0, 0.0, 0.0}, Point2D{1.0, 0.0}, config);
  EXPECT_NEAR(command.angular_z, 0.0, 1e-9);
}

TEST(HeadingControl, TargetLeft)
{
  auto config = make_default_config();
  const auto command = compute_control_command(
    Pose2D{0.0, 0.0, 0.0}, Point2D{0.0, 1.0}, config);
  EXPECT_GT(command.angular_z, 0.0);
}

TEST(HeadingControl, TargetRight)
{
  auto config = make_default_config();
  const auto command = compute_control_command(
    Pose2D{0.0, 0.0, 0.0}, Point2D{0.0, -1.0}, config);
  EXPECT_LT(command.angular_z, 0.0);
}

TEST(HeadingControl, TargetBehind)
{
  auto config = make_default_config();
  const auto command = compute_control_command(
    Pose2D{0.0, 0.0, 0.0}, Point2D{-1.0, 0.0}, config);
  EXPECT_NEAR(std::abs(command.angular_z), config.max_angular_speed_radps, 1e-9);
  EXPECT_NEAR(command.linear_x, 0.0, 1e-9);
}

TEST(HeadingControl, PositiveSaturation)
{
  auto config = make_default_config();
  config.angular_gain = 100.0;
  const auto command = compute_control_command(
    Pose2D{0.0, 0.0, 0.0}, Point2D{0.0, 1.0}, config);
  EXPECT_NEAR(command.angular_z, config.max_angular_speed_radps, 1e-9);
}

TEST(HeadingControl, NegativeSaturation)
{
  auto config = make_default_config();
  config.angular_gain = 100.0;
  const auto command = compute_control_command(
    Pose2D{0.0, 0.0, 0.0}, Point2D{0.0, -1.0}, config);
  EXPECT_NEAR(command.angular_z, -config.max_angular_speed_radps, 1e-9);
}

TEST(HeadingControl, SmallHeadingError)
{
  auto config = make_default_config();
  const auto command = compute_control_command(
    Pose2D{0.0, 0.0, 0.0}, Point2D{1.0, 0.05}, config);
  EXPECT_NEAR(command.angular_z, config.angular_gain * std::atan2(0.05, 1.0), 1e-6);
}

TEST(HeadingControl, WrappedHeadingError)
{
  auto config = make_default_config();
  // Rover heading sits just past the -pi boundary; the target is just
  // before the +pi boundary. The raw angle difference is nearly 2*pi, but
  // the true heading error should be small once wrapped into [-pi, pi).
  const double current_yaw = -kPi + 0.05;
  const double target_heading = kPi - 0.05;
  const Point2D target{std::cos(target_heading), std::sin(target_heading)};

  const auto command = compute_control_command(
    Pose2D{0.0, 0.0, current_yaw}, target, config);

  // A broken (unwrapped) error of ~2*pi would saturate the angular command
  // at exactly max_angular_speed_radps; a correctly wrapped ~-0.1 rad
  // error stays well under that.
  EXPECT_LT(std::abs(command.angular_z), config.max_angular_speed_radps - 0.1);
}

// ---------------------------------------------------------------------
// Linear control
// ---------------------------------------------------------------------

TEST(LinearControl, FullSpeedForStraightTarget)
{
  auto config = make_default_config();
  config.linear_gain = 10.0;  // Force gain*distance to exceed the cap.
  const auto command = compute_control_command(
    Pose2D{0.0, 0.0, 0.0}, Point2D{1.0, 0.0}, config);
  EXPECT_NEAR(command.linear_x, config.max_linear_speed_mps, 1e-9);
}

TEST(LinearControl, ReducedSpeedForModerateHeadingError)
{
  auto config = make_default_config();
  config.linear_gain = 10.0;
  const auto straight = compute_control_command(
    Pose2D{0.0, 0.0, 0.0}, Point2D{1.0, 0.0}, config);
  const auto angled = compute_control_command(
    Pose2D{0.0, 0.0, 0.0}, Point2D{1.0, 0.6}, config);
  EXPECT_LT(angled.linear_x, straight.linear_x);
}

TEST(LinearControl, ZeroSpeedAboveTurnThreshold)
{
  auto config = make_default_config();
  config.turn_in_place_threshold_rad = 0.5;
  const auto command = compute_control_command(
    Pose2D{0.0, 0.0, 0.0}, Point2D{0.0, 1.0}, config);  // 90 deg error.
  EXPECT_NEAR(command.linear_x, 0.0, 1e-9);
}

TEST(LinearControl, SpeedLimitedByMax)
{
  auto config = make_default_config();
  const auto command = compute_control_command(
    Pose2D{0.0, 0.0, 0.0}, Point2D{1000.0, 0.0}, config);
  EXPECT_LE(command.linear_x, config.max_linear_speed_mps + 1e-9);
}

TEST(LinearControl, SpeedReducedNearTarget)
{
  auto config = make_default_config();
  const auto near = compute_control_command(
    Pose2D{0.0, 0.0, 0.0}, Point2D{0.05, 0.0}, config);
  const auto far = compute_control_command(
    Pose2D{0.0, 0.0, 0.0}, Point2D{5.0, 0.0}, config);
  EXPECT_LT(near.linear_x, far.linear_x);
}

TEST(LinearControl, NoReverseCommand)
{
  auto config = make_default_config();
  const auto command = compute_control_command(
    Pose2D{0.0, 0.0, 0.0}, Point2D{-1.0, 0.0}, config);
  EXPECT_GE(command.linear_x, 0.0);
}

// ---------------------------------------------------------------------
// Goal completion
// ---------------------------------------------------------------------

TEST(GoalCompletion, OutsideToleranceRemainsActive)
{
  auto config = make_default_config();
  PathFollowerState state;
  reset_path(state, {Point2D{0.0, 0.0}, Point2D{5.0, 0.0}});

  const auto result = step(state, Pose2D{0.0, 0.0, 0.0}, config);

  EXPECT_FALSE(result.goal_reached);
  EXPECT_FALSE(state.goal_reached);
}

TEST(GoalCompletion, ExactToleranceStops)
{
  auto config = make_default_config();
  PathFollowerState state;
  reset_path(state, {Point2D{0.0, 0.0}, Point2D{5.0, 0.0}});

  const auto result = step(
    state, Pose2D{5.0 - config.goal_tolerance_m, 0.0, 0.0}, config);

  EXPECT_TRUE(result.goal_reached);
}

TEST(GoalCompletion, InsideToleranceStops)
{
  auto config = make_default_config();
  PathFollowerState state;
  reset_path(state, {Point2D{0.0, 0.0}, Point2D{5.0, 0.0}});

  const auto result = step(state, Pose2D{4.99, 0.0, 0.0}, config);

  EXPECT_TRUE(result.goal_reached);
  EXPECT_NEAR(result.command.linear_x, 0.0, 1e-9);
  EXPECT_NEAR(result.command.angular_z, 0.0, 1e-9);
}

TEST(GoalCompletion, OnePointGoalCompletion)
{
  auto config = make_default_config();
  PathFollowerState state;
  reset_path(state, {Point2D{0.05, 0.0}});

  const auto result = step(state, Pose2D{0.0, 0.0, 0.0}, config);

  EXPECT_TRUE(result.goal_reached);
}

TEST(GoalCompletion, FinalPointReachedAfterMultiPointPath)
{
  auto config = make_default_config();
  PathFollowerState state;
  reset_path(
    state, {Point2D{0.0, 0.0}, Point2D{1.0, 0.0}, Point2D{2.0, 0.0}, Point2D{3.0, 0.0}});

  step(state, Pose2D{0.0, 0.0, 0.0}, config);
  step(state, Pose2D{1.5, 0.0, 0.0}, config);
  const auto result = step(state, Pose2D{2.95, 0.0, 0.0}, config);

  EXPECT_TRUE(result.goal_reached);
}

TEST(GoalCompletion, CompletedPathRemainsStopped)
{
  auto config = make_default_config();
  PathFollowerState state;
  reset_path(state, {Point2D{0.0, 0.0}, Point2D{1.0, 0.0}});

  step(state, Pose2D{0.95, 0.0, 0.0}, config);
  ASSERT_TRUE(state.goal_reached);

  // A further call, even from a different pose, must keep reporting
  // completion instead of resuming tracking.
  const auto result = step(state, Pose2D{0.0, 0.0, 0.0}, config);
  EXPECT_TRUE(result.goal_reached);
  EXPECT_NEAR(result.command.linear_x, 0.0, 1e-9);
}

// ---------------------------------------------------------------------
// Safe stops
// ---------------------------------------------------------------------

TEST(SafeStop, NoPath)
{
  auto config = make_default_config();
  PathFollowerState state;
  const auto result = step(state, Pose2D{0.0, 0.0, 0.0}, config);
  EXPECT_FALSE(result.valid);
  EXPECT_NEAR(result.command.linear_x, 0.0, 1e-9);
  EXPECT_NEAR(result.command.angular_z, 0.0, 1e-9);
}

TEST(SafeStop, InvalidPathViaResetClears)
{
  auto config = make_default_config();
  PathFollowerState state;
  reset_path(state, {Point2D{kNan, 0.0}});
  EXPECT_FALSE(state.path_active);

  const auto result = step(state, Pose2D{0.0, 0.0, 0.0}, config);
  EXPECT_FALSE(result.valid);
}

TEST(SafeStop, InvalidPose)
{
  auto config = make_default_config();
  PathFollowerState state;
  reset_path(state, {Point2D{0.0, 0.0}, Point2D{5.0, 0.0}});

  const auto result = step(state, Pose2D{kNan, 0.0, 0.0}, config);
  EXPECT_FALSE(result.valid);
  EXPECT_NEAR(result.command.linear_x, 0.0, 1e-9);
  EXPECT_NEAR(result.command.angular_z, 0.0, 1e-9);
}

TEST(SafeStop, GoalReachedStop)
{
  auto config = make_default_config();
  PathFollowerState state;
  reset_path(state, {Point2D{0.0, 0.0}});

  const auto result = step(state, Pose2D{0.0, 0.0, 0.0}, config);
  EXPECT_TRUE(result.goal_reached);
  EXPECT_NEAR(result.command.linear_x, 0.0, 1e-9);
}

TEST(SafeStop, EmptyReplacementPath)
{
  PathFollowerState state;
  reset_path(state, {Point2D{0.0, 0.0}, Point2D{1.0, 0.0}});
  ASSERT_TRUE(state.path_active);

  reset_path(state, {});

  EXPECT_FALSE(state.path_active);
  EXPECT_TRUE(state.path.empty());
  EXPECT_FALSE(state.goal_reached);
}

// ---------------------------------------------------------------------
// Command validity
// ---------------------------------------------------------------------

TEST(CommandValidity, AllOutputsFinite)
{
  auto config = make_default_config();
  PathFollowerState state;
  reset_path(state, {Point2D{0.0, 0.0}, Point2D{5.0, 3.0}});

  const auto result = step(state, Pose2D{1.0, 0.5, 0.3}, config);

  EXPECT_TRUE(is_finite_value(result.command.linear_x));
  EXPECT_TRUE(is_finite_value(result.command.angular_z));
}

TEST(CommandValidity, LinearLimitRespected)
{
  auto config = make_default_config();
  config.linear_gain = 50.0;
  PathFollowerState state;
  reset_path(state, {Point2D{0.0, 0.0}, Point2D{50.0, 0.0}});

  const auto result = step(state, Pose2D{0.0, 0.0, 0.0}, config);

  EXPECT_GE(result.command.linear_x, 0.0);
  EXPECT_LE(result.command.linear_x, config.max_linear_speed_mps + 1e-9);
}

TEST(CommandValidity, AngularLimitRespected)
{
  auto config = make_default_config();
  config.angular_gain = 50.0;
  PathFollowerState state;
  reset_path(state, {Point2D{0.0, 0.0}, Point2D{0.0, 5.0}});

  const auto result = step(state, Pose2D{0.0, 0.0, 0.0}, config);

  EXPECT_LE(std::abs(result.command.angular_z), config.max_angular_speed_radps + 1e-9);
}

TEST(CommandValidity, DeterministicRepeatedInput)
{
  auto config = make_default_config();
  PathFollowerState state_a;
  PathFollowerState state_b;
  reset_path(state_a, {Point2D{0.0, 0.0}, Point2D{3.0, 2.0}});
  reset_path(state_b, {Point2D{0.0, 0.0}, Point2D{3.0, 2.0}});

  const auto result_a = step(state_a, Pose2D{0.5, 0.2, 0.1}, config);
  const auto result_b = step(state_b, Pose2D{0.5, 0.2, 0.1}, config);

  EXPECT_NEAR(result_a.command.linear_x, result_b.command.linear_x, 1e-12);
  EXPECT_NEAR(result_a.command.angular_z, result_b.command.angular_z, 1e-12);
}

}  // namespace
}  // namespace field_rover_control

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
