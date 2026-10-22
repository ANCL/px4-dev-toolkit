// Load and validate SE(3) runtime settings and vehicle parameters.

#include <offboard_controllers/se3/runtime/node.hpp>
#include <trajectory_generator/yaml.hpp>
#include <yaml-cpp/yaml.h>

namespace offboard_controllers {

namespace {
// Validate all geometric gains before any ROS timer or controller callback
// starts. They must be finite and componentwise non-negative.
void require_nonnegative_gain(const se3::Vector3& value, const char* name) {
  if (!math::is_finite(value) || value.x < 0.0 || value.y < 0.0 || value.z < 0.0) {
    throw std::invalid_argument(std::string("SE3 ") + name +
                                " must be finite and non-negative.");
  }
}

double load_vehicle_mass(const std::string& config_directory, const std::string& vehicle) {
  const std::string path = config_directory + "/" + vehicle + ".yaml";

  YAML::Node root;

  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& error) {
    throw std::invalid_argument("Unable to load vehicle configuration '" + path +
                                "': " + error.what());
  }

  const YAML::Node mass_node = root["plant"]["mass_kg"];

  if (!mass_node) {
    throw std::invalid_argument("Vehicle configuration '" + path +
                                "' does not define plant.mass_kg.");
  }

  const double mass = mass_node.as<double>();

  if (!std::isfinite(mass) || mass <= 0.0) {
    throw std::invalid_argument("Vehicle plant.mass_kg must be finite and positive.");
  }

  return mass;
}

// This is the configured PX4 normalized hover-thrust coefficient,
// not the physical mass or an actuator thrust constant. Only paths
// using PX4-normalized collective output require this value.
double load_vehicle_hover_thrust(const std::string& config_directory, const std::string& vehicle) {
  const std::string path = config_directory + "/" + vehicle + ".yaml";

  YAML::Node root;

  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& error) {
    throw std::invalid_argument("Unable to load vehicle configuration '" + path +
                                "': " + error.what());
  }

  const YAML::Node hover_thrust_node = root["px4_allocator"]["hover_thrust"];

  if (!hover_thrust_node) {
    throw std::invalid_argument("Vehicle configuration '" + path +
                                "' does not define px4_allocator.hover_thrust; "
                                "attitude, attitude_rate, and thrust_and_torque handoffs "
                                "require a validated hover-thrust value.");
  }

  const double hover_thrust = hover_thrust_node.as<double>();

  if (!std::isfinite(hover_thrust) || hover_thrust <= 0.0) {
    throw std::invalid_argument("Vehicle px4_allocator.hover_thrust must be "
                                "finite and positive.");
  }

  return hover_thrust;
}

se3::Vector3 vector3_parameter(const std::vector<double>& values, const std::string& name) {
  if (values.size() != 3) {
    throw std::invalid_argument("SE3 " + name + " must contain exactly three values.");
  }

  const se3::Vector3 result{
      values[0],
      values[1],
      values[2],
  };

  if (!math::is_finite(result)) {
    throw std::invalid_argument("SE3 " + name + " must contain finite values.");
  }

  return result;
}
} // namespace

// Parameter names and defaults are part of the ROS launch contract.
// Load controller configuration before creating publishers or timers
// so invalid vehicle/trajectory settings fail during startup.
Se3::Se3() : Node("se3") {
  vehicle_ = declare_parameter<std::string>("vehicle", "");

  trajectory_name_ = declare_parameter<std::string>("trajectory", "");

  handoff_ = declare_parameter<std::string>("handoff", "");

  direct_controller_ = declare_parameter<std::string>("direct_controller", "geometric_normalized");
  offboard_trigger_ = parse_offboard_trigger(
      declare_parameter<std::string>("offboard_trigger", "automatic"));

  publish_diagnostics_ = declare_parameter<bool>("publish_diagnostics", false);

  const std::string vehicle_config_dir = declare_parameter<std::string>("vehicle_config_dir", "");

  const std::string trajectory_config = declare_parameter<std::string>("trajectory_config", "");

  const double kx_over_mass =
      declare_parameter<double>("kx_over_mass", std::numeric_limits<double>::quiet_NaN());

  const double kv_over_mass =
      declare_parameter<double>("kv_over_mass", std::numeric_limits<double>::quiet_NaN());

  const se3::Vector3 attitude_gain = vector3_parameter(
      declare_parameter<std::vector<double>>("attitude_gain", std::vector<double>{}),
      "attitude_gain");

  const se3::Vector3 normalized_attitude_gain = vector3_parameter(
      declare_parameter<std::vector<double>>("normalized_attitude_gain", std::vector<double>{}),
      "normalized_attitude_gain");

  const se3::Vector3 normalized_angular_velocity_gain =
      vector3_parameter(declare_parameter<std::vector<double>>("normalized_angular_velocity_gain",
                                                               std::vector<double>{}),
                        "normalized_angular_velocity_gain");

  const se3::Vector3 normalized_angular_acceleration_gain =
      vector3_parameter(declare_parameter<std::vector<double>>(
                            "normalized_angular_acceleration_gain", std::vector<double>{}),
                        "normalized_angular_acceleration_gain");

  const se3::Vector3 geometric_physical_k_r = vector3_parameter(
      declare_parameter<std::vector<double>>("geometric_physical_k_r", std::vector<double>{}),
      "geometric_physical_k_r");

  const se3::Vector3 geometric_physical_k_omega = vector3_parameter(
      declare_parameter<std::vector<double>>("geometric_physical_k_omega", std::vector<double>{}),
      "geometric_physical_k_omega");

  const se3::Vector3 px4_rate_p = vector3_parameter(
      declare_parameter<std::vector<double>>("px4_rate_p", std::vector<double>{}), "px4_rate_p");

  const se3::Vector3 px4_rate_i = vector3_parameter(
      declare_parameter<std::vector<double>>("px4_rate_i", std::vector<double>{}), "px4_rate_i");

  const se3::Vector3 px4_rate_d = vector3_parameter(
      declare_parameter<std::vector<double>>("px4_rate_d", std::vector<double>{}), "px4_rate_d");

  const se3::Vector3 px4_rate_ff = vector3_parameter(
      declare_parameter<std::vector<double>>("px4_rate_ff", std::vector<double>{}), "px4_rate_ff");

  const se3::Vector3 px4_rate_k = vector3_parameter(
      declare_parameter<std::vector<double>>("px4_rate_k", std::vector<double>{}), "px4_rate_k");

  const se3::Vector3 px4_rate_integrator_limit = vector3_parameter(
      declare_parameter<std::vector<double>>("px4_rate_integrator_limit", std::vector<double>{}),
      "px4_rate_integrator_limit");

  const double px4_rate_yaw_torque_cutoff_hz = declare_parameter<double>(
      "px4_rate_yaw_torque_cutoff_hz", std::numeric_limits<double>::quiet_NaN());

  const bool px4_rate_battery_scale_enabled =
      declare_parameter<bool>("px4_rate_battery_scale_enabled", false);

  const double control_rate_hz =
      declare_parameter<double>("control_rate_hz", std::numeric_limits<double>::quiet_NaN());

  const double warmup_duration_s =
      declare_parameter<double>("warmup_duration_s", std::numeric_limits<double>::quiet_NaN());

  const double offboard_retry_interval_s = declare_parameter<double>(
      "offboard_retry_interval_s", std::numeric_limits<double>::quiet_NaN());

  runtime_timing_ =
      make_runtime_timing(control_rate_hz, warmup_duration_s, offboard_retry_interval_s);

  if (vehicle_.empty()) {
    throw std::invalid_argument("SE3 vehicle parameter must be specified.");
  }

  if (trajectory_name_.empty()) {
    throw std::invalid_argument("SE3 trajectory parameter must be specified.");
  }

  if (vehicle_config_dir.empty()) {
    throw std::invalid_argument("SE3 vehicle_config_dir parameter must be specified.");
  }

  if (trajectory_config.empty()) {
    throw std::invalid_argument("SE3 trajectory_config parameter must be specified.");
  }

  if (handoff_ != "acceleration" && handoff_ != "attitude" && handoff_ != "attitude_rate" &&
      handoff_ != "thrust_and_torque") {
    throw std::invalid_argument("SE3 handoff '" + handoff_ +
                                "' is not implemented. Available handoffs: "
                                "acceleration, attitude, attitude_rate, thrust_and_torque.");
  }

  if (handoff_ == "thrust_and_torque" && direct_controller_ != "geometric_normalized" &&
      direct_controller_ != "px4_rate" && direct_controller_ != "geometric_physical") {
    throw std::invalid_argument("SE3 direct_controller '" + direct_controller_ +
                                "' is not implemented. Available direct controllers: "
                                "geometric_normalized, px4_rate, geometric_physical.");
  }

  if (!std::isfinite(kx_over_mass) || kx_over_mass <= 0.0) {
    throw std::invalid_argument("SE3 kx_over_mass must be finite and positive.");
  }

  if (!std::isfinite(kv_over_mass) || kv_over_mass <= 0.0) {
    throw std::invalid_argument("SE3 kv_over_mass must be finite and positive.");
  }

  if (geometric_physical_k_r.x <= 0.0 || geometric_physical_k_r.y <= 0.0 ||
      geometric_physical_k_r.z <= 0.0 || geometric_physical_k_omega.x <= 0.0 ||
      geometric_physical_k_omega.y <= 0.0 || geometric_physical_k_omega.z <= 0.0) {
    throw std::invalid_argument("SE3 Geometric physical rotational gains must be positive.");
  }

  const double mass = load_vehicle_mass(vehicle_config_dir, vehicle_);

  mass_ = mass;

  // The PX4-normalized attitude/rate/direct-normalized paths use hover
  // thrust as the physical-force -> normalized-collective scale. The
  // geometric_physical path instead uses the vehicle propulsion curve and
  // therefore does not depend on MPC_THR_HOVER.
  const bool requires_px4_hover_thrust =
      handoff_ == "attitude" || handoff_ == "attitude_rate" ||
      (handoff_ == "thrust_and_torque" && direct_controller_ != "geometric_physical");

  if (requires_px4_hover_thrust) {
    hover_thrust_ = load_vehicle_hover_thrust(vehicle_config_dir, vehicle_);
  }

  configured_trajectory_ = std::make_unique<trajectory::ConfiguredTrajectory>(
      trajectory::load_trajectory(trajectory_config, trajectory_name_));

  // Validate physical gains after conversion from mass-normalized
  // ROS parameters; finite inputs can still overflow on multiplication.
  kx_ = mass * kx_over_mass;
  kv_ = mass * kv_over_mass;
  if (!std::isfinite(kx_) || kx_ <= 0.0 || !std::isfinite(kv_) || kv_ <= 0.0) {
    throw std::invalid_argument("SE3 physical translation gains must be finite and positive.");
  }

  require_nonnegative_gain(attitude_gain, "attitude_gain");
  require_nonnegative_gain(normalized_attitude_gain, "normalized_attitude_gain");
  require_nonnegative_gain(normalized_angular_velocity_gain,
                           "normalized_angular_velocity_gain");
  require_nonnegative_gain(normalized_angular_acceleration_gain,
                           "normalized_angular_acceleration_gain");
  attitude_gain_ = attitude_gain;
  normalized_attitude_gain_ = normalized_attitude_gain;
  normalized_angular_velocity_gain_ = normalized_angular_velocity_gain;
  normalized_angular_acceleration_gain_ = normalized_angular_acceleration_gain;

  if (handoff_ == "thrust_and_torque" && direct_controller_ == "geometric_physical") {
    physical_wrench_configuration_ =
        vehicle_config::load_physical_wrench_configuration(vehicle_config_dir, vehicle_);

    geometric_physical_k_r_ = geometric_physical_k_r;

    geometric_physical_k_omega_ = geometric_physical_k_omega;
  }

  if (handoff_ == "thrust_and_torque" && direct_controller_ == "px4_rate") {
    if (px4_rate_battery_scale_enabled) {
      throw std::invalid_argument("SE3 px4_rate currently requires "
                                  "px4_rate_battery_scale_enabled=false; "
                                  "the verified PX4 baseline has MC_BAT_SCALE_EN=0.");
    }

    px4_rate_controller_ = std::make_unique<px4_rate::Controller>(px4_rate::Parameters{
        px4_rate_p,
        px4_rate_i,
        px4_rate_d,
        px4_rate_ff,
        px4_rate_k,
        px4_rate_integrator_limit,
        px4_rate_yaw_torque_cutoff_hz,
    });
  }

  initialize_publishers();
  initialize_subscriptions();

  RCLCPP_INFO(get_logger(),
              "Fixed PX4 local NED start: [%.3f, %.3f, %.3f], yaw=%.3f rad",
              configured_trajectory_->start_position[0],
              configured_trajectory_->start_position[1],
              configured_trajectory_->start_position[2], configured_trajectory_->yaw);

  RCLCPP_INFO(get_logger(),
              "SE3 ready: vehicle=%s, trajectory=%s, "
              "handoff=%s, mass=%.6f kg, "
              "kx/m=%.6f, kv/m=%.6f",
              vehicle_.c_str(), trajectory_name_.c_str(), handoff_.c_str(), mass, kx_over_mass,
              kv_over_mass);

  if (handoff_ == "thrust_and_torque") {
    RCLCPP_INFO(get_logger(), "SE3 direct controller: %s", direct_controller_.c_str());
  }
}

} // namespace offboard_controllers
