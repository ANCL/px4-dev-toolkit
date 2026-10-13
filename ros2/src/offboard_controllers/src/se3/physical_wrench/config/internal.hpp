// Internal declarations for physical-wrench vehicle YAML parsing.

#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::vehicle_config::detail
{

enum class BodyFrame
{
  FRD,
  FLU,
};

std::string vehicle_path(const std::string & config_directory, const std::string & vehicle);
YAML::Node load_vehicle_root(const std::string & path);
double finite_scalar(const YAML::Node & node, const std::string & name);
int integer_value(const YAML::Node & node, const std::string & name);
std::string string_value(const YAML::Node & node, const std::string & name);
BodyFrame parse_body_frame(const YAML::Node & node, const std::string & name);
se3::Vector3 sequence_vector3(const YAML::Node & node, const std::string & name);
se3::Vector3 allocator_position(const YAML::Node & node, const std::string & name);
se3::Vector3 named_vector3(const YAML::Node & node, const std::string & name);
se3::Vector3 to_frd(const se3::Vector3 & vector, BodyFrame frame);
se3::InertiaMatrix inertia_to_frd(const se3::InertiaMatrix & inertia, BodyFrame frame);
void validate_inertia(const se3::InertiaMatrix & inertia);
int direction_sign(const YAML::Node & node, const std::string & name);

}  // namespace offboard_controllers::vehicle_config::detail
