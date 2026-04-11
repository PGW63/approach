#pragma once

#include <string>

#include "approach_planning/planner.hpp"

namespace approach_planning
{

PlannerConfig loadPlannerConfigFromYaml(const std::string & yaml_path);

}  // namespace approach_planning
