#pragma once
#include <array>
#include <vector>
#include "core/json.hpp"
#include "core/particle.h"

nlohmann::json particle_to_json(const Particle& p);
nlohmann::json particle_poses_to_json(const std::vector<Particle>& p_vec);
