#pragma once
#include <array>
#include <vector>
#include "core/json.hpp"
#include "core/particle.h"

nlohmann::json particle_to_json(const Particle& p);
