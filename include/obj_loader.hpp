#pragma once

#include <filesystem>
#include <memory>
#include <string>

#include "math.hpp"

namespace raytracer
{
std::shared_ptr<Mesh> load_obj_file(const std::filesystem::path &file_path);
}  // namespace raytracer
