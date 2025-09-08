#pragma once

#include "geometry.hpp"
#include "triangulation.hpp"
#include <span>

namespace geometry::visualization {

void Draw(std::span<geometry::Shape> shapes);

void Draw(std::span<const geometry::Triangle> triangles);

}  // namespace geometry::visualization
