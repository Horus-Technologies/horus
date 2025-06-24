#include "BoundedVolumeMap.hpp"

Sphere::Sphere(std::array<float,3>& center, float& radius)
:_center(center), _radius(radius){}

BoundedVolumeMap::BoundedVolumeMap(): _spheres{}{

}