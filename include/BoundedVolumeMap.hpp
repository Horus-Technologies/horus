#include <vector>
#include <array>

struct Sphere{
    public:
    Sphere(std::array<float,3>& center, float& radius);

    private:
    std::array<float,3> _center;
    float _radius;
};

class BoundedVolumeMap{
    public:
    BoundedVolumeMap();

    private:
    std::vector<Sphere> _spheres;
};