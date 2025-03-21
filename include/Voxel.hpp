#include <array>
#include <cstddef>
#include <functional>

class Voxel
{
public:
    Voxel();
    Voxel(double cost);
    Voxel(std::array<int, 3> &index, double cost, double scale);

    std::array<double, 3> getPosition() const;
    std::array<int, 3> getIndex() const;
    double getCost() const;
    void setCost(double cost);

    bool operator==(const Voxel& other) const;
    bool operator!=(const Voxel& other) const;

    

private:
    // Position (center of voxel)
    std::array<double, 3> _position;

    // Index
    std::array<int, 3> _index;

    // Cost
    double _cost;
};