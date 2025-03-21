#include "Voxel.hpp"

Voxel::Voxel()
    : _cost(0), _index({0,0,0}), _position({0, 0, 0}) {}

Voxel::Voxel(double cost)
    : _cost(cost), _index({0,0,0}), _position() 
    {
        double scale = 1;
        _position = {
            0.5*scale,
            0.5*scale,
            0.5*scale};
    }

Voxel::Voxel(std::array<int, 3> &index, double cost, double scale)
    : _index(index), _cost(cost) 
    {
        _position = {
            (index[0]+0.5)*scale,
            (index[1]+0.5)*scale,
            (index[2]+0.5)*scale};
    }

std::array<double, 3> Voxel::getPosition() const
{
    return _position;
}

std::array<int, 3> Voxel::getIndex() const
{
    return _index;
}

double Voxel::getCost() const
{
    return _cost;
}

void Voxel::setCost(double cost)
{
    _cost = cost;
}

bool Voxel::operator==(const Voxel& other) const {
    return _cost == other.getCost() && _position == other.getPosition() && _index == other.getIndex();
}

bool Voxel::operator!=(const Voxel& other) const {
    return !(*this == other);
}

namespace std {
    template <>
    struct hash<const Voxel*> {
        size_t operator()(const Voxel *voxel) const {
            if (voxel == nullptr) {
                return 0;
            }
            size_t h1 = std::hash<int>{}(voxel->getIndex()[0]);
            size_t h2 = std::hash<int>{}(voxel->getIndex()[1]);
            size_t h3 = std::hash<int>{}(voxel->getIndex()[2]);
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
}