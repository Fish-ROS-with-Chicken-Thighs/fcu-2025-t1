#ifndef TARGET_HPP
#define TARGET_HPP

// 目标点类
class target {
public:
    float x, y, z, yaw;
    bool reached;

    target(float x, float y, float z, float yaw) : reached(false), x(x), y(y), z(z), yaw(yaw) {}
};
#endif
