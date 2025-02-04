#ifndef VELOCITY_HPP
#define VELOCITY_HPP

// 速度类
class velocity {
public:
    float vx, vy, vz; // 速度
    float dy, dp, dr; // 角速度(d=derivative求时间微分，yaw,pitch,roll)

    velocity(float vx, float vy, float vz, float dy=0, float dp=0, float dr=0) : 
            vx(vx), vy(vy), vz(vz), dy(dy), dp(dp), dr(dr) {}
};
#endif
