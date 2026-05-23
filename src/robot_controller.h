#pragma once

#include <mujoco/mujoco.h>

class RobotController
{
public:
    RobotController(mjModel* model);
    void update(mjData* data);

private:
    mjModel* m = nullptr;

    int a_yaw = -1;
    int a_shoulder = -1;
    int a_comp = -1;
    int a_finger_left = -1;
    int a_finger_right = -1;

    bool valid = false;

    void setPose(mjData* data, double yaw, double shoulder, double comp);
    void setGripper(mjData* data, bool closed);
};