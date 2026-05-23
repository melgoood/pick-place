#include "robot_controller.h"

#include <mujoco/mujoco.h>
#include <iostream>

static double clampValue(double v, double lo, double hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static double smooth(double x)
{
    x = clampValue(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

static double lerp(double a, double b, double k)
{
    return a + (b - a) * k;
}

RobotController::RobotController(mjModel* model)
{
    m = model;

    a_yaw = mj_name2id(m, mjOBJ_ACTUATOR, "a_yaw");
    a_shoulder = mj_name2id(m, mjOBJ_ACTUATOR, "a_shoulder");
    a_comp = mj_name2id(m, mjOBJ_ACTUATOR, "a_comp");
    a_finger_left = mj_name2id(m, mjOBJ_ACTUATOR, "a_finger_left");
    a_finger_right = mj_name2id(m, mjOBJ_ACTUATOR, "a_finger_right");

    if (a_yaw < 0 || a_shoulder < 0 || a_comp < 0 ||
        a_finger_left < 0 || a_finger_right < 0)
    {
        std::cout << "Ошибка: не найдены actuator-ы." << std::endl;
        valid = false;
        return;
    }

    valid = true;
    std::cout << "Контроллер запущен. Физический захват без телепортации." << std::endl;
}

void RobotController::setPose(mjData* data, double yaw, double shoulder, double comp)
{
    if (!valid) return;

    data->ctrl[a_yaw] = clampValue(yaw, -0.1, 1.7);
    data->ctrl[a_shoulder] = clampValue(shoulder, -0.75, 0.15);
    data->ctrl[a_comp] = clampValue(comp, 0.0, 0.75);
}

void RobotController::setGripper(mjData* data, bool closed)
{
    if (!valid) return;

    if (closed)
    {
        data->ctrl[a_finger_left] = 0.009;
        data->ctrl[a_finger_right] = 0.009;
    }
    else
    {
        data->ctrl[a_finger_left] = 0.0;
        data->ctrl[a_finger_right] = 0.0;
    }
}

void RobotController::update(mjData* data)
{
    if (!valid) return;

    const double t = data->time;

    const double yaw_cube = 1.570796;
    const double yaw_box = 0.0;

    const double shoulder_up = 0.0;
    const double comp_up = 0.0;

    const double shoulder_down_cube = -0.285;
    const double comp_down_cube = 0.285;

    const double shoulder_down_box = -0.16;
    const double comp_down_box = 0.16;

    if (t < 1.2)
    {
        double k = smooth(t / 1.2);
        setPose(data, lerp(0.0, yaw_cube, k), shoulder_up, comp_up);
        setGripper(data, false);
    }
    else if (t < 2.8)
    {
        double k = smooth((t - 1.2) / 1.6);
        setPose(data, yaw_cube,
            lerp(shoulder_up, shoulder_down_cube, k),
            lerp(comp_up, comp_down_cube, k));
        setGripper(data, false);
    }
    else if (t < 3.7)
    {
        setPose(data, yaw_cube, shoulder_down_cube, comp_down_cube);
        setGripper(data, true);
    }
    else if (t < 5.4)
    {
        double k = smooth((t - 3.7) / 1.7);
        setPose(data, yaw_cube,
            lerp(shoulder_down_cube, shoulder_up, k),
            lerp(comp_down_cube, comp_up, k));
        setGripper(data, true);
    }
    else if (t < 7.3)
    {
        double k = smooth((t - 5.4) / 1.9);
        setPose(data, lerp(yaw_cube, yaw_box, k), shoulder_up, comp_up);
        setGripper(data, true);
    }
    else if (t < 8.7)
    {
        double k = smooth((t - 7.3) / 1.4);
        setPose(data, yaw_box,
            lerp(shoulder_up, shoulder_down_box, k),
            lerp(comp_up, comp_down_box, k));
        setGripper(data, true);
    }
    else if (t < 9.4)
    {
        setPose(data, yaw_box, shoulder_down_box, comp_down_box);
        setGripper(data, false);
    }
    else if (t < 10.8)
    {
        double k = smooth((t - 9.4) / 1.4);
        setPose(data, yaw_box,
            lerp(shoulder_down_box, shoulder_up, k),
            lerp(comp_down_box, comp_up, k));
        setGripper(data, false);
    }
    else
    {
        double k = smooth((t - 10.8) / 1.8);
        setPose(data, lerp(yaw_box, 0.7, k), shoulder_up, comp_up);
        setGripper(data, false);
    }
}