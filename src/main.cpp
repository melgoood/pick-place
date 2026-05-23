#include <windows.h>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <iostream>

#include "robot_controller.h"

static mjModel* m = nullptr;
static mjData* d = nullptr;
static mjvCamera cam;
static mjvOption opt;
static mjvScene scn;
static mjrContext con;
static GLFWwindow* window = nullptr;

static void keyboard(GLFWwindow*, int key, int, int act, int)
{
    if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
    {
        glfwSetWindowShouldClose(window, 1);
    }
}

static bool button_left = false;
static bool button_middle = false;
static bool button_right = false;
static double lastx = 0;
static double lasty = 0;

static void mouse_button(GLFWwindow* window, int button, int act, int)
{
    button_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    button_middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    button_right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

    glfwGetCursorPos(window, &lastx, &lasty);
}

static void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    if (!button_left && !button_middle && !button_right) return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    mjtMouse action;
    if (button_right)
        action = mjMOUSE_MOVE_H;
    else if (button_left)
        action = mjMOUSE_ROTATE_H;
    else
        action = mjMOUSE_ZOOM;

    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

static void scroll(GLFWwindow*, double, double yoffset)
{
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

int main(int, char**)
{
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);

    char error[1024] = "Could not load XML model";

    const char* model_path = "assets/scene.xml";

    m = mj_loadXML(model_path, nullptr, error, sizeof(error));
    if (!m)
    {
        std::cerr << "Ошибка загрузки assets/scene.xml:\n" << error << std::endl;
        return 1;
    }

    d = mj_makeData(m);
    if (!d)
    {
        std::cerr << "Ошибка создания mjData." << std::endl;
        mj_deleteModel(m);
        return 1;
    }

    if (!glfwInit())
    {
        std::cerr << "Ошибка GLFW init." << std::endl;
        mj_deleteData(d);
        mj_deleteModel(m);
        return 1;
    }

    window = glfwCreateWindow(1280, 720, "MuJoCo Pick and Place", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Ошибка создания окна GLFW." << std::endl;
        glfwTerminate();
        mj_deleteData(d);
        mj_deleteModel(m);
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetKeyCallback(window, keyboard);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    cam.azimuth = 135;
    cam.elevation = -25;
    cam.distance = 1.4;
    cam.lookat[0] = 0.0;
    cam.lookat[1] = 0.0;
    cam.lookat[2] = 0.85;

    RobotController controller(m);

    while (!glfwWindowShouldClose(window))
    {
        for (int i = 0; i < 3; ++i)
        {
            controller.update(d);
            mj_step(m, d);
        }

        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    mj_deleteData(d);
    mj_deleteModel(m);

    glfwTerminate();
    return 0;
}