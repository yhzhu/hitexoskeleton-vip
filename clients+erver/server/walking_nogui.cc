#include "mujoco.h"
// #include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "csvtools.h"
#include "utils.h"
#include <thread>
#include <signal.h>
#include <sys/time.h>
#include "mytimer.h"
#include <chrono>
//#include "cmp.h"
#include <signal.h>
#include "udp.h"
#include <unistd.h>
#include <termios.h>
#include <map>
#include <tuple>
#include <mutex>
using namespace std;

// sim thread synchronization
std::mutex mtx;
// model and ik data
std::string fIK = "model/q_header.csv";
std::string fModel = "model/subject_scaled_walk_mini.xml";
std::vector<std::vector<mjtNum>> qIn;
std::vector<std::string> header;

// MuJoCo data structures
mjModel *m = NULL; // MuJoCo model
mjData *d = NULL;  // MuJoCo data
int running_state = 0;//0-stopped, 1-running, 2-pause

//calc related
int idTX, idTY, idTZ, idRX, idRY, idRZ, idpelvis, idsphere, idpFootLeft, idpFootRight;
int idx = 0;
int steps = 1;
int onestep = 0;
bool firstround = true;
mjtNum pos, vel, acc;
mjtNum posLast, velLast, accLast;
mjtNum st = 0.001;
mjtNum dt = st * steps;
int _wndPos[2], _wndSize[2], _vpSize[2];
mjtNum comF[3], comT[3], comP[3], comO[3], pWrench[3], zmpPoint[3];
std::vector<std::vector<mjtNum>>  dataMatrix;
bool tosave = false;

FILE *fh = NULL;
//cmp_ctx_t cmp = {0};
//std::map<std::string,mjtNum*> vrepMsg;
#define   MSG  std::tuple<std::string,mjtNum*,int>
std::vector<MSG> vrepMsgs;
vector<char> vrepSignal ;
// int _wndPos[2], _wndSize[2],_vpSize[2];

// timer
double starttime;
chrono::system_clock::time_point tm_start;
mjtNum gettm(void)
{
    chrono::duration<double> elapsed = chrono::system_clock::now() - tm_start;
    return elapsed.count();
}


// void toggleFullscreen()
// {
//     static bool fullscrren = true;
//     GLFWmonitor *monitor = glfwGetPrimaryMonitor();
//     const GLFWvidmode *mode = glfwGetVideoMode(monitor);
//     if (fullscrren)
//     {
//         // backup window position and window size
//         glfwGetWindowPos(window, &_wndPos[0], &_wndPos[1]);
//         glfwGetWindowSize(window, &_wndSize[0], &_wndSize[1]);
//         glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
//         glfwSwapInterval(1);
//     }
//     else
//     {
//         glfwSetWindowMonitor(window, nullptr, _wndPos[0], _wndPos[1], _wndSize[0], _wndSize[1], 0);
//     }
//     fullscrren = !fullscrren;
//     _updateViewport = true;
// }

// // keyboard callback*********************************************************
// void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
// {
//     // backspace: reset simulation
//     if (act == GLFW_PRESS && key == GLFW_KEY_F2)
//     {
//         (mjVISSTRING[mjVIS_CONVEXHULL][1] == "0") ? mjVISSTRING[mjVIS_CONVEXHULL][1] = "1" : mjVISSTRING[mjVIS_CONVEXHULL][1] = "0";
//         mjv_defaultOption(&opt);
//     }
//     else if (act == GLFW_PRESS && key == GLFW_KEY_F3)
//     {
//         (mjVISSTRING[mjVIS_JOINT][1] == "0") ? mjVISSTRING[mjVIS_JOINT][1] = "1" : mjVISSTRING[mjVIS_JOINT][1] = "0";
//         mjv_defaultOption(&opt);
//     }
//     else if (act == GLFW_PRESS && key == GLFW_KEY_F4)
//     {
//         (mjVISSTRING[mjVIS_COM][1] == "0") ? mjVISSTRING[mjVIS_COM][1] = "1" : mjVISSTRING[mjVIS_COM][1] = "0";
//         mjv_defaultOption(&opt);
//     }
//     else if (act == GLFW_PRESS && key == GLFW_KEY_F5)
//     {
//         (scn.flags[mjRND_WIREFRAME] == 0) ? scn.flags[mjRND_WIREFRAME] = 1 : scn.flags[mjRND_WIREFRAME] = 0;
//         printf("mjRNDSTRING[mjRND_WIREFRAME][1]:%s\n", mjRNDSTRING[mjRND_WIREFRAME][1]);
//     }
//     else if (act == GLFW_PRESS && key == GLFW_KEY_F6)
//     {
//         toggleFullscreen();
//     }
//     else if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
//     {
//         glfwSetWindowShouldClose(window, GL_TRUE);
//     }
// }

// // mouse button callback*********************************************************
// void mouse_button(GLFWwindow *window, int button, int act, int mods)
// {
//     // update button state
//     button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
//     button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
//     button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

//     // update mouse position
//     glfwGetCursorPos(window, &lastx, &lasty);
// }

// // mouse move callback*********************************************************
// void mouse_move(GLFWwindow *window, double xpos, double ypos)
// {
//     // no buttons down: nothing to do
//     if (!button_left && !button_middle && !button_right)
//         return;

//     // compute mouse displacement, save
//     double dx = xpos - lastx;
//     double dy = ypos - lasty;
//     lastx = xpos;
//     lasty = ypos;

//     // get current window size
//     int width, height;
//     glfwGetWindowSize(window, &width, &height);

//     // get shift key state
//     bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
//                       glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

//     // determine action based on mouse button
//     mjtMouse action;
//     if (button_right)
//         action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
//     else if (button_left)
//         action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
//     else
//         action = mjMOUSE_ZOOM;

//     // move camera
//     mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
// }

// // scroll callback*********************************************************
// void scroll(GLFWwindow *window, double xoffset, double yoffset)
// {
//     // emulate vertical mouse motion = 5% of window height
//     mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
// }

void initMj()
{
    // load and compile model
    char error[1000] = "Could not load binary model";
    m = mj_loadXML(fModel.c_str(), 0, error, 1000);

    if (!m)
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    //read qIK file
    // int startline = 0;
    // if(fIK.find("header")!=-1)startline = 1;
    // printf("qIKfile:%s,startline:%d\n",fIK.c_str(),startline);
    int rows = readMatrixFromFile(fIK, qIn, true, &header);
    printf("%zd rows %zd cols data read\n", qIn.size(), qIn[0].size());
    printf("total mass=%0.3f\n", mj_getTotalmass(m));

    idpelvis = mj_name2id(m, mjOBJ_BODY, "pelvis");
    idsphere = mj_name2id(m, mjOBJ_BODY, "sphere");
    idTX = mj_name2id(m, mjOBJ_JOINT, "pelvis_tx");
    idTY = mj_name2id(m, mjOBJ_JOINT, "pelvis_ty");
    idTZ = mj_name2id(m, mjOBJ_JOINT, "pelvis_tz");
    idRX = mj_name2id(m, mjOBJ_JOINT, "pelvis_list");
    idRY = mj_name2id(m, mjOBJ_JOINT, "pelvis_tilt");
    idRZ = mj_name2id(m, mjOBJ_JOINT, "pelvis_rotation");
    idpFootLeft = mj_name2id(m, mjOBJ_BODY, "pFootLeft");
    idpFootRight = mj_name2id(m, mjOBJ_BODY, "pFootRight");
}

// void resize(GLFWwindow* wnd, int cx, int cy)
// {
//     _updateViewport = true;
// }

// void initRender()
// {
//     // init GLFW
//     if (!glfwInit())
//         mju_error("Could not initialize GLFW");

//     // create window, make OpenGL context current, request v-sync
//     window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
//     glfwMakeContextCurrent(window);
//     glfwSwapInterval(1);

//     glfwSetWindowSizeCallback( window, resize );

//     // initialize visualization data structures

//     mjv_defaultCamera(&cam);
//     mjv_defaultOption(&opt);
//     mjv_defaultScene(&scn);
//     mjr_defaultContext(&con);

//     // create scene and context
//     mjv_makeScene(m, &scn, 2000);
//     mjr_makeContext(m, &con, mjFONTSCALE_150);

//     // install GLFW mouse and keyboard callbacks
//     glfwSetKeyCallback(window, keyboard);
//     glfwSetCursorPosCallback(window, mouse_move);
//     glfwSetMouseButtonCallback(window, mouse_button);
//     glfwSetScrollCallback(window, scroll);
// }

// void render()
// {
//     // get framebuffer viewport
//     mjrRect viewport = {0, 0, 0, 0};
//     if (_updateViewport)
//     {
//         glfwGetFramebufferSize(window, &_vpSize[0], &_vpSize[1]);
//         glViewport(0, 0, _vpSize[0], _vpSize[1]);
//         _updateViewport = false;
//     }

//     glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

//     // update scene and render
//     mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
//     mjr_render(viewport, &scn, &con);

//     // swap OpenGL buffers (blocking call due to v-sync)
//     glfwSwapBuffers(window);

//     // process pending GUI events, call GLFW callbacks
//     glfwPollEvents();
// }

void finish()
{
    //free visualization storage
    // mjv_freeScene(&scn);
    // mjr_freeContext(&con);

    // free MuJoCo model and data
    mj_deleteData(d);
    mj_deleteModel(m);

    // // terminate GLFW (crashes with Linux NVidia drivers)
    // #if defined(__APPLE__) || defined(_WIN32)
    //     glfwTerminate();
    // #endif
}

void getZmp()
{
    comF[0] = d->qfrc_inverse[idTX];
    comF[1] = d->qfrc_inverse[idTY];
    comF[2] = d->qfrc_inverse[idTZ];
    comT[0] = d->qfrc_inverse[idRX];
    comT[1] = d->qfrc_inverse[idRY];
    comT[2] = d->qfrc_inverse[idRZ];
    comP[0] = d->qpos[idTX];
    comP[1] = d->qpos[idTY];
    comP[2] = d->qpos[idTZ];
    comO[0] = d->qpos[idRX];
    comO[1] = d->qpos[idRY];
    comO[2] = d->qpos[idRZ];

    //********************************************************
    //step1 把力矩从关节坐标系转换到世界坐标系 comT -> comTNew

    mjtNum offset[3];
    mjtNum dir[3], dir1[3];
    mjtNum comTCal[3];
    mjtNum rotMatX[9] = {1, 0, 0,
                         0, cos(comO[0]), -sin(comO[0]),
                         0, sin(comO[0]), cos(comO[0])};

    mjtNum rotMatY[9] = {cos(comO[1]), 0, sin(comO[1]),
                         0, 1, 0,
                         -sin(comO[1]), 0, cos(comO[1])};

    mjtNum rotMatZ[9] = {cos(comO[2]), -sin(comO[2]), 0,
                         sin(comO[2]), cos(comO[2]), 0,
                         0, 0, 1};
    mjtNum rotYX[9], rotYXZ[9];
    mjtNum comTNew[3];
    mju_mulMatMat(rotYX, rotMatX, rotMatY, 3, 3, 3);
    mju_mulMatMat(rotYXZ, rotMatZ, rotYX, 3, 3, 3);
    mju_mulMatMat(comTNew, comT, rotYXZ, 1, 3, 3);
    //mju_printMat(rotZXY,3,3);
    //mju_printMat(rotZX,3,3);
    //mju_printMat(comF,1,3);
    //mju_printMat(comTNew,1,3);
    //mju_copy3(comT,comTNew);

    //********************************************************
    //step2 由于数据误差，合力和合力矩并不完全垂直，将力矩进行校正到垂直于合力方向且幅值不变
    mju_cross(dir, comF, comTNew);
    mju_cross(dir1, dir, comF);
    mju_normalize3(dir1);
    mju_scl3(comTNew, dir1, mju_norm3(comTNew));
    mju_cross(dir, comF, comTNew);

    //********************************************************
    //step3 计算用于产生合力矩的合力作用点的平移分量offset，这样原来的力和力矩转换为一个单独的力矢量
    mjtNum sintheta = mju_norm3(dir) / mju_norm3(comTNew) / mju_norm3(comF);
    mjtNum len = mju_norm3(comTNew) / mju_norm3(comF) / sintheta;
    mju_normalize3(dir);
    mju_scl3(offset, dir, len);
    mju_add3(pWrench, comP, offset);

    //********************************************************
    //step3.a 校核一下偏移量叉乘合力是否等于合力矩，offset x comF = comTCal ?= comTNew
    mju_cross(comTCal, offset, comF);
    mjtNum force[3] = {0, 0, 1000};
    mjtNum torque[3];
    mjtNum point[3] = {0, 0, 0};
    mjtNum qfrc[100];

    //mj_applyFT(m,d,comF,comTNew,point,idpelvis,qfrc);
    // printf("sintheta:%0.3f*******************\n",sintheta);
    // printf("comT:\n");
    // mju_printMat(comT,1,3);
    // printf("comTCal:\n");
    // mju_printMat(comTCal,1,3);
    // printf("comTNew:\n");
    // mju_printMat(comTNew,1,3);
    // printf("dir:\n");
    // mju_printMat(dir,1,3);

    //mju_copy3(pWrench,comP);

    //printf("sintheta=%0.3f\n",sintheta);

    //********************************************************
    //step4 合力反向延长线与地面交点即为zmp点 -zmpPoint
    mju_cross(comTCal, offset, comF);
    mjtNum v1[3] = {0, 0, 0};
    mjtNum v2[3] = {0, 0, 1};
    mjtNum v3[3] = {pWrench[0], pWrench[1], pWrench[2]};
    mjtNum v4[3] = {comF[0], comF[1], comF[2]};
    //mju_normalize3(v4);
    //PL射线端点，PP平面一点，e射线单位向量，q平面信息，P返回交点
    lineIntersection(v1, v2, v3, v4, zmpPoint);

    mju_copy3(d->mocap_pos + m->body_mocapid[idsphere] * 3, zmpPoint);

    // if (firstround)
    // {
    //     printf("%0.3f  \t%0.6f \t%0.6f \t%0.6f \t%0.6f \t%0.6f \t%0.6f\n",(idx-steps)*st,comF[0],comF[1],comF[2],comTNew[0],comTNew[1],comTNew[2]);
    // }
}

void setData()
{
    int rows = qIn.size();
    int cols = header.size();
    int dof = cols - 1;
    if (true)
    {
        if (idx < rows)
        {
            mjtNum t = qIn[idx][0];
            for (int j = 1; j < cols; j++)
            {
                const char *jname = header[j].c_str();
                int id = mj_name2id(m, mjOBJ_JOINT, jname);
                if (id != -1)
                {
                    pos = qIn[idx][j];
                    if (m->jnt_type[id] == 3)
                        pos = pos * mjPI / 180.0;
                    if (idx == 0)
                    {
                        d->qacc[id] = 0;
                        d->qvel[id] = 0;
                    }
                    else if (idx == steps)
                    {
                        d->qacc[id] = 0;
                        d->qvel[id] = (pos - d->qpos[id]) / dt;
                    }
                    else
                    {
                        vel = (pos - d->qpos[id]) / dt;
                        d->qacc[id] = (vel - d->qvel[id]) / dt;
                        d->qvel[id] = (pos - d->qpos[id]) / dt;
                    }
                    d->qpos[id] = pos;
                }
            }
            if(running_state == 1)idx += steps;

            if(running_state == 2 && onestep != 0)
            {
                idx += onestep;
                onestep = 0;
            }

        }
        else
        {
            idx = 0;
            firstround = false;
        }
    }
}

void invDyna()
{
    mj_inverse(m, d);
}

void footForce()
{
    bool insideFoot[2] = {false, false};
    mjtNum pntFoot[2][3];
    mjtNum forceFoot[2][3] ;
    //int idlfoot = mj_name2id(m,mjOBJ_GEOM, "l_foot");
    // int idlfootBox = mj_name2id(m, mjOBJ_GEOM, "l_foot_box");
    // //int idrfoot = mj_name2id(m,mjOBJ_GEOM, "r_foot");
    // int idrfootBox = mj_name2id(m, mjOBJ_GEOM, "r_foot_box");
    int ifootBox[2] = {mj_name2id(m, mjOBJ_GEOM, "l_foot_box"), mj_name2id(m, mjOBJ_GEOM, "r_foot_box")};

    mjtNum size[3] = {0.08, 0.005, 0.07};
    mjtNum pnt[3] = {pWrench[0], pWrench[1], pWrench[2]};
    mjtNum vec[3] = {-comF[0], -comF[1], -comF[2]};
    mjtNum hitpoint[3];
    mju_normalize3(vec);

    mjtNum *pos[2];
    mjtNum *mat[2];
    mjtNum dis[2];
    mjtNum dir[2][3];

    // if(dis==-1)dis = mj_rayMesh(m,d,idrfoot,pnt,vec);

    for (int i = 0; i < 2; i++)
    {
        pos[i] = d->geom_xpos + ifootBox[i] * 3;
        mat[i] = d->geom_xmat + ifootBox[i] * 9;

        dis[i] = mju_rayGeom(pos[i], mat[i], size, pnt, vec, mjGEOM_BOX);
        if (dis[i] != -1)
        {
            mju_addScl3(hitpoint, pnt, vec, dis[i]);
            hitpoint[2] = 0;
            mju_copy3(d->mocap_pos + m->body_mocapid[(i == 0) ? idpFootLeft : idpFootRight] * 3, zmpPoint);
            mju_copy3(forceFoot[i], comF);
            //printf("%0.3f,%0.3f,%0.3f\n", forceFoot[i][0], forceFoot[i][1], forceFoot[i][2]);
            insideFoot[i] = true;
            break;
        }
    }

    if (insideFoot[0] == false && insideFoot[1] == false)
    {
        mjtNum lenF[2];
        mjtNum dirF[2][3];
        mjtNum eleF[2][3];
        mjtNum errF;
        mjtNum cacF[3];
         for (int i = 0; i < 2; i++)
        {
            mju_sub3(dir[i], zmpPoint, pos[i]);
            mju_normalize3(dir[i]);

            mjtNum center[3] = {pos[i][0] ,pos[i][1],0};
            mju_addScl3(pntFoot[i], center, dir[i], 0.08);
            pntFoot[i][2] = 0;
            mju_copy3(d->mocap_pos + m->body_mocapid[(i == 0) ? idpFootLeft : idpFootRight] * 3, pntFoot[i]);

            mju_sub3(dirF[i], pnt,pntFoot[i]);
            mju_normalize3(dirF[i]);
        }

        lenF[1] = (dirF[0][2]*comF[0] - dirF[0][0]*comF[2])/(dirF[1][0]*dirF[0][2] - dirF[0][0]*dirF[1][2]) ;
        lenF[0] = (dirF[1][0]*comF[2] - dirF[1][2]*comF[0])/(dirF[1][0]*dirF[0][2] - dirF[0][0]*dirF[1][2]) ;
        for (int i = 0; i < 2; i++)
        {
            mju_scl3(forceFoot[i], dirF[i], lenF[i]);
        }
        mju_add3(cacF,forceFoot[0],forceFoot[1]);
        errF = mju_norm3(comF) - mju_norm3(cacF);
    }

    if(firstround)
    {
        std::vector<mjtNum> row = {idx*st,forceFoot[0][0], forceFoot[0][1], forceFoot[0][2],forceFoot[1][0], forceFoot[1][1], forceFoot[1][2]};
        dataMatrix.push_back(row);
        //printf("left foot force: %0.3f,%0.3f,%0.3f\n", forceFoot[0][0], forceFoot[0][1], forceFoot[0][2]);
        tosave = true;
    }
    else
    {
        if(tosave)
        {
            tosave = false;
            const char fame[] = "./result.csv";
            const char header[] = "time, fxLeft, fyLeft, fzLeft, fxRight, fyRight, fzRight";
            printMatrixToFile(dataMatrix,header,fame);
        }
    }
    
}

void calc()
{
    int idpelvis = mj_name2id(m, mjOBJ_BODY, "pelvis");
    int idsphere = mj_name2id(m, mjOBJ_BODY, "sphere");

    //step1 运动采集数据输入到动力学模型，速度和加速度为数值差分，pos,vel,acc
    setData();
    //step2 逆运动学计算
    invDyna();
    //step3 zmp点计算
    getZmp();
    //step4 两脚受力分解，待完成
    footForce();
    
    double elapsed = gettm() - starttime;
    if(elapsed >= 1.0)
    {
        starttime = gettm();
        // printf("simulation time: %0.3f\n",idx*st);
    }
}

volatile static long counter = 0;
static void notify( int signum ){
        static int counter;
        if( signum == SIGALRM ){
            calc();
            ++ counter;
            //cout << counter << endl;
        }
}

// void sig_exit(int s)
// {
// 	tcp.exit();
// 	exit(0);
// }

// void setupTcp(const char ip[], int port)
// {
// 	signal(SIGINT, sig_exit);
// 	tcp.setup(ip,port);
// }

// static void error_and_exit(const char *msg) {
//     fprintf(stderr, "%s\n\n", msg);
//     exit(EXIT_FAILURE);
// }

// static bool read_bytes(void *data, size_t sz, FILE *fh) {
//     return fread(data, sizeof(uint8_t), sz, fh) == (sz * sizeof(uint8_t));
// }

// static bool file_reader(cmp_ctx_t *ctx, void *data, size_t limit) {
//     return read_bytes(data, limit, (FILE *)ctx->buf);
// }

// static bool file_skipper(cmp_ctx_t *ctx, size_t count) {
//     return fseek((FILE *)ctx->buf, count, SEEK_CUR);
// }

// static size_t file_writer(cmp_ctx_t *ctx, const void *data, size_t count) {
//     return fwrite(data, sizeof(uint8_t), count, (FILE *)ctx->buf);
// }

// void initPack()
// {
//     if (fh == NULL) {
//         error_and_exit("Error opening data.dat");
//     }
// 	cmp_init(&cmp, fh, file_reader, file_skipper, file_writer);
//     fh = fopen("cmp_data.dat", "w+b");
// }

// void packMsg()
// {
//     if(cmp.buf)
//     {
//         if (!cmp_write_str(&cmp, "PCOM", 5)) {
//             error_and_exit(cmp_strerror(&cmp));
//         }

//         if (!cmp_write_array(&cmp, 2)) {
//             error_and_exit(cmp_strerror(&cmp));
//         }

//         if (!cmp_write_str(&cmp, "Hello", 5)) {
//             error_and_exit(cmp_strerror(&cmp));
//         }

//         if (!cmp_write_str(&cmp, "MessagePack", 11)) {
//             error_and_exit(cmp_strerror(&cmp));
//         }
//     }
// }

// void sendTCPMsg(const char* buf,int count)
// {
// 	tcp.Send(buf,count);
// }

void resetVrepSignal()
{
    vrepSignal.clear();
    char buf[4] = {0};
    for(int i=0;i<4;i++)vrepSignal.push_back(buf[i]);
}

void packVrepSignal(const char type[], mjtNum data[], int len)
{
    int count = len;
    float* data1 = new float[len];
    for(int i=0;i<len;i++)data1[i] = data[i];
    char buf[4*count+8];
    memcpy(&buf[0],type,4);
    memcpy(&buf[4],&count,4);
    memcpy(&buf[8],&data1[0],count*4);
    for(int i=0;i<sizeof(buf);i++)
    {
        vrepSignal.push_back(buf[i]);
    }
}

void sendVrepSignal()
{
    int totallen = vrepSignal.size()+4;
    char buf1[4];
    memcpy(buf1,&totallen,4);
    char buf2[] = "DEND";
    for(int i=0;i<4;i++)
    {
        vrepSignal[i] = buf1[i];
        vrepSignal.push_back(buf2[i]);
    }
    udp_send(vrepSignal.data(),vrepSignal.size());
    //printf("vrepSignal.size():%ld\n",vrepSignal.size());
    // for(int i=0;i<vrepSignal.size();i++)
    // {
    //     char ch =*(vrepSignal.data()+i);
    //     printf("vrepSignal.data[%d]:%d-%c\n",i,ch,ch);
    // }

}

char getch(void)
{
    char buf = 0;
    struct termios old = {0};
    fflush(stdout);
    if(tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if(tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if(read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if(tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    //printf("%c\n", buf);
    return buf;
}

void sendVrepMsg(MSG msg)
{
    vrepMsgs.clear();
    //vrepMsgs.push_back(MSG("COMP",comP,3));
    vrepMsgs.push_back(msg);
    resetVrepSignal();
    for(int i=0;i<vrepMsgs.size();i++)
    {
        packVrepSignal(std::get<0>(vrepMsgs[i]).c_str(),std::get<1>(vrepMsgs[i]),std::get<2>(vrepMsgs[i]));
    }
    sendVrepSignal();
}

void time_handler1(size_t timer_id, void * user_data)
{
    mtx.lock();
    calc();
    mtx.unlock();
}

void on_udpmsg(char* msg)
{
    if(strcmp(msg,"data")==0)
    {
        if(commtoClient)
        {
            mtx.lock();
            sendVrepMsg(MSG("JPOS",d->qpos,m->nq));
            mtx.unlock();
        }
    }
}

// main function
int main()
{
    //init
    initMj();
    //initRender();
    udp_init(ANY_IP, 20001, 20000, on_udpmsg);

    size_t timer1 = 0;
    init_timer();
    bool startstop = true;
    tm_start = chrono::system_clock::now();
    starttime = gettm();
    
    while (true)
    {
        printf("start mujocoRemoteGUI client, or press [q] to quit\n");
        char ch = getch();
        // printf("%d\n",ch);
        if(ch == 'q')
        {
            break;
        }

        if(ch == ' ')
        {
            if(running_state == 0)
            {
                timer1 = start_timer(steps, time_handler1, TIMER_PERIODIC, NULL);
                sendVrepMsg(MSG("test",NULL,0));
                running_state = 1;
            }
            else if(running_state == 1  || running_state == 2)
            {
                stop_timer(timer1);
                sendVrepMsg(MSG("stop",NULL,0));
                running_state = 0;
            }
        }

        if(ch == 'd')
        {
            running_state = 2;
            onestep = 1;
        }

        if(ch == 'a')
        {
            running_state = 2;
            onestep = -1;
        }
        // calc();
        // chrono::duration<double> elapsed = chrono::system_clock::now() - tm_start;

        // // if(icount % (1000 / steps) == 0)
        // // {
        // //     printf("calc loops %d, time:%0.3f\n", icount,(gettm() - starttime));
        // // }
        // icount++;
        // tm_until = tm_until + std::chrono::milliseconds(steps);
        // //this_thread::sleep_until(tm_until);
    }
    // to quit
    finalize();
    printf("finish\n");
    finish();

    return 1;
}
