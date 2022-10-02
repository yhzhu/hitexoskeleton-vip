#include "mjGUI.h"
#include "udp.h"
bool testing = false;

void on_udpmsg(char* msg)
{
    mjtNum qpos[BUF_SIZE] = {0};
    int framelen = -1;
    float data[1000];
    int datacount = 0;
    char type[5] = {0};
    memcpy(&framelen,&msg[0],4);
    memcpy(type,&msg[4],4);
    memcpy(&datacount,&msg[8],4);
    if(strcmp(type,"test")==0)
    {
        testing = true;
    }
    else if(strcmp(type,"stop")==0)
    {
        testing = false;
    }
    // printf("udpmsg type:%s,framelen=%d,datacount=%d\n",type,framelen,datacount);

    if(strcmp(type,"JPOS")==0 && datacount == 51)
    {
        memcpy(data,&msg[12],datacount*4);
        //printf("framelen = %d, type = %s, datacount=%d \n", framelen,type,datacount);
        //printf("data=[%0.3f,%0.3f,%0.3f]\n", data[0],data[1],data[2]);
        for(int i=0;i<datacount;i++)qpos[i] = data[i];
        setQPos(qpos);
    }
}


// run event loop
int main(int argc, const char** argv)
{
    // initialize everything
    init();

    // udp_init(settings.ip, 20000, 20001, on_udpmsg);

    // udp_send("CMD1",4);

    // request loadmodel if file given (otherwise drag-and-drop)
    if( argc>1 )
    {
        mju_strncpy(filename, argv[1], 1000);
        settings.loadrequest = 2;
    }

    // start simulation thread
    std::thread simthread(simulate);

    // event loop
    while( !glfwWindowShouldClose(window) && !settings.exitrequest )
    {
        // start exclusive access (block simulation thread)
        mtx.lock();

        // load model (not on first pass, to show "loading" label)
        if( settings.loadrequest==1 )
            loadmodel();
        else if( settings.loadrequest>1 )
            settings.loadrequest = 1;

        // handle events (calls all callbacks)
        glfwPollEvents();

        // prepare to render
        prepare();

        // end exclusive access (allow simulation thread to run)
        mtx.unlock();
        
        if(testing)
        {
            udp_send("data",4);
        }
        // render while simulation is running
        render(window);


    }

    // stop simulation thread
    settings.exitrequest = 1;
    simthread.join();

    // delete everything we allocated
    uiClearCallback(window);
    mj_deleteData(d);
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 0;
}