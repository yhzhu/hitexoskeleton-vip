#！/bin/bash
g++ -O2 -I./include -L./bin -std=c++11 -mavx -pthread -Wl,-rpath,'$ORIGIN' walking_nogui.cc ./include/csvtools.cc   ./include/mytimer.c  ./include/cmp.c ./include/TCPClient.cpp -lmujoco210nogl -o ./bin/walking_nogui
