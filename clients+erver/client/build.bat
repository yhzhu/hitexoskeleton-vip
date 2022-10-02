g++ -O2 -I./include -L./bin -std=c++11  mjGUI.cc ./include/uitools.c ./bin/glfw3.lib  ./bin/mujoco210.lib  -lWs2_32 -o ./bin/mjGUI.exe

