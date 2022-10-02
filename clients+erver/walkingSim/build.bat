g++ -O2 -I./include -I./ -L./bin -std=c++11  main.cc  ./include/uitools.c ./bin/glfw3.lib  ./bin/mujoco210.lib  -lWs2_32 -o ./bin/walkingSim.exe

