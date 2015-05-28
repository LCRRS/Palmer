color_detect: color_detect.o PID.o
	g++ -std=c++11 -o  color_detect color_detect.o PID.o `pkg-config opencv --cflags --libs`

color_detect.o: color_detect.cpp PID.h
	g++ -c -std=c++11 -o color_detect.o color_detect.cpp

PID.o: PID.cpp PID.h
	g++ -c -std=c++11 -o PID.o PID.cpp