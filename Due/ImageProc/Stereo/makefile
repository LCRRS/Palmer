stereo_detect: stereo_detect.o PID.o
	g++ -std=c++11 -o  stereo_detect stereo_detect.o PID.o `pkg-config opencv --cflags --libs` -lserial

stereo_detect.o: stereo_detect.cpp PID.h
	g++ -c -std=c++11 -o stereo_detect.o stereo_detect.cpp

PID.o: PID.cpp PID.h
	g++ -c -std=c++11 -o PID.o PID.cpp