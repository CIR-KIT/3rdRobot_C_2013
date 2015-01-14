CC = g++
CFLAGS = -g -I/usr/local/include/urg_c -lm -lurg_c -pthread -lopencv_core -lopencv_highgui -lopencv_imgproc
LDFLAGS = -L/usr/local/lib
LDLIBS = -lm -lurg_c -pthread -lopencv_core -lopencv_highgui -lopencv_imgproc

robot: ./GPS/gps.o ./InputCheck/inputcheck.o ./Encoder/hexa_to_binary.o ./Encoder/encoder.o 
gps.o: ./GPS/gps.h ./GPS/_gps.h
inputcheck.o: ./InputCheck/inputcheck.h ./InputCheck/_inputcheck.h
hexa_to_binary.o: ./Encoder/_encoder.h ./Encoder/encoder.h
encoder.o: ./Encoder/_encoder.h ./Encoder/encoder.h