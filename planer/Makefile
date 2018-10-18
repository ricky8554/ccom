CC = gcc
CXX = g++

all: executive controler

executive: executive.cpp communication.o
	$(CXX)  executive.cpp communication.o -Wall -g -o executive -std=c++11 -g -O2

controler: controler.cpp communication.o
	$(CXX)  controler.cpp communication.o -Wall -g -o controler -std=c++11 -g -O2

communication.o: communication.cpp communication.h
	$(CXX) -g -c communication.cpp -std=c++11 -g -O2





clean :
	-rm executive controler communication.o 

