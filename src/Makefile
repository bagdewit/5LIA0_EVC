VPATH = ./
OBJDIR = ../gen
SRCDIR = ./src
EXECUTABLE  = vision_demo
_SOURCES = demo.cpp vision_toolkit.cpp
SOURCES = $(_SOURCES:%.cpp=$(SRCDIR)/%.cpp)
OBJECTFILES = $(_SOURCES:%.cpp=$(OBJDIR)/%.o)

DEBUG = -g 
CC = g++
CFLAGS = -c -O3 -fopenmp -Wall `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv` -L/usr/lib64 -lstdc++ -fopenmp

$(EXECUTABLE): $(OBJECTFILES)
	$(CC) -o $(EXECUTABLE) $(OBJECTFILES) $(LIBS)

%.o: ../$(SRCDIR)/%.cpp
	$(CC) $(CFLAGS) -w -c  $(DEBUG) $< -o $@ 

fix: clean $(OBJECTFILES)
	$(CC) -o $(EXECUTABLE) $(OBJECTFILES) $(LIBS)

runv: $(EXECUTABLE)
	./$(EXECUTABLE) "../../test_images_videos/video_new_scaled.h264"

runs: $(EXECUTABLE)
	./$(EXECUTABLE) 2>/dev/null

run: $(EXECUTABLE)
	./$(EXECUTABLE)

.PHONY: clean fix

clean:
	rm $(OBJECTFILES) $(EXECUTABLE)
