VPATH = ./
OBJDIR = gen
SRCDIR = src
EXECUTABLE  = rpi_prog
_SOURCES = main.cpp state_funcs.cpp vision_toolkit.cpp
SOURCES = $(_SOURCES:%.cpp=$(SRCDIR)/%.cpp)
OBJECTFILES = $(_SOURCES:%.cpp=$(OBJDIR)/%.o)
HEADERS = $(wildcard $(SRCDIR)/*.h)

DEBUG = -g 
CC = g++
CFLAGS = -c -O3 -fopenmp -Wall `pkg-config --cflags opencv` 
LIBS = `pkg-config --libs opencv` -L/usr/lib64 -lstdc++ -fopenmp -lraspicam -lraspicam_cv

$(EXECUTABLE): $(OBJECTFILES)
	$(CC) -o $(EXECUTABLE) $(OBJECTFILES) $(LIBS)

%.o: ../$(SRCDIR)/%.cpp $(HEADERS)
	$(CC) $(CFLAGS) -w -c  $(DEBUG) $< -o $@ 

runs: $(EXECUTABLE)
	./$(EXECUTABLE) 2>/dev/null

run: $(EXECUTABLE)
	./$(EXECUTABLE)

fix: 
	make clean
	make run
		
.PHONY: clean fix

clean:
	rm $(OBJECTFILES) $(EXECUTABLE)
