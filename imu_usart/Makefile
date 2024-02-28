CC=gcc

# Include directory for header files

INCLUDE_DIRS= CMAKE_CURRENT_SOURCE_DIR
SOURCES = main.cpp serial_port.cpp 
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=main

# Add the include directory flag
CFLAGS=-I$(INCLUDE_DIRS)

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(CFLAGS) $(OBJECTS) -o $@
	@echo "Cleaning up..."
	#@rm -f $(OBJECTS)

%.o: %.c
	$(CXX) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	#rm -f $(OBJECTS) $(EXECUTABLE)
	
	
	
