# ==============================================================================
# VLSI Global Router Build System
# ==============================================================================

CXX      = g++
CXXFLAGS = -m64 -O3 -fPIC -fexceptions -DNDEBUG -Wall
LDFLAGS  = -lm -pthread

TARGET   = ROUTE.exe
OBJS     = main.o global_router.o

# Default build target
all: $(TARGET)

# Link the executable
$(TARGET): $(OBJS)
	rm -f $(TARGET)
	$(CXX) $(CXXFLAGS) $(OBJS) $(LDFLAGS) -o $(TARGET)

# Compile main logic
main.o: main.cpp global_router.h
	rm -f main.o
	$(CXX) $(CXXFLAGS) main.cpp -c

# Compile routing engine
global_router.o: global_router.cpp global_router.h
	rm -f global_router.o
	$(CXX) $(CXXFLAGS) global_router.cpp -c

# Clean build artifacts
clean:
	rm -f *~ *.o $(TARGET)