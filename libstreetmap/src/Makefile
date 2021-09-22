# Copyright 2021 University of Toronto
#  
# Permission is hereby granted, to use this software and associated 
# documentation files (the "Software") in course work at the University 
# of Toronto, or for personal use. Other uses are prohibited, in 
# particular the distribution of the Software either publicly or to third 
# parties.
# 
# The above copyright notice and this permission notice shall be included in 
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#


########################################################################
## Makefile for basic_application
########################################################################

# turn on/off verbosity with 'make {all|release|debug|clean} VERBOSE=1'. Default is silent.
ifeq ($(VERBOSE),1)
	ECHO := 
else
	ECHO := @
endif

# the compiler
CXX = g++

# target EXE
TARGET_DIR = .
TARGET = basic_application

# the version of GTK being used
GTK_VERSION_NUM = 3.0

# the base directory of EZGL
EZGL_DIR = ezgl

# get the source and header files for the application and from EZGL
SRCS = $(wildcard ./*.cpp $(EZGL_DIR)/*.cpp)
HDRS = $(wildcard ./*.h $(EZGL_DIR)/*.hpp)

# the GTK include directories. Runs "pkg-config --cflags gtk+-3.0" to get the include directories.
GTK_INCLUDE_DIRS := $(shell pkg-config --cflags gtk+-$(GTK_VERSION_NUM) x11)

# the GTK libraries. Runs "pkg-config --libs gtk+-3.0" to get the libraries to include for GTK.
GTK_LIBS := $(shell pkg-config --libs gtk+-$(GTK_VERSION_NUM) x11)

# set the include directories
INC_DIRS = . $(EZGL_DIR) $(EZGL_DIR)/..

# compiler flags
CXX_FLAGS = -g -Wall -std=c++14


# 'make all' target
all: $(TARGET_DIR)/$(TARGET)

# create the exe
$(TARGET_DIR)/$(TARGET) : Makefile $(SRCS)
	$(ECHO) $(CXX) $(CXX_FLAGS) $(foreach D,$(INC_DIRS),-I$D) $(GTK_INCLUDE_DIRS) $(SRCS) $(GTK_LIBS) -o $(TARGET_DIR)/$(TARGET)

# clean the EXE 
clean:
	$(ECHO) rm -f $(TARGET_DIR)/$(TARGET)

# 'make release' is the same as 'make all' but the -O3 flag is added
release: all
CXX_FLAGS += -O3

# 'make debug' is the same as 'make all'. Currently nothing is added here
debug: all


.PHONY: all release debug clean
