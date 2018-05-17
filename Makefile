SYSTEM = x86-64_linux
LIBFORMAT = static_pic
CPLEX_TOP = ../CPLEX
EXDIR = ../optimizer-cplex
PROTOBUF_TAG = 3.3.0
GLOG_TAG = 0.3.5
GFLAGS_TAG = 2.2.0
MKDIR = mkdir
UNIX_PROTOBUF_DIR = dependencies/install
CMAKE = cmake
PROTOBUF_INC = -I$(UNIX_PROTOBUF_DIR)/include
CCC = g++ -fPIC -std=c++0x -fwrapv
GFLAGS_INC = -I$(EXDIR)/dependencies/install/include -I$(CPLEX_TOP)/cplex/include -I$(CPLEX_TOP)/cpoptimizer/include -I$(CPLEX_TOP)/concert/include

CFLAGS = -Wno-deprecated $(PROTOBUF_INC) 
#------------------------------------------------------------
#
# When you adapt this makefile to compile your CPLEX programs
# please copy this makefile and set CPLEXDIR and CONCERTDIR to
# the directories where CPLEX and CONCERT are installed.
#
#------------------------------------------------------------
export ILOG_HOME= ../CPLEX

export LD_LIBRARY_PATH=$ILOG_HOME/cpoptimizer/bin/x86-64_linux

export ILOG_LICENSE_FILE=$ILOG_HOME/license

# ILOGSTUDIODIR =/home/hb/ILOG/CPLEX_Studio_AcademicResearch122
CPLEXDIR = $(ILOG_HOME)/cplex
CONCERTDIR = $(ILOG_HOME)/concert

# ---------------------------------------------------------------------
# Compiler selection
# ---------------------------------------------------------------------

# CCC = g++
CC = gcc
JAVAC = javac

# ---------------------------------------------------------------------
# Compiler options
# ---------------------------------------------------------------------

CCOPT = -m64 -O -fPIC -fexceptions -DNDEBUG -DIL_STD

# ---------------------------------------------------------------------
# Link options and libraries
# ---------------------------------------------------------------------

CPLEXBINDIR = $(CPLEXDIR)/bin/x86-64_linux
CONCERTXBINDIR = $(CONCERTDIR)/include
CPLEXLIBDIR = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)

CCLNFLAGS = $(ILOG_HOME)/cpoptimizer/lib/x86-64_linux/static_pic -lcp -L $(CPLEXLIBDIR) -lilocplex -lcplex -L $(CONCERTLIBDIR) -lconcert -m64 -lm -pthread
LDFLAGS = -L $(ILOG_HOME)/cpoptimizer/lib/x86-64_linux/static_pic -lcp -L$(CPLEXDIR)/lib/x86-64_linux/static_pic -lcplex -L$(CONCERTDIR)/lib/x86-64_linux/static_pic -lconcert  -lpthread -lm -ldl 

CCFLAGS = $(CCOPT) -I $(CPLEXBINDIR) -I $(CONCERTXBINDIR) -w

# include $(CPLEX_TOP)/Makefile

.PHONY: all local_clean
third_party: \
	install_protobuf \
	install_gflags \
	install_glog \

# Install GFLAGS
install_gflags: dependencies/install/include/gflags/gflags.h

CMAKE_MISSING = "cmake not found in /Applications, nor in the PATH. Install the official version, or from brew"

dependencies/install/include/gflags/gflags.h: dependencies/sources/gflags-$(GFLAGS_TAG)/build_cmake/Makefile
	cd dependencies/sources/gflags-$(GFLAGS_TAG)/build_cmake && \
	$(SET_COMPILER) make -j 4 && make install
	touch $@

dependencies/sources/gflags-$(GFLAGS_TAG)/build_cmake/Makefile: dependencies/sources/gflags-$(GFLAGS_TAG)/CMakeLists.txt
	-mkdir dependencies/sources/gflags-$(GFLAGS_TAG)/build_cmake
	cd dependencies/sources/gflags-$(GFLAGS_TAG)/build_cmake && $(SET_COMPILER) \
	$(CMAKE) -D BUILD_SHARED_LIBS=OFF \
		 -D BUILD_STATIC_LIBS=ON \
	         -D CMAKE_INSTALL_PREFIX=../../../install \
		 -D CMAKE_CXX_FLAGS="-fPIC $(MAC_VERSION)" \
	         ..

dependencies/sources/gflags-$(GFLAGS_TAG)/CMakeLists.txt:
	git clone -b v$(GFLAGS_TAG) https://github.com/gflags/gflags.git dependencies/sources/gflags-$(GFLAGS_TAG)


# Install GLOG.
install_glog: dependencies/install/include/glog/logging.h

dependencies/install/include/glog/logging.h: dependencies/sources/glog-$(GLOG_TAG)/build_cmake/Makefile
	cd dependencies/sources/glog-$(GLOG_TAG)/build_cmake && $(SET_COMPILER) make -j 4 && make install
	touch $@

dependencies/sources/glog-$(GLOG_TAG)/build_cmake/Makefile: dependencies/sources/glog-$(GLOG_TAG)/CMakeLists.txt install_gflags
	-$(MKDIR) dependencies/sources/glog-$(GLOG_TAG)/build_cmake
	cd dependencies/sources/glog-$(GLOG_TAG)/build_cmake && \
	  $(CMAKE) -D CMAKE_INSTALL_PREFIX=../../../install \
                   -D BUILD_SHARED_LIBS=OFF \
                   -D CMAKE_CXX_FLAGS="-fPIC $(MAC_VERSION)" \
                   -D CMAKE_PREFIX_PATH="$(OR_TOOLS_TOP)/dependencies/install" \
	           ..

dependencies/sources/glog-$(GLOG_TAG)/CMakeLists.txt:
	git clone -b v$(GLOG_TAG) https://github.com/google/glog.git dependencies/sources/glog-$(GLOG_TAG)


#Install PROTOBUF
install_protobuf: dependencies/install/bin/protoc

dependencies/install/bin/protoc: dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/build/Makefile
	cd dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/build && $(SET_COMPILER) make -j 4 && make install

dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/build/Makefile: dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/CMakeLists.txt
	-$(MKDIR) dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/build
	cd dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/build && \
	  $(CMAKE) -D CMAKE_INSTALL_PREFIX=../../../../install \
		   -D protobuf_BUILD_TESTS=OFF \
                   -D BUILD_SHARED_LIBS=OFF \
                   -D CMAKE_CXX_FLAGS="-fPIC $(MAC_VERSION)" \
	           ..

dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/CMakeLists.txt:
	git clone https://github.com/google/protobuf.git dependencies/sources/protobuf-$(PROTOBUF_TAG) && cd dependencies/sources/protobuf-$(PROTOBUF_TAG) && git checkout 3d9d1a1




# all: tsp_cplex

%.pb.cc: %.proto
	$(EXDIR)/dependencies/install/bin/protoc --cpp_out . $<

%.o: %.cc %.h
	$(CCC) $(CFLAGS) -c $< -o $@

cplex_vrp.pb.h: cplex_vrp.pb.cc

cplex_result.pb.h: cplex_result.pb.cc

tsp_cplex.o: tsp_cplex.cc \
	cplex_vrp.pb.h \
	cplex_result.pb.h \
	tsptw_data_dt.h
	$(CCC) $(CFLAGS) $(GFLAGS_INC) -c $(LDFLAGS) tsp_cplex.cc -o tsp_cplex.o 

tsp_cplex: tsp_cplex.o cplex_vrp.pb.o cplex_result.pb.o $(CPLEX_TOP)/cpoptimizer/lib/x86-64_linux/static_pic/libcp.a
	$(CCC) $(CFLAGS) -g tsp_cplex.o cplex_vrp.pb.o cplex_result.pb.o -lz -lrt -lpthread \
	-L $(CPLEX_TOP)/cpoptimizer/lib/x86-64_linux/static_pic/libcp.a -L dependencies/install/lib -lprotobuf -L dependencies/install/lib -lgflags \
	-o tsp_cplex $(LDFLAGS)

local_clean:
	rm -f *.pb.cc *.pb.h
	rm *.o

mrproper: local_clean
	rm tsp_cplex
