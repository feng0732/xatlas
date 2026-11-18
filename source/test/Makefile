# Standalone makefile for the parameterization failure tests.
#
# The project's premake5.lua also defines these targets as
# "test_parameterization_failure" and "test_parameterization_failure_norecompute";
# this makefile is provided for environments where premake5 isn't installed.
#
#   make            # build both test binaries
#   make test       # build and run both
#   make clean

CXX ?= g++
CXXFLAGS ?= -std=c++11 -O2 -g -Wall -Wextra
XATLAS_DIR := ../xatlas

.PHONY: all test clean

all: test_parameterization_failure test_parameterization_failure_norecompute

test_parameterization_failure: test_parameterization_failure.cpp $(XATLAS_DIR)/xatlas.cpp $(XATLAS_DIR)/xatlas.h
	$(CXX) $(CXXFLAGS) -I$(XATLAS_DIR) \
		test_parameterization_failure.cpp $(XATLAS_DIR)/xatlas.cpp \
		-o $@ -pthread

# XA_RECOMPUTE_CHARTS=0 disables the piecewise recompute so an unrecoverable
# chart remains ChartType::Invalid. The #ifndef guards in xatlas.cpp allow the
# define to be overridden on the command line.
test_parameterization_failure_norecompute: test_parameterization_failure_norecompute.cpp $(XATLAS_DIR)/xatlas.cpp $(XATLAS_DIR)/xatlas.h
	$(CXX) $(CXXFLAGS) -DXA_RECOMPUTE_CHARTS=0 -I$(XATLAS_DIR) \
		test_parameterization_failure_norecompute.cpp $(XATLAS_DIR)/xatlas.cpp \
		-o $@ -pthread

test: all
	./test_parameterization_failure
	./test_parameterization_failure_norecompute

clean:
	rm -f test_parameterization_failure test_parameterization_failure_norecompute
