# File: makefile
# Makefile for the movmf based clustering code
# Authors: Arindam Banerjee, Suvrit Sra
# (c) The University of Texas at Austin
# 

BASE = /kyb/agbs/suvrit/local
CC = g++
CPPFLAGS	= -Wall -g -O2
LIBNTL = -L$(BASE)/lib -lntl -lgmp
INCNTL = -I$(BASE)/include
LDFLAGS		= $(LIBNTL)
LIBS      =  -lm

SRCS1 = main.cc movmf.cc movmf_base.cc newbessel.cc mat_vec.cc RandomGenerator.cc SparseMatrixDouble.cc DenseMatrixDouble.cc 
OBJS1 = main.o movmf.o movmf_base.o newbessel.o mat_vec.o RandomGenerator.o SparseMatrixDouble.o DenseMatrixDouble.o 

.SUFFIXES: .c .cc .o

.cc.o:
	$(CC) $(CPPFLAGS) $(INCNTL) -c $<

moVMF : $(OBJS1)                      ## normal fskmeans
	$(CC) -o moVMF $(OBJS1) $(LDFLAGS) $(LIBS)

clean :
	rm -f moVMF $(OBJS1) *~
