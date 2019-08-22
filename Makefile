################################################################################
# Command_PPM_Capture Makefile
################################################################################
PROJECT_ROOT=../../
OPT_INC = ${PROJECT_ROOT}/common/make/common_spin.mk
-include ${OPT_INC}

################################################################################
# Key paths and settings
################################################################################
CFLAGS += -std=c++11
ifeq ($(wildcard ${OPT_INC}),)
CXX = g++ ${CFLAGS}
ODIR  = .obj/build${D}
SDIR  = .
MKDIR = mkdir -p
PLATFORM = $(shell uname)
ifeq ($(PLATFORM),Darwin)
OS = mac
endif
endif

OUTPUTNAME = Command_PPM_Capture${D}
OUTDIR = bin

################################################################################
# Dependencies
################################################################################
# Spinnaker deps
SPINNAKER_LIB = -L../../lib -lSpinnaker${D} -lSpinVideo${D}                    \
			${SPIN_DEPS} ${SPINVIDEO_DEPS}                                     \
			$(shell pkg-config --libs   libswscale)                            \
			$(shell pkg-config --libs   libswresample)                         \
			$(shell pkg-config --libs   libavcodec)                            \
			$(shell pkg-config --libs   libavutil)                             \
			$(shell pkg-config --libs   libavformat)

################################################################################
# Master inc/lib/obj/dep settings
################################################################################
_OBJ = Command_PPM_Capture.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))
INC = -I../../include -I/usr/include/spinnaker
ifneq ($(OS),mac)
LIB += -Wl,-Bdynamic ${SPINNAKER_LIB}
LIB += -Wl,-rpath-link=../../lib 
else
LIB += -rpath ../../lib/
LIB += ${SPINNAKER_LIB}
endif
################################################################################
# Rules/recipes
################################################################################
# Final binary
${OUTPUTNAME}: ${OBJ}
	${CXX} -o ${OUTPUTNAME} ${OBJ} ${LIB}
	mv ${OUTPUTNAME} ${OUTDIR}

# Intermediate object files
${OBJ}: ${ODIR}/%.o : ${SDIR}/%.cpp
	@${MKDIR} ${ODIR}
	${CXX} ${CFLAGS} ${INC} -Wall -D LINUX -c $< -o $@

# Clean up intermediate objects
clean_obj:
	rm -f ${OBJ}
	@echo "intermediate objects cleaned up!"

# Clean up everything.
clean: clean_obj
	rm -f ${OUTDIR}/${OUTPUTNAME}
	@echo "all cleaned up!"
