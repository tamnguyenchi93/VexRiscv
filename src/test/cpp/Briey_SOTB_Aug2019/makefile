DEBUG?=no
TRACE?=no
TRACE_INSTRUCTION?=no
TRACE_REG?=no
PRINT_PERF?=no
VGA?=no
TRACE_START=0
ADDCFLAGS += -CFLAGS -pthread 
ADDCFLAGS += -CFLAGS -lSDL2
ADDCFLAGS += -LDFLAGS -lSDL2



ifeq ($(TRACE),yes)
	VERILATOR_ARGS += --trace
	ADDCFLAGS += -CFLAGS -DTRACE --trace-fst
endif
ifeq ($(DEBUG),yes)
	ADDCFLAGS += -CFLAGS "-g3 -O0"
endif
ifneq ($(DEBUG),yes)
	ADDCFLAGS += -CFLAGS "-O3"
endif
ifeq ($(PRINT_PERF),yes)
	ADDCFLAGS += -CFLAGS -DPRINT_PERF
endif

ifeq ($(VGA),yes)
	ADDCFLAGS += -CFLAGS -DVGA
endif
ifeq ($(TRACE_INSTRUCTION),yes)
	ADDCFLAGS += -CFLAGS -DTRACE_INSTRUCTION
endif

ifeq ($(TRACE_REG),yes)
	ADDCFLAGS += -CFLAGS -DTRACE_REG
endif

ADDCFLAGS += -CFLAGS -DTRACE_START=${TRACE_START}

LDFLAGS = -LDFLAGS '-lpthread' 

all: clean compile

run: compile
	./obj_dir/VBriey_SOTB_Aug2019

verilate: ../../../../Briey_SOTB_Aug2019.v
	rm -f Briey_SOTB_Aug2019.v*.bin
	cp Briey_SOTB_Aug2019.v*.bin . | true
	verilator -cc ../../../../Briey_SOTB_Aug2019.v -CFLAGS -std=c++11  ${ADDCFLAGS} --gdbbt ${VERILATOR_ARGS} ${LDFLAGS} -Wno-WIDTH -Wno-UNOPTFLAT --x-assign unique --exe main.cpp
 	
compile: verilate
	make -j  -C obj_dir/ -f VBriey_SOTB_Aug2019.mk VBriey_SOTB_Aug2019
 	
clean:
	rm -f Briey_SOTB_Aug2019.v*.bin
	rm -rf obj_dir
 	
