CC         = g++
SRCFILES   = tty.cpp message_queue.cpp radar.cpp sensor.cpp
CFLAGS     = -g 
CPPFLAGS   = -std=c++17
LDFLAGS    = -pthread -lrt
OUTPUT     = radar sensor

DEPDIR     = .dep
COMMON_SRC = tty.cpp message_queue.cpp
COMMON_OBJ = $(patsubst %.cpp,%.o,$(COMMON_SRC))

DEPFLAGS = -MT $@ -MMD -MP -MF .dep/$*.d
CFLAGS += $(DEPFLAGS)

.PHONY: clean all
all: $(OUTPUT)

radar: radar.o $(COMMON_OBJ)
	g++  $^ -o $@ $(LDFLAGS)

sensor: sensor.o $(COMMON_OBJ)
	g++  $^ -o $@ $(LDFLAGS)

clean:
	rm -rf $(DEPDIR)
	rm -f *.o
	rm -f $(OUTPUT)

DEPFILES := $(OBJFILES:%.o=$(DEPDIR)/%.d)
$(DEPFILES):
include $(wildcard $(DEPFILES))

$(shell mkdir -p $(DEPDIR))
