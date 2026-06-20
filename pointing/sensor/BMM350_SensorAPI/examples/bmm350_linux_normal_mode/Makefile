CC      ?= cc
CSTD    ?= -std=c99
WARN    ?= -Wall -Wextra -Wpedantic
OPT     ?= -O2

API_LOCATION    ?= ../..
COMMON_LOCATION ?= ..

TARGET  = bmm350_linux_normal_mode

C_SRCS = \
    $(API_LOCATION)/bmm350.c \
    $(COMMON_LOCATION)/common/common.c \
    bmm350_linux_normal_mode.c

INCLUDEPATHS = \
    $(API_LOCATION) \
    $(COMMON_LOCATION)/common

CFLAGS  += $(CSTD) $(WARN) $(OPT) $(addprefix -I,$(INCLUDEPATHS))
LDLIBS  += -li2c

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(C_SRCS)
	$(CC) $(CFLAGS) $(C_SRCS) -o $@ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f $(TARGET)
