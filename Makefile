CC=gcc -m32
CFLAGS = -g -O0 -fPIC -m32 -D_GNU_SOURCE -Wall -Werror -Wno-unused-variable -Wno-unused-function -DLOG_USE_COLOR
LD = g++ -m32
LDFLAGS = -Wl,-z,defs -rdynamic -static-libstdc++ -static-libgcc -lc -ldl -lGL -lglut -lX11 -lm -lpthread -shared -nostdlib

BUILD = build

OBJS := $(patsubst %.c,%.o,$(wildcard src/lindbergh/*.c))

all: lindbergh.so libsegaapi.so

lindbergh.so: $(OBJS)
	mkdir -p $(BUILD)
	$(LD) $(OBJS) $(LDFLAGS) $(CFLAGS) -o $(BUILD)/lindbergh.so
	rm -f src/lindbergh/*.o

LIBSEGA_LD=gcc #clang
LIBSEGA_LDFLAGS=-m32 -O0 -g

libsegaapi.so: src/libsegaapi/segaapi.o
	$(LIBSEGA_LD) $(LIBSEGA_LDFLAGS) src/libsegaapi/segaapi.o -L/usr/lib/i386-linux-gnu -lalut -fPIC -shared -o $(BUILD)/libsegaapi.so
	rm -f src/libsegaapi/*.o

clean:
	rm -rf $(BUILD)
