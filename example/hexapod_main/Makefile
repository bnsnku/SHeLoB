TARGET		= hexapod_main
OBJS		= hexapod_main.o BioloidController.o nuke.o
SRCS		= $(OBJS:.o=.c)
INCLUDEDIRS	+= -I../../include
LIBDIRS		+= -L../../lib
CFLAGS		= $(INCLUDEDIRS) -W -Wall -O2
LDFLAGS		= $(LIBDIRS) -lm -ldxl

CC			= g++

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)
	
.cpp.o:
	$(CC) -c $< $(CFLAGS)

.c.o:
	$(CC) -c $< $(CFLAGS)
	
clean:
	rm -f $(OBJS) $(TARGET)
	@echo "file deleted."

dep:
	gccmakedep $(SRCS)