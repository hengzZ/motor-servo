TARGET=client-test
#CC=arm-linux-gnueabihf-gcc
CC=gcc
CFLAGS=-I./
LIBS=-lpthread

#_DEPS= alpha_setting.h alpha_motion_control.h am335x_setting.h encoder.h global_setting.h \
	   #modbus.h modbus-private.h modbus-rtu.h modbus-rtu-private.h modbus-tcp.h modbus-tcp-private.h modbus-version.h config.h
#DEPS =$(_DEPS)
#_OBJS= alpha_setting.o alpha_motion_control.o alpha_motion_test.o am335x_setting.o client.o encoder.o \
	   #modbus.o modbus-data.o modbus-rtu.o modbus-tcp.o
#OBJS =$(_OBJS)
DEPS=$(wildcard ./*.h)
SRC=$(wildcard ./*.c)
OBJS=$(patsubst %.c, %.o, $(notdir $(SRC)))


$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

%.o:%.c $(DEPS)
	$(CC) -c $< $(CFLAGS) $(LIBS)


.PHONY:clean

clean:
	rm -rf *.o $(TARGET)
