OBJS = pdaqtool.o pdaq_lib.o
CFLAGS = -Wall

all: $(OBJS)
	$(CC) $^ -o pdaqtool -lm

clean:
	rm -f *.o *~ core pdaqtool
