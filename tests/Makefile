#CFLAGS += -pedantic
CFLAGS += -Wall -Wextra -Werror
CFLAGS += -Wmissing-prototypes
CFLAGS += -Wstrict-prototypes
CFLAGS += -g
CFLAGS += -I..

PROGRAMS = unit

all: ${PROGRAMS}

unit: unit.o mock_i2c.o ../axp192.o

test: unit
	./unit

%.o: %.c
	${CC} -c -o $@ ${CFLAGS} $<

%: %.o
	${CC} -o $@ ${LDFLAGS} $^

*.o: Makefile
*.o: greatest.h
clean:
	rm -f ${PROGRAMS} ${PROGRAMSPP} *.o *.core ../*.o

