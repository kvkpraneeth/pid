CC=gcc
CFlags=-Wall -g
Bins=libpid.o libpidtest

all: $(Bins)

libpid.o: pid.c pid.h
	$(CC) $(CFlags) -c pid.c

libpidtest: executable.c pid.o
	$(CC) $(CFlags) -o $@ $^ 

clean:
	rm *.o libpidtest