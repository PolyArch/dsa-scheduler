
CFLAGS := -O3 -g -Wall -Werror

.phony: all clean

all: gams-server gams-client

gams-server: gams-server.c
	$(CC) $(CFLAGS) -o $@ $<

gams-client: gams-client.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	-rm -f gams-server gams-client
