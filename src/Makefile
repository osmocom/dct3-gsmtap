CFLAGS=-Wall -Werror

all: dct3-gsmtap

dct3-gsmtap: main.o
	$(CC) $(LDFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $^

clean:
	@rm -f dct3-gsmtap *.o

