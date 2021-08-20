CFLAGS=-Wall -Wextra

benchmark: benchmark/main.c
	gcc $(CFLAGS) -O3  -Werror benchmark/main.c -Ilibrtlsdr/include ./librtlsdr.dll -o benchmark

fm: fm/main.cpp
	g++ $(CFLAGS) -O3 -ggdb -std=c++20 fm/main.cpp -Ilibrtlsdr/include ./librtlsdr.dll -o fm

clean:
	rm -rf fm.exe benchmark.exe