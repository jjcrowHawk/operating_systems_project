CC = gcc

all: proyecto  sensorSO sensorSO2 sensorSO3

sensorSO: sensorSO.c
	$(CC) -o $@ $^ -lm

sensorSO2: sensorSO2.c
	$(CC) -o $@ $^ -lm

sensorSO3: sensorSO3.c
	$(CC) -o $@ $^ -lm

proyecto: proyecto.c
	$(CC) -o $@ $^ -lm -lpthread

.PHONY: clean

clean:
	rm -rf proyecto sensorSO sensorSO2 sensorSO3