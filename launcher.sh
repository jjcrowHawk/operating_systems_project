#! /bin/sh
./sensorSO & PIDSENSOR1=$!
./sensorSO2 & PIDSENSOR2=$!
./sensorSO3 & PIDSENSOR3=$!
echo "Sensors simulation started"
wait $PIDSENSOR1
wait $PIDSENSOR2
wait $PIDSENSOR3
echo "Sensors simulation finished"