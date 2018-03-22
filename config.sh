#!/bin/bash

for kp in "10.0" "11.0" "12.0" "13.0" "14.0" "15.0" "16.0" "17.0" "18.0" "19.0" "20.0"
do
#  for kd in "9.5" "10" "10.5" "11" "11.5" "12" "12.5"
#  do
    sed -i "" -e "s/int32_t y_s = (int)([0-9]*.[0-9]*/int32_t y_s = (int)($kp/g" "src/main.cpp" 
    make &> /dev/null
    cp BUILD/ES_CW2_Starter.bin /Volumes/NODE_F303K8/ &> /dev/null
    sleep 10
    screen -ls | grep Detached | cut -d. -f1 | awk '{print $1}' | xargs kill
    screen -dm -L /dev/cu.usbmodem1423 9600
    sleep 60
    printf "kp:%s, kd:%s\n" $kp $kd 
    tail -1 "screenlog.0"
#  done
done
