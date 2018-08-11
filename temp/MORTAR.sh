#!/bin/bash
sudo chmod 777 /var/run/sdp
head /dev/serial0 > g_result.txt
./main3
