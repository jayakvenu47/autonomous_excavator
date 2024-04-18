#!/bin/bash
while :
do
    echo $(((RANDOM%10)+1)),$(((RANDOM%10)+1)),$(((RANDOM%10)+1)) > /tmp/excavator/config.txt
    sleep 0.3
done