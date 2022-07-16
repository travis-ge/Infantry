#!/bin/bash

cd /home/ares/workspace/infantry/build
./ArmourShoot &

while [ 1 ] ; do
sleep 1
	if [ $(ps -ef|grep ArmourShoot|grep -v grep|wc -l) -eq 0 ] ; then
		echo `date +"%Y-%m-%d %H:%M:%S"` ArmourShoot is offline, try to rebegin... >> /home/ares/logs/check_es.log;
		cd /home/ares/workspace/infantry/build
		./ArmourShoot &
	else
		echo `date +"%Y-%m-%d %H:%M:%S"` ArmourShoot is online... >> /home/ares/logs/check_es.log;
	fi
done
	
