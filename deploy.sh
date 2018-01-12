#!/bin/bash
HOST='10.35.12.2'

if [ $# -ne 1 ] ; then
    echo usage: ./deploy.sh DIRECTORY
    echo -e "DIRECTORY is the directory in which the robot code binary is located"
    echo -e "\nThe roboRIO admin account has no password. Just press <Enter> at the password prompts."
    exit 1
fi

ssh lvuser@$HOST "rm -f /home/lvuser/FRCUserProgram; killall -q netconsole-host || :"
scp $1/FRCUserProgram lvuser@$HOST:/home/lvuser
ssh admin@$HOST "setcap 'cap_sys_nice=pe' /home/lvuser/FRCUserProgram;"
ssh lvuser@$HOST ". /etc/profile.d/natinst-path.sh; chmod a+x /home/lvuser/FRCUserProgram; /usr/local/frc/bin/frcKillRobot.sh -t -r; /usr/local/frc/bin/frcKillRobot.sh -t -r;"
