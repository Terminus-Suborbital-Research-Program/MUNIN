#!/usr/bin/env bash

apt install gnuradio python3.13 virtualenv

bin=/usr/local/bin/munin

git checkout release && git pull

#cd ./pointing
make 

mkdir -p $bin
#cp ./outputs/main $bin/pointing
cp ./MUNIN_POINTING $bin/pointing

#cd ../
cp -r ./tracking $bin/tracking

cp munin-auto-start.service /etc/systemd/system/munin-auto-start.service
