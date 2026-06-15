#!/usr/bin/env bash

bin=/usr/local/bin/munin

git checkout release && git pull

cd ./pointing
make 

mkdir -p $bin
cp ./outputs/main $bin/pointing

cd ../
cp -r ./tracking $bin/tracking

cp munin-auto-start.service /etc/systemd/system/munin-auto-start.service
