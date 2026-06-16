#!/usr/bin/env bash

apt install vim gnuradio python3.13 virtualenv gcc make libgpiod3 libgpiod-dev

bin=/usr/local/bin/munin

git checkout release && git pull

cd ./pointing
make 

mkdir -p $bin
\cp ./outputs/main $bin/pointing
#cp ./MUNIN_POINTING $bin/pointing

cd ../
\cp -r ./tracking $bin/tracking
\cp ./requirements.txt $bin/tracking/requirements.txt
chmod +x $bin/tracking/run.bash

\cp munin-auto-tracking.service /etc/systemd/system/munin-auto-tracking.service
\cp munin-auto-pointing.service /etc/systemd/system/munin-auto-pointing.service

systemctl enable munin-auto-tracking.service munin-auto-pointing.service
