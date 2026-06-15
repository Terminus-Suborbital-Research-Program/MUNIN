#!/usr/bin/env bash

bin=/usr/local/bin/munin

git checkout release && git pull

cd ./pointing
make .

mkdir $bin
cp -r ./output/build $bin/pointing

cd ../
cp -r ./tracking $bin/tracking


