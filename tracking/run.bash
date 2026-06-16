#!/usr/bin/env bash

bin=/usr/local/bin/munin/tracking

python3 -m venv $bin/.venv
source $bin/.venv/bin/activate
pip install -r $bin/requirements.txt

python3 $bin/tracking.py
