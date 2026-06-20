#!/usr/bin/env bash


TARGET_FILE="/boot/firmware/config.txt"

STRING0="dtparam=i2c_arm=on"
STRING1="i2c_arm_baudrate="
STRING2="10000"

if [ ! -f "$TARGET_FILE" ]; then
    echo "Error: File '$TARGET_FILE' does not exist."
    exit 1
fi

if grep -q "$STRING0" "$TARGET_FILE"; then 
    sed -i -E "s/#$STRING0/$STRING0/" "$TARGET_FILE"
fi

if grep -q "$STRING1" "$TARGET_FILE"; then
    sed -i -E "s/(.*)$STRING1[^$STRING2].*/\1$STRING1$STRING2/" "$TARGET_FILE"
else
    echo "'$STRING1' not found. Appending to the end of the file."
    echo -e "\n$STRING1$STRING2" >> "$TARGET_FILE"
fi