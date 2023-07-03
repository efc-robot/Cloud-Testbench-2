#!/bin/bash
Xvfb :1 -screen 0 1024x768x16 &
export DISPLAY=:1
openbox &
script -q -c "x11vnc -display :1 -usepw -forever " /dev/null
sleep infinity