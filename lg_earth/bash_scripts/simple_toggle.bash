#!/bin/bash

# Loop over the instance numbers of Google Earths
for window in $(xdotool search --name "Earth EC")
    do
        # select instance
        DISPLAY=:0 xdotool windowfocus $window
        # open menu
        DISPLAY=:0 xdotool key ctrl+alt+b
        # Tab to Layers
        for i in {1..5}; do DISPLAY=:0 xdotool key Tab; done
        # Down to 3D Buildings layer
        for i in {1..4}; do DISPLAY=:0 xdotool key Down; done
        # select/deselect layer
        DISPLAY=:0 xdotool key space
        # Up again so the script will keep running
        for i in {1..3}; do DISPLAY=:0 xdotool key Up; done
        # close the menu
        DISPLAY=:0 xdotool key ctrl+alt+b
    done

