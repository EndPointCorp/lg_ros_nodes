#!/bin/bash
# exit if no arguments
if [[ $# -eq 0 ]] ; then
    exit 0
fi

# Desired layer state
BUILDING_LAYER="$1"
# Earth Instance IDs
EARTH_INSTANCES=$(export DISPLAY=:0 ; xdotool search --name "Earth")
# Track if Errors Present
ERRORS_PRESENT=false

# Toggle Left Side Menu
toggle_menus () {
    for i in $EARTH_INSTANCES
    do
        export DISPLAY=:0
        xdotool windowactivate $i
        xdotool key --window $i ctrl+alt+b
    done
}

# Toggle 3D Building Layer
toggle_layer () {
    echo "TOGGLING LAYER"
    for i in $EARTH_INSTANCES
        do
            export DISPLAY=:0
            xdotool windowactivate $i
            xdotool mousemove --window $i 38 1175
            sleep .2
            xdotool click --window $i 1
        done
}

# Capture Current Layer State
capture_current_state ()
{
    echo "CAPTURING CURRENT"
    for i in $EARTH_INSTANCES
    do
        export DISPLAY=:0
        xwd -display :0 -id $i | convert xwd:- png:- > ~/tmp/42-a_"$i".png
        convert ~/tmp/42-a_"$i".png -crop 5x5+33+1151 ~/tmp/42-a_"$i".png
        convert ~/tmp/42-a_"$i".png txt:- > ~/tmp/42-a_"$i"
    done
}

# Validate 3D Layer On
validate_on () {

    echo "VALIDATING ON"
    for i in $EARTH_INSTANCES
    do
        cmp --silent /home/lg/bash_scripts/3d_layer_off ~/tmp/42-a_"$i"
        result=$?
        if [ $result == 0 ]
            then
                EARTH_ERROR[$i]="$i"
                ERRORS_PRESENT=true
        fi
    done
}

# Validate 3D Layer Off
validate_off () {
    echo "VALIDATING OFF"
    for i in $EARTH_INSTANCES
    do
        cmp --silent  /home/lg/bash_scripts/3d_layer_on_1 ~/tmp/42-a_"$i"
        result1=$?
        cmp --silent  /home/lg/bash_scripts/3d_layer_on_2 ~/tmp/42-a_"$i"
        result2=$?
        if [ $result1 == 0 ] || [ $result2 == 0 ]
            then
                EARTH_ERROR[$i]="$i"
                ERRORS_PRESENT=true
            fi
    done
}

# Toggle 3D Layer Off
toggle_off () {
    #toggle_layer
    capture_current_state
    validate_off
    while [ "$ERRORS_PRESENT" = true ]
    do
        fix_errors "$BUILDING_LAYER"
    done
}

# Toggle 3D Layer On
toggle_on ()
{
    #toggle_layer
    capture_current_state
    validate_on
    while [ "$ERRORS_PRESENT" = true ]
    do
        fix_errors "$BUILDING_LAYER"
    done
}

# Fix Errors if Present
fix_errors() {
    echo "FIXING ERRORS"
    echo "${EARTH_ERROR[@]}"
    if [ ${#EARTH_ERROR[@]} -ne 0 ]; then
        for i in "${EARTH_ERROR[@]}"
        do
            export DISPLAY=:0
            xdotool windowactivate $i
            xdotool mousemove --window $i 38 1175
            sleep .2
            xdotool click --window $i 1
        done
    fi
    unset EARTH_ERROR
    EARTH_ERROR=()
    ERRORS_PRESENT=false
    capture_current_state
    validate_"$BUILDING_LAYER"
}

toggle_menus
if [ "$BUILDING_LAYER" == on ]
then
    toggle_on
else
    toggle_off
fi
toggle_menus
sleep 1
if [ $HOSTNAME == "42-a" ]
then
	python /home/lg/bash_scripts/repub.py
fi
rm ~/tmp/42-a*

