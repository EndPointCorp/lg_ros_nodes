#!/bin/bash
# exit if no arguments
if [[ $# -eq 0 ]] ; then
    exit 0
fi

# Desired layer state
BUILDING_LAYER="$1"
# Earth Instance IDs
EARTH_INSTANCES=$(export DISPLAY=:0 ; xdotool search --name "Earth EC")
# Track if Errors Present
ERRORS_PRESENT=false

# disable logitech keyboard to prevent errors
disable_keyboard () {
	# find keyboard devices
	KEYBOARD=$(grep c52b /sys/bus/usb/devices/*/idProduct)
	for i in $KEYBOARD
	do
		DEVICE=$(echo $i | awk -F/ '{print $(NF-1)}')
		echo "$DEVICE" | sudo tee /sys/bus/usb/drivers/usb/"$1"

	done
}

# Toggle Left Side Menu
toggle_menu () {
        export DISPLAY=:0
        xdotool windowactivate $1
		if [ $2 == "on" ]
		then
			xdotool key  ctrl+alt+b
			for i in {1..7}
			do
				xdotool key Tab
			done
			xdotool key space
		else
			sleep .1
			xdotool key Tab
			xdotool key space
			xdotool key  ctrl+alt+b
		fi
} 

# Capture Current Layer State
capture_current_state ()
{
    for i in $EARTH_INSTANCES
    do
        export DISPLAY=:0
        xwd -display :0 -id $i | convert xwd:- png:- > ~/tmp/42-a_"$i".png
        convert ~/tmp/42-a_"$i".png -crop 20x20+25+1120 ~/tmp/42-a_"$i".png
        convert ~/tmp/42-a_"$i".png txt:- > ~/tmp/42-a_"$i"
    done
}
 Validate 3D Layer On
validate_on () {
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
    echo "${EARTH_ERROR[@]}"
    if [ ${#EARTH_ERROR[@]} -ne 0 ]; then
        for i in "${EARTH_ERROR[@]}"
        do
            export DISPLAY=:0
            xdotool windowactivate $i
            sleep .1
			for i in {1..6}
			do
				xdotool key Tab
				sleep .1
			done
			sleep .1
			for i in {1..8}
            do
                xdotool key Down
            done
			sleep .1
            xdotool key space
			sleep .1
            for i in {1..8}
            do
                xdotool key Up
            done
			sleep .1
        done
    fi
	sleep .3
    unset EARTH_ERROR
    EARTH_ERROR=()
    ERRORS_PRESENT=false
    capture_current_state
    validate_"$BUILDING_LAYER"
}

validate_closed () {
	export DISPLAY=:0
	# Capture point on menu that is white and compare
	for i in $EARTH_INSTANCES
	do
		xdotool windowactivate $i
		xwd -display :0 -id $i | convert xwd:- png:- > ~/tmp/42-a_blank.png
		convert ~/tmp/42-a_blank.png -crop 15x15+120x760 ~/tmp/42-a_blank.png
		convert ~/tmp/42-a_blank.png txt:- > ~/tmp/42-a_blank
		cmp --silent  /home/lg/bash_scripts/menu_open ~/tmp/42-a_blank
        result=$?
		if [ $result == 0 ] ; then
			export DISPLAY=:0
			xdotool windowactivate $i
			xdotool key ctrl+alt+b
		fi
	done
}

disable_keyboard "unbind"
for i in $EARTH_INSTANCES
do
	sleep .1
	toggle_menu $i "on"
done
if [ "$BUILDING_LAYER" == on ]
then
    toggle_on
else
    toggle_off
fi
for i in $EARTH_INSTANCES
do
    toggle_menu $i "off"
done
validate_closed
if [ $HOSTNAME == "42-b" ]
then
	python /home/lg/bash_scripts/repub.py
fi
#rm ~/tmp/42-a*
disable_keyboard "bind"

