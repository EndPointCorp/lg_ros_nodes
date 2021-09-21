#!/bin/bash
# exit if no arguments
if [[ $# -eq 0 ]] ; then
    exit 0
fi

# keep the temp files until next run
rm ~/tmp/42-a*

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
        xdotool windowfocus $1
        xdotool key ctrl+alt+b
} 

# Capture Current Layer State
capture_current_state ()
{
	echo "CAPPING CURRENT STATE"
    for i in $EARTH_INSTANCES
    do
        export DISPLAY=:0
        xwd -display :0 -id $i | convert xwd:- png:- > ~/tmp/42-a_"$i".png
        convert ~/tmp/42-a_"$i".png -crop 5x5+34+1092 ~/tmp/42-a_"$i".png
        convert ~/tmp/42-a_"$i".png txt:- > ~/tmp/42-a_"$i"
#    	sleep .1
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
        cmp --silent /home/lg/bash_scripts/3d_layer_off ~/tmp/42-a_"$i"
        result=$?
        if [ $result != 0 ]
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
        for e in "${EARTH_ERROR[@]}"
        do
            export DISPLAY=:0
            xdotool windowfocus $e
			for i in {1..5}
			do
				xdotool key Tab
			done
			for i in {1..4}
            do
                xdotool key Down
            done
            xdotool key space
            for i in {1..3}
            do
                xdotool key Up
            done
        done
    fi
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
		xdotool windowfocus $i
		xwd -display :0 -id $i | convert xwd:- png:- > ~/tmp/42-a_blank.png
		convert ~/tmp/42-a_blank.png -crop 15x15+120+760 ~/tmp/42-a_blank.png
		convert ~/tmp/42-a_blank.png txt:- > ~/tmp/42-a_blank
		cmp --silent  /home/lg/bash_scripts/menu_open ~/tmp/42-a_blank
        result1=$?
		xwd -display :0 -id $i | convert xwd:- png:- > ~/tmp/42-a_blank2.png
		convert ~/tmp/42-a_blank.png -crop 15x15+125+1360 ~/tmp/42-a_blank2.png
		convert ~/tmp/42-a_blank.png txt:- > ~/tmp/42-a_blank2
		cmp --silent  /home/lg/bash_scripts/menu_open ~/tmp/42-a_blank2
        result2=$?
		if [ $result1 == 0 ] || [ $result2 == 0 ]
		    then
			export DISPLAY=:0
			xdotool windowfocus $i
			xdotool key ctrl+alt+b
		fi
	done
}

disable_keyboard "unbind"
for i in $EARTH_INSTANCES
do
	toggle_menu $i
done
if [ "$BUILDING_LAYER" == on ]
then
    toggle_on
else
    toggle_off
fi
for i in $EARTH_INSTANCES
do
    toggle_menu $i
done
validate_closed
if [ $HOSTNAME == "42-b" ]
then
  python /home/lg/bash_scripts/repub.py
fi
disable_keyboard "bind"
