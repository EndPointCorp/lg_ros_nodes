#!/bin/bash
# exit if no arguments
if [[ $# -eq 0 ]] ; then
    exit 0
fi
rm ~/tmp/42-a*
# '''Simple Script to toggle all, no bugs found'''
#for window in $(xdotool search --name "Earth EC")
#    do
#        # select instance
#        DISPLAY=:0 xdotool windowfocus $window
#        # open menu
#        DISPLAY=:0 xdotool key ctrl+alt+b
#        # Tab to Layers
#        for i in {1..5}; do DISPLAY=:0 xdotool key Tab; done
#        # Down to 3D Buildings layer
#        for i in {1..4}; do DISPLAY=:0 xdotool key Down; done
#        # select/deselect layer
#        DISPLAY=:0 xdotool key space
#        # Up again so the script will keep running
#        for i in {1..3}; do DISPLAY=:0 xdotool key Up; done
#        # close the menu
#        DISPLAY=:0 xdotool key ctrl+alt+b
#    done

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
#	sleep .1
#        if [ $2 == "on" ]
#		then
#			xdotool key ctrl+alt+b
#			sleep .1
#			for i in {1..5}
#			do
#				xdotool key Tab
#				sleep .1
#			done
#			xdotool key space
#			sleep .1
#		else
#			xdotool key Tab
#			sleep .1
##			xdotool key space
##			sleep .1
#			xdotool key  ctrl+alt+b
#			sleep .1
#	fi
} 

# Capture Current Layer State
capture_current_state ()
{
	echo "CAPPING CURRENT STATE"
    for i in $EARTH_INSTANCES
    do
        export DISPLAY=:0
        xwd -display :0 -id $i | convert xwd:- png:- > ~/tmp/42-a_"$i".png
        convert ~/tmp/42-a_"$i".png -crop 5x5+34+1058 ~/tmp/42-a_"$i".png
        convert ~/tmp/42-a_"$i".png txt:- > ~/tmp/42-a_"$i"
    done
}

# Validate 3D Layer On
validate_on () {
	echo "VALIDATING ON"
    for i in $EARTH_INSTANCES
    do
        cmp --silent /home/lg/bash_scripts/3d_layer_on_1 ~/tmp/42-a_"$i"
        result1=$?
        cmp --silent /home/lg/bash_scripts/3d_layer_on_2 ~/tmp/42-a_"$i"
        result2=$?
        if [ $result1 != 0 ] && [ $result2 != 0 ]
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
        cmp --silent /home/lg/bash_scripts/3d_layer_on_1 ~/tmp/42-a_"$i"
        result1=$?
        cmp --silent /home/lg/bash_scripts/3d_layer_on_2 ~/tmp/42-a_"$i"
        result2=$?
        if [[ $result1 == 0 ]] || [[ $result2 == 0 ]]
            then
                EARTH_ERROR[$i]="$i"
                ERRORS_PRESENT=true
            fi
      	sleep .1
#    do
#        cmp --silent /home/lg/bash_scripts/3d_layer_off ~/tmp/42-a_"$i"
#        result=$?
#        if [ $result == 0 ]
#            then
#                EARTH_ERROR[$i]="$i"
#                ERRORS_PRESENT=true
#        fi
#	sleep .1
#    done
    done
}

# Toggle 3D Layer Off
toggle_off () {
    #toggle_layer
    capture_current_state
    sleep .3
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
    sleep .3
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
            sleep .1
			for i in {1..5}
			do
				xdotool key Tab
				sleep .1
			done
	  sleep .1
	    for i in {1..4}
            do
                xdotool key Down
        	sleep .1
	    done
            xdotool key space
            for i in {1..3}
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
    sleep .3
    validate_"$BUILDING_LAYER"
}

validate_closed () {
	export DISPLAY=:0
	# Capture point on menu that is white and compare
	for i in $EARTH_INSTANCES
	do
		xdotool windowfocus $i
		sleep .1
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
		if [ $result1 == 0 ] && [ $result2 == 0 ]
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
	sleep .1
	toggle_menu $i # "on"
done
if [ "$BUILDING_LAYER" == on ]
then
    toggle_on
else
    toggle_off
fi
for i in $EARTH_INSTANCES
do
    toggle_menu $i # "off"
done
validate_closed
if [ $HOSTNAME == "42-b" ]
then
  python /home/lg/bash_scripts/repub.py
fi
#rm ~/tmp/42-a*
disable_keyboard "bind"

