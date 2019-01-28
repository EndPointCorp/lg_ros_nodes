#!/bin/bash

# exit if no arguments
if [[ $# -eq 0 ]] ; then
	exit 0
fi

#HOST
host=$(hostname)
# Starting Positions for Validation
A_START_X_VAL=1946
B_START_X_VAL=26

#desired layer state
BUILDING_LAYER="$1"

# Error Tracking
A_ERROR=()
B_ERROR=()
errors_present=false
# Coordinates for toggle
declare -A toggle_button=( ["42-a-0"]="1958 1151" ["42-a-1"]="3038 1151" ["42-a-2"]="4118 1151" ["42-b-0"]="40 1150" ["42-b-1"]="1120 1150" ["42-b-2"]="2200 1150" ["42-b-3"]="3280 1150" )


toggleMenus ()
{
   	if [ "$host" == "42-a" ]
	then

	 	# Open Menus 42-a
        export DISPLAY=:0;
		 xdotool mousemove 2460 960 ; xdotool click 1; xdotool key ctrl+alt+b ; sleep .2;
		 xdotool mousemove_relative 1080 0; xdotool click 1; xdotool key ctrl+alt+b; sleep .2;
		 xdotool mousemove_relative 1080 0; xdotool click 1; xdotool key ctrl+alt+b; 
	else
		# Open Menus 42-b
        export DISPLAY=:0;xdotool mousemove 1600 960 ;  xdotool click 1; xdotool key ctrl+alt+b; sleep .2
		xdotool mousemove 0420 1169;  xdotool click 1; xdotool key ctrl+alt+b; sleep .2 
		xdotool mousemove 2580 1169; xdotool click 1; xdotool key ctrl+alt+b; sleep .2
		xdotool mousemove 3660 1169; xdotool click 1; xdotool key ctrl+alt+b;
	fi	
}



toggle_layer ()
{
	if [ "$host" == "42-a" ]
	then
		# Toggle 42-a 3D Imagery
		export DISPLAY=:0; xdotool mousemove 1958 1151;  xdotool click 1; sleep .1; xdotool mousemove_relative 1080 0; xdotool click 1; sleep .1; xdotool mousemove_relative 1080 0; xdotool click 1; sleep .1; xdotool mousemove_relative 540 0; xdotool click 1 
	else
		# Toggle 42-b 3D Imagery
		export DISPLAY=:0; xdotool mousemove 40 1150;   xdotool click 1; sleep .1; xdotool mousemove_relative 1080 0;  xdotool click 1; sleep .1; xdotool mousemove_relative 1080 0;  xdotool click 1; sleep .1; xdotool mousemove_relative 1080 0;  xdotool click 1
	fi
}

capture_initial_state ()
{
	if [ "$host" == "42-a" ]
	then
		#screenshot 42-a
		xwd -display :0 -root | convert xwd:- png:- > ~/tmp/42-a_initial.png
		# crop for current state 
		convert ~/tmp/42-a_initial.png -crop 25x25+"$((A_START_X_VAL))"+1139 ~/tmp/initial_layer_state.png
		img2png ~/tmp/initial_layer_state.png &>/dev/null
		# convert to text for comparisson
	else
		# screenshot 42-b
 		xwd -display :0 -root | convert xwd:- png:- > ~/tmp/42-b_initial.png
		#crop for current state	
		convert ~/tmp/42-b_toggle.png -crop 25x25+"$((B_START_X_VAL))"+1139 ~/tmp/42-b_"$i".png
		img2png ~/tmp/initial_layer_state.png &>/dev/null
	fi
}

capture_current_state ()
{ 
	if [ "$host" == "42-a" ]
	then
		# screenshot 
		xwd -display :0 -root | convert xwd:- png:- > ~/tmp/42-a_toggle.png
		#crop for each screen in 42-a
		for i in 0 1 2
		do
			convert ~/tmp/42-a_toggle.png -crop 25x25+"$((A_START_X_VAL+(1080*$i)))"+1139 ~/tmp/42-a_"$i".png
			img2png ~/tmp/42-a_"$i".png &>/dev/null
			#img2txt ~/tmp/42-a_"$i".png > ~/tmp/42-a_"$i"
		done
	else
		# screenshot 42-b
		xwd -display :0 -root | convert xwd:- png:- > ~/tmp/42-b_toggle.png
		# crop for each screen in 42-b
		for i in 0 1 2 3
		do
			convert ~/tmp/42-b_toggle.png -crop 25x25+"$((B_START_X_VAL+(1080*$i)))"+1139 ~/tmp/42-b_"$i".png
			img2png ~/tmp/42-b_"$i".png &>/dev/null
			#img2txt ~/tmp/42-b_"$i".png > ~/tmp/42-b_"$i"
		done
	fi
}


fix_errors ()
{
	if [ "$host" == "42-a" ]
	then
		if [ ${#A_ERROR[@]} -ne 0 ]; then
			for i in  "${A_ERROR[@]}"
			do
				export DISPLAY=:0; xdotool mousemove ${toggle_button[$i]};  xdotool click 1; sleep .3
			done
			unset A_ERROR
			A_ERROR=()
		fi
	else
		if [ ${#B_ERROR[@]} -ne 0 ]; then
			for i in "${B_ERROR[@]}"
			do
				export DISPLAY=:0; xdotool mousemove ${toggle_button[$i]};  xdotool click 1; sleep .3
			done
			unset B_ERROR
			B_ERROR=()
		fi
	fi
	errors_present=false
	capture_current_state
	validate_"$BUILDING_LAYER"
}

validate_on ()
{
	if [ "$host" == "42-a" ]
	then
		# check 42-a state
		for i in 0 1 2
		do 
			cmp --silent ~/tmp/3d_layer_off.png ~/tmp/42-a_"$i".png
			result=$?
			if [ $result == 0 ]
			then
				A_ERROR[$i]="42-a-$i"
				errors_present=true
			fi
		done
	else
		# check 42-b state
		for i in 0 1 2 3
		do
			cmp --silent ~/tmp/3d_layer_off.png ~/tmp/42-b_"$i".png
			result=$?
			if [ $result == 0 ]
			then
				B_ERROR[$i]="42-b-$i"
				errors_present=true
			fi
		done
	fi
}

validate_off ()
{
	if [ "$host" == "42-a" ] 
	then
		# check 42-a state	
		for i in 0 1 2
		do 
			cmp --silent  ~/tmp/3d_layer_on_1.png ~/tmp/42-a_"$i".png
			result1=$?
			cmp --silent  ~/tmp/3d_layer_on_2.png ~/tmp/42-a_"$i".png
			result2=$?
			if [ $result1 == 0 ] || [ $result2 == 0 ]
			then
					A_ERROR[$i]="42-a-$i"
					errors_present=true
			fi
		done
	else
		# check 42-b state
		for i in 0 1 2 3
		do
			cmp --silent ~/tmp/3d_layer_on_1.png ~/tmp/42-b_"$i".png
			result1=$?
			cmp --silent ~/tmp/3d_layer_on_2.png ~/tmp/42-b_"$i".png
			result2=$?
			if [ $result1 == 0 ] || [ $result2 == 0 ]
			then
					B_ERROR[$i]="42-b-$i"
					errors_present=true
			fi
		done
	fi

}

toggle_off ()
{
	#toggle_layer
	capture_current_state
	validate_off	
	while [ "$errors_present" = true ]
	do
		fix_errors "$BUILDING_LAYER"
	done	
}

toggle_on ()
{
	#toggle_layer
	capture_current_state
	validate_on
	while [ "$errors_present" = true ]
	do
		fix_errors "$BUILDING_LAYER"
	done
}

toggleMenus
capture_initial_state
if [ "$BUILDING_LAYER" == on ]
then
	toggle_on
else
	toggle_off
fi
toggleMenus

