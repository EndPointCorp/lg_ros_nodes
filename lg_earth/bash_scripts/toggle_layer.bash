#!/bin/bash

selfie="buildings_toggle"
touch "$HOME"/tmp/toggles.log && echo "Start $selfie $(date) " >> "$HOME"/tmp/toggles.log

#DISPLAY=:0 xdotool search "Earth EC" windowfocus
Find_Instances () {
  EARTH_INSTANCES=(""); x=0
  [ "$(DISPLAY=:0 xdotool mousemove --clearmodifiers 1 100 click 1 sleep .1 getwindowfocus)" == "$(DISPLAY=:0 xdotool mousemove --clearmodifiers 1919 100 click 1 sleep .1 getwindowfocus)" ] && \
      x=1920 && earth_x=1920
  while [ ! "$(DISPLAY=:0 xdotool mousemove --clearmodifiers $x 100 click 1 sleep .1 getwindowfocus)" == "${EARTH_INSTANCES[-1]}" ]
    do
      EARTH_INSTANCES+=("$(DISPLAY=:0 xdotool mousemove --clearmodifiers $x 100 click 1 sleep .1 getwindowfocus)")
      ((x+=1080))
      [ "${#EARTH_INSTANCES[@]}" -gt 7 ] && break
    done
  #overwrite them for solos
  [ "$(DISPLAY=:0 xdotool mousemove --clearmodifiers 1 100 click 1 sleep .1 getwindowfocus)" == "$(DISPLAY=:0 xdotool mousemove --clearmodifiers 3839 100 click 1 sleep .1 getwindowfocus)" ] && \
    EARTH_INSTANCES=("$(DISPLAY=:0 xdotool mousemove --clearmodifiers 1 100 click 1 sleep .1 getwindowfocus)") && \
      x=0 && earth_x=0
return 0
}

#open all the side menus
Side_Toggle () {
  for earth in ${EARTH_INSTANCES[@]}; do DISPLAY=:0 xdotool windowfocus "$earth" key ctrl+alt+b; done
}

#take screenshots, crop and convert all files
Parse_Earths () {
  for earth in ${EARTH_INSTANCES[@]}; do Just_XWD $earth &
    done; wait
  for earth in ${EARTH_INSTANCES[@]}; do Prep_Files $earth &
    done; wait
}

# screenshot, takes one instance #test optional name suffix addon
Just_XWD () {
  xwd -display :0 -id "$1" | convert xwd:- png:- > /home/lg/tmp/GE_"$1$2".png
}

##screenshot, crop everything, transform to text
Prep_Files () {
#  local earth=$1
  convert /home/lg/tmp/GE_"$1".png -crop 1x2160+47+0 /home/lg/tmp/GE_icons_"$1".png
  convert /home/lg/tmp/GE_icons_"$1".png txt:- > /home/lg/tmp/GE_icons_"$1".txt
  convert /home/lg/tmp/GE_"$1".png -crop 1x2160+11+0 /home/lg/tmp/GE_tabs_"$1".png
  convert /home/lg/tmp/GE_tabs_"$1".png txt:- > /home/lg/tmp/GE_tabs_"$1".txt
}

#find and open all the tabs
Open_All_Tabs () {
  local instance_x=$earth_x
  for earth in ${EARTH_INSTANCES[@]}; do
    CLOSED_TABS=($(grep '5477AB\|5376AB\|99AECB\|99AFCC\|6282B1' /home/lg/tmp/GE_tabs_"$earth".txt |cut -f1 -d':' |cut -f2 -d','))
    for tab_y in ${CLOSED_TABS[@]}; do
          Click_Earth_x_y $earth $((instance_x+11)) $tab_y
        done
        ((instance_x+=1080))
    done
}

#need to try without focusing or focusing the touch window (lg_wm?); takes earth,x and y
Click_Earth_x_y () {
  DISPLAY=:0 xdotool windowfocus $1 mousemove --clearmodifiers $2 $3 sleep .1 click 1 && \
  echo "...click $*" >> /home/lg/tmp/toggles.log
}

#loads onscreen position of first found color for center and for others
Find_Button () {
  grep -m 1 '6FC43C\|6EC43B\|7DCB76\|7CCB75' /home/lg/tmp/GE_icons_"$1".txt |cut -f1 -d':' |cut -f2 -d','
}

#cleanup of previous temp files
touch /home/lg/tmp/GE_clean && rm /home/lg/tmp/GE_*

Find_Instances && Side_Toggle || exit 1
echo "...found instances: ${EARTH_INSTANCES[*]}" >> /home/lg/tmp/toggles.log
sleep 1
Parse_Earths
#check for a button, open tabs if not found
if ! grep -q '6FC43C\|6EC43B\|7DCB76\|7CCB75' /home/lg/tmp/GE_icons_"$earth".txt |cut -f1 -d':' |cut -f2 -d','
  then Open_All_Tabs && Parse_Earths; fi
#exit if still no button can be found
if ! grep -q '6FC43C\|6EC43B\|7DCB76\|7CCB75' /home/lg/tmp/GE_icons_"$earth".txt |cut -f1 -d':' |cut -f2 -d','
  then Side_Toggle; echo "button not found, closed menus, exiting..."; exit 1; fi
#click them
for earth in ${EARTH_INSTANCES[@]}; do
    Click_Earth_x_y $earth $((earth_x+36)) "$(Find_Button $earth)"
    ((earth_x+=1080))
  done
Side_Toggle
#testing a screenshot delay at the end with menus closed
for earth in ${EARTH_INSTANCES[@]}
  do
    Just_XWD $earth "_closed"&
  done; wait
#TODO validate closed menus
#TODO parse layer states/whatever
echo "done with $selfie at $(date)" >> /home/lg/tmp/toggles.log
exit 0
