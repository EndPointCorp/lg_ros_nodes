start_xvfb() {
    #/etc/init.d/xvfb start
    #export DISPLAY=:1.0

    msg="Started Xvfb"

    xinerama="+xinerama"
    if [ "$NO_XINERAMA" = "true" ]; then
        xinerama=""
        msg="$msg, xinerama disabled"
    fi
    randr="+extension RANDR"
    if [ "$NO_RANDR" = "true" ]; then
        randr=""
        msg="$msg, randr disabled"
    fi

    Xvfb :1 -ac $randr $xinerama -nolock \
        -nocursor -screen 0 1920x1080x24 \
        &> /tmp/xvfb_start.log &

    export DISPLAY=:1

    msg="$msg, DISPLAY=$DISPLAY"
    echo $msg
}

start_xvfb
