#/bin/bash

set -e

if [ -z $1 ]; then
  COUNT=100
else
  COUNT=$1
fi

FAIL_COUNT=0

while [ $COUNT -ge 0 ]; do
  echo -n "$COUNT ->"
  echo -n "Relaunching..."
  lg-relaunch >/dev/null
  echo -n "sleeping for 10 secs..."
  sleep 10
  echo -n "getting chrome instance..."
  for hostname in $LG_FRAMES; do
    chrome=$(ssh $hostname 'pgrep chrome > /dev/null; echo $?')
    if [ $chrome == "0" ];then
      echo -n "Chrome exists on $hostname.."
    else
      echo -n "No chrome on $hostname"
      let $FAIL_COUNT "FAIL_COUNT+=1"
    fi
  done
  let $COUNT "COUNT-=1"
  echo "FAIL COUNT = $FAIL_COUNT"
done
