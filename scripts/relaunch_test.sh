#/bin/bash

set -e

if [ -z $1 ]; then
  COUNT=100
else
  COUNT=$1
fi

FAIL_COUNT=0

while [ $COUNT -ge 0 ]; do
  echo "$COUNT relaunches to go"
  let $COUNT "COUNT-=1"
  echo -n "Relaunching..."
  lg-relaunch >/dev/null
  echo -n "sleeping for 10 secs..."
  sleep 10
  echo -n "getting chrome instance..."
  chrome=$(ssh 42-a 'pgrep chrome > /dev/null; echo $?')
  if [ $chrome == "0" ];then
    echo "Chrome exists!"
  else
    echo "No chrome on 42-a"
    let $FAIL_COUNT "FAIL_COUNT+=1"
  fi
done

echo "FAIL COUNT = $FAIL_COUNT"
