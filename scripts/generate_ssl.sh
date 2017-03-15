if [ -f /ssl/rosbridge.crt ] ; then
  echo "Not creating SSL certificate as one alrady exists"
else
  echo "Making new SSL self signed ceritificate for rosbridge under /ssl"
  /usr/bin/openssl req -nodes -new -x509 -extensions v3_req -keyout rosbridge.key -out rosbridge.crt -days 3650 -config /ssl/self_signed_openssl.conf
fi
