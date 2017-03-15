if [ -f /ssl/cert/rosbridge.crt ] ; then
  echo "Not creating SSL certificate as one alrady exists"
else
  echo "Making new SSL self signed ceritificate for rosbridge under /ssl/cert"
  mkdir -p /ssl/cert
  cd /ssl/
  /usr/bin/openssl req -nodes -new -x509 -extensions v3_req -keyout rosbridge.key -out rosbridge.crt -days 3650 -config /ssl/self_signed_openssl.conf
  mv rosbridge.crt /ssl/cert/rosbridge.crt
fi
