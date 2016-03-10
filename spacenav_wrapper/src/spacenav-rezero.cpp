/*
** Copyright 2013 Google Inc.
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**    http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
** spacenav-rezero
** Sends a rezero event to a Space Navigator.
**
** hidapi is redistributed under the "original" license, available at:
** https://github.com/signal11/hidapi
*/

#include <stdio.h>
#include <stdlib.h>

//#include "hidapi.h"
#include <spacenav_wrapper/hidapi.h>

int main( int argc, char* argv[] )
{
  unsigned char buf[2];
  hid_device *handle;
  int res;

  // Ruggedized Space Navigator: 256f c641
  handle = hid_open( 0x256f, 0xc641, NULL );

  // Space Navigator: 046d c626
  if ( handle == NULL ) {
    handle = hid_open( 0x046d, 0xc626, NULL );
  }

  // Also try legacy device c628
  if ( handle == NULL ) {
    handle = hid_open( 0x046d, 0xc628, NULL );
  }

  if ( handle == NULL ) {
    fprintf( stderr, "%s: Could not open HID device (got sudo?)\n", argv[0] );
    exit( EXIT_FAILURE );
  }

  buf[0] = 0x07; // This proprietary(?) feature report will rezero the device.
  buf[1] = 0x00;
  res = hid_send_feature_report( handle, buf, sizeof(buf) );

  if ( res != sizeof(buf) ) {
    fprintf( stderr, "%s: Write failed\n", argv[0] );
    exit( EXIT_FAILURE );
  }

  hid_close( handle );

  return EXIT_SUCCESS;
}
