# lg\_screenshot

ROS node for making screenshots

## Overview

This ROS node allows for making screenshots by publishing a URL on a ROS
topic. In return you will get bas64 representation of the screenshot
that's ready for being consumed by ROSlibjs.

## Topic

- `/screenshot/get`: `lg_screenshot/GetScreenshot`:
 - `url`: `string` - url to make screenshot of
 - `page_width`: `int32` - width of the page to return in screenshot
   response

- `/screenshot/screenshot`
 - `url`: `string` - url of the screenshot that matches the other
   attribure in this message
 - `base64`: `string` - base64 representation of the screenshot

## Params

- `binary`: `default: phantomjs` - absolute path to phantomjs binary
- `script`: `default: screenshots.js` - relative path to phantomjs
  script for making screenshots
- `delay`: `default: 250` - delay between event when page reports its
  complete and DOM modification methods
- `user_agent`: `default: DEFAULT_UA`
