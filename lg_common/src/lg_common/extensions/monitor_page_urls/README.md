## Pages urls monitor

This extension watch all page open events
and checks the urls among the list of allow regexps.

### Testing

1. #### Manual test with local browser:

   First check Yandex. Yandex don't remove unknow get arguments,
   but uses the proxy page with redirection when you go outside.

       google-chrome \
        --user-data-dir=/tmp/chrome \
        --no-first-run \
        --disable-translate \
        --ignore-gpu-blacklist \
        --load-extension=/opt/ep/lg_ros_nodes/lg_common/src/lg_common/extensions/monitor_page_urls \
        "https://yandex.ru/search/?text=End+Point+Liquid+Galaxy&allowed_urls=endpoint.com&allowed_urls=chrome:&allowed_urls=yandex"

 Check Google. Google don't use proxy page
 but removes unknown get arguments.
 Another problem with google: after couple of
 requests with unknown getargs it will ask
 you for captcha.

       google-chrome \
        --user-data-dir=/tmp/chrome \
        --no-first-run \
        --disable-translate \
        --ignore-gpu-blacklist \
        --load-extension=/opt/ep/lg_ros_nodes/lg_common/src/lg_common/extensions/monitor_page_urls \
        "http://www.google.com/search?q=End+Point+Liquid+Galaxy&allowed_urls=endpoint.com&allowed_urls=chrome:&allowed_urls=google.com"

  Both of them should show you the results and allow you to go to endpoint.com site.
  If you try to open youtube or wikipedia extensions
  should redirect you back to yandex/google search
  results.

2. #### Test with ros infrastructure:

  Publish `page_monitor_yandex.json` and `page_monitor_google.json`.

  Extension should works the same way as it was
  manual test. Both scenes configured to use
  mirroring so you should be able to touch links.

  `adhoc_browser_pool` should translate `allowed_urls`
  array from json message into get arguments.
