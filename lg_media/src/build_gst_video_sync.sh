g++ -fPIC -Wall gst_video_sync.cpp -o gst_video_sync \
  -DG_LOG_DOMAIN="\"gst_video_sync\"" \
  $(pkg-config --cflags --libs gstreamer-1.0) \
  $(pkg-config --cflags --libs gstreamer-video-1.0) \
  $(pkg-config --cflags --libs gstreamer-plugins-base-1.0) \
  $(pkg-config --cflags --libs Qt5GStreamerUi-1.0)
