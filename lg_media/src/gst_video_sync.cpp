#include <glib.h>
#include <gst/gst.h>
#include <gst/gstregistry.h>

// from libgstreamer-plugins-base1.0-dev
#include <gst/video/videooverlay.h>

#include <QApplication>
#include <QTimer>
#include <QWidget>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

#include <unistd.h>
#include <inttypes.h>

#include <mutex>
#include <thread>


const char* DEFAULT_ADDR = "127.0.0.1";
const uint16_t DEFAULT_PORT = 9999;

// If the difference is above the hard threshold, seek.
const gint64 HARD_SYNC_THRESHOLD = GST_SECOND / 1.0;
// If the difference is above the soft threshold, step.
const gint64 SOFT_SYNC_THRESHOLD = GST_SECOND / 60.0;
// If we haven't heard from the master in this time, don't sync.
const gint64 SYNC_TIMEOUT = HARD_SYNC_THRESHOLD / 2.0;


/* Copied from gst-plugins-base/gst/playback/gstplay-enum.h */
typedef enum
{
  GST_PLAY_FLAG_VIDEO = (1 << 0),
  GST_PLAY_FLAG_AUDIO = (1 << 1),
  GST_PLAY_FLAG_TEXT = (1 << 2),
  GST_PLAY_FLAG_VIS = (1 << 3),
  GST_PLAY_FLAG_SOFT_VOLUME = (1 << 4),
  GST_PLAY_FLAG_NATIVE_AUDIO = (1 << 5),
  GST_PLAY_FLAG_NATIVE_VIDEO = (1 << 6),
  GST_PLAY_FLAG_DOWNLOAD = (1 << 7),
  GST_PLAY_FLAG_BUFFERING = (1 << 8),
  GST_PLAY_FLAG_DEINTERLACE = (1 << 9),
  GST_PLAY_FLAG_SOFT_COLORBALANCE = (1 << 10),
  GST_PLAY_FLAG_NATIVE_TEXT = (1 << 11)
} GstPlayFlags;


class SyncVideoApp {
  public:
    SyncVideoApp(char* uri, char* wname, bool master, bool slave, struct sockaddr_in addr);
    int init();
    void quit();
    void play();
    gboolean bus_callback(GstBus *bus, GstMessage *msg);
    void video_changed(GstElement *element);
    GstPadProbeReturn buffer_callback(GstPad *pad, GstPadProbeInfo *info);

  private:
    void pause_for_sync_();
    void resume_();
    void run_udp_thread_();
    bool seek_(gdouble rate, GstFormat format, GstSeekFlags flags, GstSeekType type, guint64 pos);
    bool step(GstFormat format, guint64 amount);
    bool send_pos_(guint64 pos);

    char* uri;
    char* wname;
    bool master;
    bool slave;
    struct sockaddr_in sockaddr;

    std::mutex lock;
    gint64 duration;
    GMainLoop *loop;
    GstElement *player;
    guint64 mypos;
    guint64 remotepos;
    gint64 remotestamp;
    bool holding;
    int sockfd;
    QWidget *window;
    std::thread udp_thread;
};

SyncVideoApp::SyncVideoApp
(char* uri, char* wname, bool master, bool slave, struct sockaddr_in addr)
  : uri(uri), wname(wname), master(master), slave(slave), sockaddr(addr)
{
  this->duration = 0;

  this->loop = NULL;
  this->player = NULL;

  this->mypos = 0;
  this->remotepos = GST_CLOCK_TIME_NONE;
  this->remotestamp = 0;

  this->holding = false;
  this->sockfd = -1;
  this->window = NULL;
}

static gboolean bus_callback_(GstBus *bus, GstMessage *msg, gpointer data) {
  SyncVideoApp *app = (SyncVideoApp *)data;
  return app->bus_callback(bus, msg);
}

static void video_changed_(GstElement *element, gpointer *data) {
  SyncVideoApp *app = (SyncVideoApp *)data;
  app->video_changed(element);
}

static GstPadProbeReturn buffer_callback_(GstPad *pad, GstPadProbeInfo *info, gpointer *data) {
  SyncVideoApp *app = (SyncVideoApp *)data;
  return app->buffer_callback(pad, info);
}

int SyncVideoApp::init() {
  GstState state = GST_STATE_NULL;
  GstBus *bus = NULL;
  GstElement *sink = NULL;
  GstCaps *caps = NULL;
  GstPad *pad = NULL;
  GstStructure *structure = NULL;
  gint width = 0;
  gint height = 0;

  this->window = new QWidget();
  this->loop = g_main_loop_new(NULL, false);
  this->player = gst_element_factory_make("playbin", "sync_player");
  if (!this->player) {
    g_printerr("Failed to create playbin!\n");
    return -1;
  }

  sink = gst_element_factory_make("glimagesink", "glsink");
  g_object_set(this->player, "video-sink", sink, NULL);

  GstPlayFlags playbin_flags = (GstPlayFlags)(
    GST_PLAY_FLAG_VIDEO |
    GST_PLAY_FLAG_VIS |
    GST_PLAY_FLAG_DOWNLOAD |
    GST_PLAY_FLAG_BUFFERING |
    GST_PLAY_FLAG_DEINTERLACE |
    GST_PLAY_FLAG_SOFT_VOLUME |
    GST_PLAY_FLAG_SOFT_COLORBALANCE
  );
  if (!this->slave) {
    playbin_flags = (GstPlayFlags)(playbin_flags | GST_PLAY_FLAG_AUDIO | GST_PLAY_FLAG_TEXT);
  }
  g_object_set(this->player, "flags", playbin_flags, NULL);

  g_object_set(this->player, "uri", this->uri, NULL);

  gst_element_set_state(this->player, GST_STATE_PAUSED);
  gst_element_get_state(this->player, &state, NULL, GST_SECOND * 3);  // Wait for state change.
  g_signal_emit_by_name(this->player, "get-video-pad", 0, &pad, NULL);
  if (!pad) {
    g_printerr("Could not get a video pad!\n");
    return -1;
  }

  caps = gst_pad_get_current_caps(pad);
  structure = gst_caps_get_structure(caps, 0);

  if (!gst_structure_get_int(structure, "width", &width)) {
    g_printerr("Could not query media width!\n");
    return -1;
  }
  if (!gst_structure_get_int(structure, "height", &height)) {
    g_printerr("Could not query media height!\n");
    return -1;
  }
  if (!gst_element_query_duration(this->player, GST_FORMAT_TIME, &this->duration)) {
    g_printerr("Could not query media duration!\n");
    return -1;
  }

  gst_element_set_state(this->player, GST_STATE_NULL);
  g_object_unref(pad);

  bus = gst_pipeline_get_bus(GST_PIPELINE(this->player));
  gst_bus_add_watch(bus, bus_callback_, this);
  gst_object_unref(bus);

  g_signal_connect(this->player, "video-changed", (GCallback)video_changed_, this);

  gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(this->player), this->window->winId());

  this->window->resize(width, height);
  if (this->wname) {
    this->window->setWindowTitle(wname);
  }

  if (this->master || this->slave) {
    if ((this->sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      g_printerr("Failed to create socket, errno %d\n", errno);
      return -1;
    }

    int one = 1;
    if (setsockopt(this->sockfd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one)) < 0) {
      g_warning("Failed to set socket broadcast flag, errno %d\n", errno);
    }
    if (setsockopt(this->sockfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) < 0) {
      g_warning("Failed to set socket reuseaddr flag, errno %d\n", errno);
    }
  }

  if (this->slave) {
    this->window->hide();
    this->udp_thread = std::thread(&SyncVideoApp::run_udp_thread_, this);
    this->udp_thread.detach();
  } else {
    this->play();
  }

  return 0;
}

gboolean SyncVideoApp::bus_callback(GstBus *bus, GstMessage *msg) {
  switch (GST_MESSAGE_TYPE (msg)) {
    case GST_MESSAGE_ERROR: {
      gchar *debug;
      GError *error;

      gst_message_parse_error(msg, &error, &debug);
      g_free(debug);

      g_printerr("Error: %s\n", error->message);
      g_error_free(error);

      this->quit();
      break;
    }
    case GST_MESSAGE_EOS:
      if (!this->seek_(
        1.0,
        GST_FORMAT_TIME,
        GST_SEEK_FLAG_FLUSH,
        GST_SEEK_TYPE_SET,
        0
      )) {
        g_printerr("Loop failed!\n");
        this->quit();
      }

      break;
    case GST_MESSAGE_BUFFERING: {
      gint percent = 0;
      gst_message_parse_buffering(msg, &percent);
      g_debug("buffering at %u percent\n", percent);
      /*
      if (percent < 100) {
        gst_element_set_state(this->player, GST_STATE_PAUSED);
      } else {
        gst_element_set_state(this->player, GST_STATE_PLAYING);
      }
      */
      break;
    }
    default:
      break;
  }

  return true;
}

void SyncVideoApp::video_changed(GstElement *element) {
  GstPad *pad = NULL;
  gint current_stream;

  g_object_get(element, "current-video", &current_stream, NULL);
  g_signal_emit_by_name(element, "get-video-pad", current_stream, &pad, NULL);
  if (!GST_IS_PAD(pad)) {
    g_debug("Could not get pad in video_changed()\n");
    return;
  }

  gst_pad_add_probe(
    pad,
    (GstPadProbeType)(GST_PAD_PROBE_TYPE_BUFFER | GST_PAD_PROBE_TYPE_BLOCK),
    (GstPadProbeCallback)buffer_callback_,
    this,
    NULL
  );
  g_object_unref(pad);
}

GstPadProbeReturn SyncVideoApp::buffer_callback(GstPad *pad, GstPadProbeInfo *info) {
  std::lock_guard<std::mutex> lock(this->lock);
  GstState state = GST_STATE_NULL;
  gint64 mypos = 0;

  gst_element_query_position(this->player, GST_FORMAT_TIME, &mypos);

  this->mypos = mypos;

  if (this->master) {
    this->send_pos_(mypos);
    return GST_PAD_PROBE_PASS;
  }

  if (this->slave && this->remotepos != GST_CLOCK_TIME_NONE) {
    // Slaves sync to the master clock.
    GstStateChangeReturn sret = gst_element_get_state(this->player, &state, NULL, 0);
    if (sret != GST_STATE_CHANGE_SUCCESS || state != GST_STATE_PLAYING) {
      g_debug("player is not playing, pass\n");
      return GST_PAD_PROBE_PASS;
    }

    gint64 remote_age;
    gint64 master_offset;
    gint64 now = g_get_monotonic_time();

    remote_age = (now - this->remotestamp) * 1000;
    master_offset = this->remotepos + remote_age - mypos;

    if (remote_age > SYNC_TIMEOUT) {
      // We haven't heard from the master for a while, don't sync.
      this->remotepos = GST_CLOCK_TIME_NONE;
      return GST_PAD_PROBE_PASS;
    }

    if (master_offset > this->duration - HARD_SYNC_THRESHOLD) {
      g_debug("we looped first, hold on\n");
      this->pause_for_sync_();
    } else if (abs(master_offset) > HARD_SYNC_THRESHOLD) {
      // Hard sync is seeking to the position.
      g_debug("seeking to %ld\n", remotepos);

      if (!this->seek_(
        1.0,
        GST_FORMAT_TIME,
        (GstSeekFlags)(GST_SEEK_FLAG_FLUSH),
        GST_SEEK_TYPE_SET,
        this->remotepos + remote_age
      )) {
        g_printerr("Seek failed!\n");
      }

      return GST_PAD_PROBE_DROP;

    } else if(master_offset > SOFT_SYNC_THRESHOLD) {
      // Soft sync is stepping forwards to the position.
      g_debug("stepping forward by %ld\n", master_offset);

      if (!this->step(GST_FORMAT_TIME, master_offset)) {
        g_printerr("step failed!\n");
      }

      return GST_PAD_PROBE_DROP;

    } else if(master_offset < -SOFT_SYNC_THRESHOLD) {
      // Slave is ahead, pause until master catches up.
      g_debug("holding pattern\n");
      this->pause_for_sync_();
    }
  }

  return GST_PAD_PROBE_PASS;
}

void SyncVideoApp::pause_for_sync_() {
  this->holding = true;
  GstStateChangeReturn sret = gst_element_set_state(this->player, GST_STATE_PAUSED);
  if (sret == GST_STATE_CHANGE_FAILURE) {
    g_printerr("failed to pause for sync\n");
  }
}

void SyncVideoApp::resume_() {
  this->holding = false;
  GstStateChangeReturn sret = gst_element_set_state(this->player, GST_STATE_PLAYING);
  if (sret == GST_STATE_CHANGE_FAILURE) {
    g_printerr("failed to resume sync\n");
  }
}

void SyncVideoApp::play() {
  GstStateChangeReturn sret = gst_element_set_state(this->player, GST_STATE_PLAYING);
  if (sret == GST_STATE_CHANGE_FAILURE) {
    g_printerr("failed to start playing in play()\n");
    this->quit();
  } else {
    this->window->show();
  }
}

void SyncVideoApp::quit() {
  this->window->hide();
  gst_element_set_state(this->player, GST_STATE_NULL);
  g_main_loop_quit(this->loop);

  gst_object_unref(this->player);
  g_main_loop_unref(this->loop);
}

bool SyncVideoApp::seek_(gdouble rate, GstFormat format, GstSeekFlags flags, GstSeekType type, guint64 pos) {
  gboolean ret = gst_element_seek(
    this->player,
    rate,
    format,
    flags,
    type,
    pos,
    GST_SEEK_TYPE_NONE,
    GST_CLOCK_TIME_NONE
  );
  if (!ret) {
    g_printerr("Seek failed\n");
    return false;
  }

  return true;
}

bool SyncVideoApp::step(GstFormat format, guint64 amount) {
  return gst_element_send_event(
    this->player,
    gst_event_new_step(format, amount, 1.0, true, false)
  );
}

bool SyncVideoApp::send_pos_(guint64 pos) {
  unsigned char buffer[sizeof(pos)];
  buffer[0] = (pos >> 56) & 0xff;
  buffer[1] = (pos >> 48) & 0xff;
  buffer[2] = (pos >> 40) & 0xff;
  buffer[3] = (pos >> 32) & 0xff;
  buffer[4] = (pos >> 24) & 0xff;
  buffer[5] = (pos >> 16) & 0xff;
  buffer[6] = (pos >> 8) & 0xff;
  buffer[7] = pos & 0xff;
  ssize_t nsent = 0;
  nsent = sendto(this->sockfd, buffer, sizeof(pos), 0, (struct sockaddr*)&this->sockaddr, sizeof(this->sockaddr));
  return nsent == sizeof(pos);
}

void SyncVideoApp::run_udp_thread_() {
  bind(this->sockfd, (struct sockaddr*)&this->sockaddr, sizeof(this->sockaddr));

  gboolean first_sync = true;
  guint64 pos;
  gint64 stamp;
  unsigned char buffer[8];
  while (true) {
    memset(buffer, 0, sizeof(buffer));
    ssize_t bytes = recv(this->sockfd, buffer, sizeof(buffer), 0);
    if (bytes != sizeof(buffer)) {
      g_printerr("Got %lu bytes, expected %lu\n", bytes, sizeof(buffer));
      continue;
    }
    stamp = g_get_monotonic_time();
    pos = guint64(
      (guint64)(buffer[0]) << 56 |
      (guint64)(buffer[1]) << 48 |
      (guint64)(buffer[2]) << 40 |
      (guint64)(buffer[3]) << 32 |
      (guint64)(buffer[4]) << 24 |
      (guint64)(buffer[5]) << 16 |
      (guint64)(buffer[6]) << 8 |
      (guint64)(buffer[7])
    );

    {
      std::lock_guard<std::mutex> lock(this->lock);

      this->remotepos = pos;
      this->remotestamp = stamp;

      if (this->holding && pos >= this->mypos) {
        g_debug("done holding\n");
        this->resume_();
      }
    }

    if (first_sync) {
      first_sync = false;
      this->play();
    }
  }
}

static void set_factory(const gchar *name, gboolean enable) {
  GstRegistry *registry = NULL;
  GstElementFactory *factory = NULL;

  registry = gst_registry_get();
  if (!registry) {
    g_printerr("Could not get the registry in enable_factory()\n");
    return;
  }

  factory = gst_element_factory_find(name);
  if (!factory) {
    g_printerr("Could not get factory for %s in enable_factory()\n", name);
    return;
  }

  GstRank rank = GST_RANK_NONE;
  if (enable) {
    rank = (GstRank)(GST_RANK_PRIMARY + 1);
  }
  gst_plugin_feature_set_rank(GST_PLUGIN_FEATURE(factory), rank);

  gst_registry_add_feature(registry, GST_PLUGIN_FEATURE(factory));
  g_object_unref(factory);
}


static bool str_to_uint16(const char *str, uint16_t *res) {
  char *end;
  errno = 0;
  intmax_t val = strtoimax(str, &end, 10);
  if (errno == ERANGE || val < 0 || val > UINT16_MAX || end == str || *end != '\0')
    return false;
  *res = (uint16_t) val;
  return true;
}

int main (int argc, char **argv) {
  int ret = -1;
  struct sockaddr_in sockaddr;
  bool argmaster = false;
  bool argslave = false;
  char *argaddr = NULL;
  char *arguri = NULL;
  char *argwname = NULL;
  uint16_t argport = DEFAULT_PORT;
  bool argsoftware = false;

  gst_init (&argc, &argv);

  opterr = 0;
  int c = 0;
  while ((c = getopt (argc, argv, "msda:p:u:w:")) != -1) {
    switch (c)
      {
      case 'm':
        g_debug("I am the master\n");
        argmaster = true;
        break;
      case 's':
        g_debug("I am a slave\n");
        argslave = true;
        break;
      case 'd':
        g_debug("I will disable hardware decoding\n");
        argsoftware = true;
        break;
      case 'a':
        g_debug("Using addr %s\n", optarg);
        argaddr = optarg;
        break;
      case 'p':
        g_debug("Using port %s\n", optarg);
        if (!str_to_uint16(optarg, &argport)) {
          g_printerr("Could not parse port number\n");
          return -1;
        }
        break;
      case 'u':
        g_debug("Using url %s\n", optarg);
        arguri = optarg;
        break;
      case 'w':
        g_debug("Setting window name to %s\n", optarg);
        argwname = optarg;
        break;
      case '?':
        if (optopt == 'a' || optopt == 'p' || optopt == 'u')
          g_printerr("Option -%c requires an argument.\n", optopt);
        else if (isprint (optopt))
          g_printerr("Unknown option `-%c'.\n", optopt);
        else
          g_printerr("Unknown option character `\\x%x'.\n", optopt);
        return -1;
      default:
        abort();
      }
  }

  if (arguri == NULL) {
    g_printerr("Setting content uri with -u is required!\n");
    return -1;
  }

  if (argmaster && argslave) {
    g_printerr("Can't be both master and slave!\n");
    return -1;
  }


  if (argsoftware) {
    set_factory("vaapidecodebin", false);
    set_factory("vaapipostproc", false);
    set_factory("vaapisink", false);
  }

  memset(&sockaddr, 0, sizeof(sockaddr));
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_port = htons(argport);
  if (argaddr != NULL) {
    sockaddr.sin_addr.s_addr = inet_addr(argaddr);
  } else {
    sockaddr.sin_addr.s_addr = inet_addr(DEFAULT_ADDR);
  }

  QApplication app(argc, argv);
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

  SyncVideoApp sync(
    arguri,
    argwname,
    argmaster,
    argslave,
    sockaddr
  );
  ret = sync.init();
  if (ret != 0) {
    g_printerr("Failed to initialize the player\n");
    return ret;
  }

  ret = app.exec();

  sync.quit();

  return ret;
}
