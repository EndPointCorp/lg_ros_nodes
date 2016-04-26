"""
InfluxDB submission / communication implementations.

"""

import socket

import rospy
import nanotime
from influxdb import InfluxDBClient


class Submitter(object):
    """
    LG Stats - base submission class for sending stats messages.

    """

    def __init__(self):
        raise RuntimeError("Can't instantiate this base class directly.")

    def get_data_for_influx(self, msg):
        raise RuntimeError("Base class method called, not implemented.")

    def write_stats(self, data):
        raise RuntimeError("Base class method called, not implemented.")

    @staticmethod
    def get_timestamp():
        return nanotime.now().nanoseconds()


class InfluxDirect(Submitter):
    """
    Direct connection to InfluxDB database.

    """
    def __init__(self, host=None, port=None, database=None):
        self._client = InfluxDBClient(host=host, port=port, database=database)
        rospy.loginfo("InfluxDB (direct) client initialized (%s:%s/%s)." % (host, port, database))

    @staticmethod
    def get_data_for_influx(msg):
        """
        Prepare data for InfluxDB based on the ROS topic message that
        is sent to the debug topic, it contains all stats pertinent details.
        Direct InfluxDB submitter talks JSON.

        """
        influx_dict = dict(tags=dict(topic=msg.src_topic,
                                     field_name=msg.field_name,
                                     type=msg.type,
                                     value=msg.value),
                           # timestamp may be added here or will be added by the server
                           # "time": "2015-11-10T23:00:00Z",
                           # fields must be of type float
                           fields=dict(value=0.0))
        return influx_dict

    def write_stats(self, data):
        """
        Send data to InfluxDB database.
        The Python Influx library converts the Python dictionary to
        the default *line_protocol* before submitting to Influx.

        """
        self._client.write_points([data])


class InfluxTelegraf(Submitter):
    """
    Handles connection to InfluxDB via Telegraf submission agent.
    Telegraf accepts data through its tcp_listener.
    It accepts text message in the form of Influx line protocol via plain socket.

    Debugging:
    echo "application,application=someapplication1,type=event value=0.0" | nc localhost 8094
        (sent right to the telegraf tcp_listener port)

    Another format possibility is JSON, was not successful with
    sending JSON, still getting parsing errors.

    """
    def __init__(self, host=None, port=None, database=None):
        self.host = host
        self.port = port
        rospy.loginfo("InfluxDB (telegraf-socket) client initialized (%s:%s)." % (host, port))

    @staticmethod
    def get_data_for_influx(msg):
        try:
            influx_str = ("""lg_stats_metric topic_name="%s",field_name="%s",type="%s",value=%s %s""" %
                          (msg.src_topic,
                           msg.field_name,
                           msg.type,
                           float(msg.value),
                           InfluxTelegraf.get_timestamp()))
        except ValueError:
            # value is a string - we could not turn it into float
            influx_str = ("""lg_stats_event topic_name="%s",field_name="%s",type="%s",value="%s" %s""" %
                          (msg.src_topic,
                           msg.field_name,
                           msg.type,
                           msg.value,
                           InfluxTelegraf.get_timestamp()))
        except TypeError:
            return ''

        return influx_str

    def write_stats(self, data):
        """
        Input is a text message in the form of Influx line protocol.

        A socket connection connection and close is performed at each send operation.

        It's impossible to tell whether all data was sent or not.
        """
        rospy.logdebug("Going to write: '%s' to influx" % data)
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_address = (self.host, self.port)
            sock.connect(server_address)
            sock.sendall(data)
            rospy.logdebug("Wrote: '%s' to influx" % data)
        except Exception, ex:
            rospy.logerr("Socket error while sending data '%s' to %s, reason: %s" %
                         (data, server_address, ex))
        finally:
            sock.close()


class InfluxMock(Submitter):
    """
    Mock test class which doesn't submit anything
    and does not report any Connection refused and stuff from the tests.

    """
    def __init__(self, host=None, port=None, database=None):
        self.messages = []
        rospy.loginfo("InfluxDB Mock client initialized ... won't do anything.")

    @staticmethod
    def get_data_for_influx(msg):
        rospy.debug("%s called, received msg: '%s'" % (InfluxMock.__class__.__name__, msg))
        return InfluxTelegraf.get_data_for_influx(msg)

    def write_stats(self, data):
        rospy.logdebug("%s called, received msg: '%s'" % (self.__class__.__name__, data))
        self.messages.append(data)
