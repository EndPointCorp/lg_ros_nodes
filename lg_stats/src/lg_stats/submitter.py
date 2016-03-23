"""
InfluxDB submission / communication implementations.

"""


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


class InfluxDirect(Submitter):
    """
    Direct connection to InfluxDB database.

    """
    def __init__(self, host=None, port=None, database=None):
        self._client = InfluxDBClient(host=host, port=port, database=database)

    @staticmethod
    def get_data_for_influx(msg):
        """
        Prepare data for InfluxDB based on the ROS topic message that
        is sent to the debug topic, it contains all stats pertinent details.
        Direct InfluxDB submitter talks JSON.

        """
        influx_dict = dict(measurement=msg.src_topic,
                           tags=dict(application=msg.application,
                                     field_name=msg.field_name,
                                     type=msg.type,
                                     value=msg.value),
                           # timestamp may be added here or will be added by the server
                           #"time": "2015-11-10T23:00:00Z",
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

    """
    def __init__(self):
        pass

    def write_stats(self, data):
        pass
