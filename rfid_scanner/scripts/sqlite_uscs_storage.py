#!/usr/bin/env python

import sqlite3
import json
from std_msgs.msg import String
import rospy
from lg_common.helpers import run_with_influx_exception_handler

NODE_NAME = 'rfid_sqlite_uscs_storage'


class MockPub(object):
    def publish(self, *args, **kwargs):
        pass


class RfidStorage(object):
    def __init__(self, database_path, state_set_pub=MockPub(),
                 error_pub=MockPub(), table_name="RFID_USCS_table",
                 rows=None):
        self.table_name = table_name
        self.database_path = database_path
        self.state_set_pub = state_set_pub
        self.error_pub = error_pub
        self._init_database()
        self._close_database()

    def handle_scan(self, msg):
        """
        Read USCS from database and publish it
        """
        self._init_database()
        rfid = msg.data
        rospy.loginfo('got rfid: %s' % rfid)
        uscs = self.get_uscs(rfid)
        if uscs:
            self.state_set_pub.publish(json.dumps(uscs))
        else:
            self.error('Failed to read USCS msg for %s' % rfid)
        self._close_database()

    def handle_set(self, msg):
        """
        Read RFID from USCS message and save it to database
        """
        self._init_database()
        try:
            data = json.loads(msg.data)
            rfid = data.get('rfid', None)
        except Exception:
            rospy.logerr('Error with json passed')
            return

        rospy.loginfo('got data: %s' % data)

        if rfid:
            self.insert_uscs_row(rfid, data)
        else:
            rospy.logerr('No rfig passed in message: %s' % data)

        self._close_database()

    def _init_database(self):
        self.db = sqlite3.connect(self.database_path)
        self.cur = self.db.cursor()
        self.check_if_exists_or_create()

    def check_if_exists_or_create(self):
        uscs_table = self.db.execute(
            "select name from sqlite_master where type='table' and name='%s'"
            % (self.table_name)).fetchone()
        if uscs_table is None:
            self.create_table()
            self.db.commit()

    def create_table(self):
        try:
            self.cur.execute('CREATE table if not exists %s (rfid PRIMARY KEY, uscs)' % self.table_name)
        except sqlite3.OperationalError:
            rospy.logfatal('Error trying to create uscs msg table...')

    def insert_uscs_row(self, rfid, uscs_message):
        insert_query = 'INSERT OR REPLACE INTO %s (rfid, uscs) VALUES (?, ?)' % self.table_name
        self.cur.execute(insert_query, (rfid, json.dumps(uscs_message)))
        self.db.commit()

    def get_uscs(self, rfid):
        self.cur.execute('SELECT * from %s where rfid="%s"' % (self.table_name, rfid))
        row = self.cur.fetchone()
        if row is None:
            self.error('Could not find a matching id for %s' % rfid)
            return

        try:
            return json.loads(row[1])
        except Exception:
            self.error('Cant parse json for stored uscs message. rfid: %s, message: %s' % (rfid, row[1]))

        return None

    def error(self, msg):
        self.error_pub.publish(msg)

    def _close_database(self):
        self.db.close()
        self.cur = None
        self.db = None


def main():
    rospy.init_node(NODE_NAME)

    local_path = rospy.get_param('~database_path', '/home/lg/rfid/rfid_storage.db')
    scan_topic = rospy.get_param('~scan_topic', '/rfid/uscs/scan')
    state_set_topic = rospy.get_param('~state_set_topic', '/state_setter/set_state')
    error_topic = rospy.get_param('~error_topic', '/info/error')
    update_topic = rospy.get_param('~update_topic', '/rfid/uscs/update')

    state_set_pub = rospy.Publisher(state_set_topic, String, queue_size=10)
    error_pub = rospy.Publisher(error_topic, String, queue_size=10)

    d = RfidStorage(local_path, state_set_pub=state_set_pub, error_pub=error_pub)

    rospy.Subscriber(scan_topic, String, d.handle_scan)
    rospy.Subscriber(update_topic, String, d.handle_set)

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
