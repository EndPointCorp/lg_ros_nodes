#!/usr/bin/env python3

import sqlite3
import json
import rospy
from std_msgs.msg import String
from lg_common.helpers import run_with_influx_exception_handler

NODE_NAME = 'sqlite_rfid_storage'
from lg_common.logger import get_logger
logger = get_logger(NODE_NAME)


class MockPub(object):
    def publish(self, *args, **kwargs):
        pass


class RfidStorage(object):
    def __init__(self, database_path, state_set_pub=MockPub(),
                 error_pub=MockPub(), table_name="RFID_table", rows=None):
        if rows is None:
            rows = ['rfid', 'runway_card', 'mode', 'kiosk_url', 'display_url']
        self.rows = rows
        self.table_name = table_name
        self.database_path = database_path
        self.state_set_pub = state_set_pub
        self.error_pub = error_pub
        self._init_database()
        self._close_database()

    def handle_scan(self, msg):
        self._init_database()
        rfid = msg.data
        logger.debug('got rfid: %s' % rfid)
        row = self.get_row_as_dict(rfid)
        self.state_set_pub.publish(json.dumps(row))
        self._close_database()

    def handle_set(self, msg):
        self._init_database()
        try:
            data = json.loads(msg.data)
        except Exception:
            logger.exception('Error with json passed')
            return
        logger.debug('got data: %s' % data)
        self.insert_row(data)
        self._close_database()

    def _init_database(self):
        self.db = sqlite3.connect(self.database_path)
        self.cur = self.db.cursor()
        self.check_if_exists_or_create()

    def check_if_exists_or_create(self):
        table = self.db.execute(
            "select name from sqlite_master where type='table' and name='%s'"
            % (self.table_name)).fetchone()
        if table is None:
            self.create_table()
            self.db.commit()

    def create_table(self):
        try:
            self.cur.execute('CREATE table %s (%s)' % (self.table_name,
                                                       ','.join(self.rows)))
        except sqlite3.OperationalError:
            logger.fatal('Error trying to create table...')

    def insert_row(self, new_row):
        """
        Makes sure the row is valid, then turns the dict into an
        array, deletes the row based on the rfid (in case it exists),
        and inserts it.
        """
        # check validity, will throw exception if invalid
        self.check_row_validity(new_row)
        insert_row = []
        row_string = ''
        # convert the dict into a list, in the proper order
        # by iterating over our rows
        for row in self.rows:
            insert_row.append(new_row[row])
            row_string += '%s = ? ' % row
        # assuming row[0] is the key, since it should be rfid
        # delete just in case an entry already exsited for this rfid
        self.delete_row(insert_row[0])
        insert_query = 'INSERT INTO %s (%s) VALUES (%s)' % (self.table_name, ','.join(self.rows), ','.join(['?'] * len(self.rows)))
        logger.debug('insert_query:\n%s\n' % insert_query)
        self.cur.execute(insert_query, tuple(insert_row))
        self.db.commit()

    def delete_row(self, rfid):
        self.cur.execute('DELETE FROM %s WHERE rfid="%s"' % (self.table_name, rfid))
        self.db.commit()

    def get_row_as_dict(self, rfid):
        """
        Grabs the row from the database and turns it into a dict

        Assumes the data returned from the database is in the same order
        as self.rows, and iterates over those to turn the tuple into a dict.
        """
        ret = {}
        self.cur.execute('SELECT * from %s where rfid="%s"' % (self.table_name, rfid))
        row = self.cur.fetchone()
        if row is None:
            self.error('Could not find a matching id...')
            return
        logger.debug('got row: %s' % row.__class__)
        for i in range(len(row)):
            ret[self.rows[i]] = row[i]
        return ret

    def check_row_validity(self, row):
        if len(row) > len(self.rows):
            logger.error('Too much data was supplied to be inserted, (%s)' % row)
        for member in self.rows:
            if member not in row:
                logger.error('%s does not exist in the valid rows..' % member)
                raise

    def error(self, msg):
        self.error_pub.publish(msg)

    def _close_database(self):
        self.db.close()
        self.cur = None
        self.db = None


def main():
    rospy.init_node(NODE_NAME)

    local_path = rospy.get_param('~remote_database', '/home/lg/rfid/rfid_storage.db')
    scan_topic = rospy.get_param('~scan_topic', '/rfid/scan')
    state_set_topic = rospy.get_param('~state_set_topic', '/state_setter/set_state')
    error_topic = rospy.get_param('~error_topic', '/info/error')
    update_topic = rospy.get_param('~update_topic', '/rfid/spreadsheet/update')

    state_set_pub = rospy.Publisher(state_set_topic, String, queue_size=10)
    error_pub = rospy.Publisher(error_topic, String, queue_size=10)

    d = RfidStorage(local_path, state_set_pub=state_set_pub, error_pub=error_pub)

    rospy.Subscriber(scan_topic, String, d.handle_scan)
    rospy.Subscriber(update_topic, String, d.handle_set)

    rospy.spin()


if __name__ == "__main__":
    run_with_influx_exception_handler(main, NODE_NAME)
