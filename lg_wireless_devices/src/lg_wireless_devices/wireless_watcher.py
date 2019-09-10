#!/usr/bin/env python3
# THIS FILE IS MANAGED BY CHEF AND THIS IS JUST A PLACEHOLDER ####
import evdev
import rospy
from lg_wireless_devices.msg import Command
from time import time


class WirelessWatcher:
    def __init__(self, config, command_handler):
        self.keys = self.get_keys(config)
        self.codes = self.get_codes(config)
        self.dev = self.get_device(config)
        self.interval = rospy.get_param('~interval', 5)
        self.last_execution_time = time()
        self.command_pub = command_handler
        self.run()

    def get_codes(self, config):
        """
        Grabs all code=>command pairs out of the config supplied

        Skips the sections general and keys, and ignores any errors
        when parsing the config file.
        """
        codes = {}
        for section in config.sections():
            # ignore general and key sections
            if section == 'general' or section == 'keys':
                continue
            try:
                code = config.get(section, 'code')
                command = config.get(section, 'command')
            except Exception:
                print "Problem parsing the code or command, continuing"
                print "Current codes are: %s" % codes
                continue
            codes[code] = command

        return codes

    def get_keys(self, config):
        """
        Grabs data on all the keys that can be pressed by the
        wireless device. These keys are required so this will throw
        an exception and fail if the [keys] section can't be found.

        Iterates over all options in the keys section, they are tuples
        that look like ('name', '1,2,3') where 1,2,3 correspond to code,
        type, and value respectively.
        """
        keys = {}
        for key in config.items('keys'):
            name = key[0]
            data = key[1].split(',')
            try:
                keys[name] = {"code": data[0], "type": data[1], "value": data[2]}
            except IndexError:
                print 'Malformed data, could not grab three comma separated values from (%s)' % data

        return keys

    def get_device(self, config):
        """
        Opens the device, grabs it so no one else receives its input,
        and then returns the device
        """
        device_path = config.get('general', 'device_path')
        dev = evdev.InputDevice(device_path)
        dev.grab()
        return dev

    def run(self):
        while True:
            for ev in self.dev.read_loop():
                self.handle_event(ev)

    def handle_event(self, event):
        """
        Looks for event in self.keys, if there's a match, the
        command associated is executed
        """
        command = self.get_command(event.code, event.type, event.value)
        if not command:
            return
        now = time()
        if now - self.last_execution_time < self.interval:
            return
        self.last_execution_time = now

        comm_msg = Command()
        comm_msg.command = command

        self.command_pub.publish(comm_msg)

    def get_command(self, c, t, v):
        """
        Matches code / type / value against all known codes
        """
        for code, key in self.keys.iteritems():
            if key['code'] == str(c) and key['type'] == str(t) and key['value'] == str(v):
                return self.codes[code]
        return None
