import rospy

def escape_asset_url(asset_url):
    ss = ''
    try:
        for ch in asset_url:
            if str.isalnum(str(ch)):
                ss += ch
            else:
                ss += "_"
        return ss
    except TypeError:
        rospy.logerr("Got invalid type when trying to escape assets url %s" % asset_url)
        return ss


def write_log_to_file(message):
    with open('/tmp/rostests.out', 'a') as rostest_file:
        rostest_file.write(message)
        rostest_file.write('\n')
