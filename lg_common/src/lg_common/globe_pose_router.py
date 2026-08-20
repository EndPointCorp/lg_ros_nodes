import copy


BASES = ('earth', 'cesium', 'unreal')
TOUCHSCREEN_SOURCE_PREFIX = 'touchscreen:'


class GlobePoseRouter(object):
    """Route one control stream through the selected globe renderer.

    Only feedback from the selected renderer becomes authoritative. The
    normalized result is published to touchscreen controllers and copied to
    every inactive renderer, avoiding pairwise feedback loops.
    """

    def __init__(self, selected='earth', command_outputs=None,
                 sync_outputs=None, pose_output=None):
        if selected not in BASES:
            raise ValueError('unknown base application: {}'.format(selected))
        self.selected = selected
        self.owner = None
        self.latest_pose = None
        self.command_outputs = command_outputs or {}
        self.sync_outputs = sync_outputs or {}
        self.pose_output = pose_output

    def select(self, base):
        if base not in BASES:
            raise ValueError('unknown base application: {}'.format(base))
        changed = base != self.selected
        self.selected = base
        # The new foreground renderer has normally followed in the background,
        # but replay the latest pose once to close any switch-time race.
        if changed and self.latest_pose is not None:
            self._publish(self.sync_outputs, base, self.latest_pose.pose)
        return changed

    def set_owner(self, owner):
        self.owner = owner or None

    def handle_command(self, message):
        source = getattr(message.header, 'frame_id', '')
        if source.startswith(TOUCHSCREEN_SOURCE_PREFIX):
            owner = source[len(TOUCHSCREEN_SOURCE_PREFIX):]
            if not owner or owner != self.owner:
                return False
        return self._publish(self.command_outputs, self.selected, message.pose)

    def handle_feedback(self, base, message):
        if base != self.selected:
            return False

        canonical = copy.deepcopy(message)
        if base == 'earth':
            # Earth ViewSync's legacy wire format reports latitude in x and
            # longitude in y. The shared pose contract is longitude x,
            # latitude y, matching KML, Cesium, and Unreal.
            canonical.pose.position.x = message.pose.position.y
            canonical.pose.position.y = message.pose.position.x
        canonical.header.frame_id = 'globe:{}'.format(base)
        self.latest_pose = canonical

        if self.pose_output:
            self.pose_output(canonical)
        for follower in BASES:
            if follower != base:
                self._publish(self.sync_outputs, follower, canonical.pose)
        return True

    @staticmethod
    def _publish(outputs, base, message):
        output = outputs.get(base)
        if output is None:
            return False
        output(message)
        return True
