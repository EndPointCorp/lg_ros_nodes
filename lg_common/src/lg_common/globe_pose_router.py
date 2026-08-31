import copy
import json


BASES = ('earth', 'cesium', 'unreal')
TOUCHSCREEN_SOURCE_PREFIX = 'touchscreen:'


class GlobePoseRouter(object):
    """Route one control stream through the selected globe renderer.

    Only feedback from the selected renderer becomes authoritative. The
    normalized result is published to touchscreen controllers and copied to
    every inactive renderer, avoiding pairwise feedback loops.
    """

    def __init__(self, selected='earth', command_outputs=None,
                 live_outputs=None, live_stop_outputs=None,
                 sync_outputs=None, pose_output=None):
        if selected not in BASES:
            raise ValueError('unknown base application: {}'.format(selected))
        self.selected = selected
        self.owner = None
        self.control_session = None
        self.latest_pose = None
        self.command_outputs = command_outputs or {}
        self.live_outputs = live_outputs or {}
        self.live_stop_outputs = live_stop_outputs or {}
        self.sync_outputs = sync_outputs or {}
        self.pose_output = pose_output

    def select(self, base):
        if base not in BASES:
            raise ValueError('unknown base application: {}'.format(base))
        changed = base != self.selected
        old_base = self.selected
        self.selected = base
        if changed:
            self._stop_live_output(old_base)
        # The new foreground renderer has normally followed in the background,
        # but replay the latest pose once to close any switch-time race.
        if changed and self.latest_pose is not None:
            self._publish(self.sync_outputs, base, self.latest_pose.pose)
        return changed

    def set_owner(self, owner):
        owner = owner or None
        if owner != self.owner:
            self._stop_control_session()
        self.owner = owner

    def handle_control_session(self, message):
        """Begin or end an explicitly identified touchscreen gesture."""
        try:
            state = json.loads(message.data)
            action = state['action']
            owner = state['owner']
            session = state['session']
        except (AttributeError, KeyError, TypeError, ValueError):
            return False

        if not owner or not session:
            return False
        if action == 'begin':
            # A control session is the authoritative ownership event for globe
            # movement. /touchscreen/owner remains useful for scene writes and
            # early cancellation, but may arrive later on its separate topic.
            self.set_owner(owner)
            self._stop_control_session()
            self.control_session = (owner, session)
            return True
        if (action == 'end' and owner == self.owner and
                self.control_session == (owner, session)):
            # Leave the last live target in place long enough for a renderer
            # with physical-camera feedback (Earth) to finish converging. Its
            # controller has its own short stale-target safety limit.
            self.control_session = None
            return True
        return False

    def handle_command(self, message):
        source = getattr(message.header, 'frame_id', '')
        live = False
        if source.startswith(TOUCHSCREEN_SOURCE_PREFIX):
            identity = source[len(TOUCHSCREEN_SOURCE_PREFIX):].split(':', 1)
            owner = identity[0]
            if not owner or owner != self.owner:
                return False
            if len(identity) == 2:
                live = self.control_session == (owner, identity[1])
                if not live:
                    return False
        outputs = self.live_outputs if live else self.command_outputs
        return self._publish(
            outputs, self.selected, self._normalize_pose(message.pose))

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
        self._normalize_orientation(canonical.pose)
        canonical.header.frame_id = 'globe:{}'.format(base)
        self.latest_pose = canonical

        if self.pose_output:
            self.pose_output(canonical)
        return True

    def sync_inactive(self):
        """Send only the newest canonical pose to background renderers."""
        if self.latest_pose is None:
            return False
        published = False
        for follower in BASES:
            if follower != self.selected:
                published = (
                    self._publish(self.sync_outputs, follower,
                                  self.latest_pose.pose) or published
                )
        return published

    def _stop_control_session(self):
        if self.control_session is not None:
            self._stop_live_output(self.selected)
        self.control_session = None

    def _stop_live_output(self, base):
        output = self.live_stop_outputs.get(base)
        if output is not None:
            output()

    @staticmethod
    def _normalize_pose(pose):
        """Use one equivalent angle representation across globe renderers."""
        normalized = copy.deepcopy(pose)
        GlobePoseRouter._normalize_orientation(normalized)
        return normalized

    @staticmethod
    def _normalize_orientation(pose):
        pose.orientation.z = pose.orientation.z % 360.0
        pose.orientation.y = (
            (pose.orientation.y + 180.0) % 360.0 - 180.0
        )

    @staticmethod
    def _publish(outputs, base, message):
        output = outputs.get(base)
        if output is None:
            return False
        output(message)
        return True
