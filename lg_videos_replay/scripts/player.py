import os
import json
import subprocess
from pathlib import Path
from visionport.vpros.models.interactivespaces_msgs.msg import GenericMessage

class VideoReplayPlayer:
    def __init__(self, recordings_log, recordings_path):
        self.recordings_log = Path(recordings_log)
        self.recordings_path = Path(recordings_path)
        self.process = None

    def read_log(self):
        log = []
        if not self.recordings_log.exists():
            return log
        with open(self.recordings_log, 'r') as f:
            for line in f:
                if line.strip():
                    try:
                        log.append(json.loads(line))
                    except json.JSONDecodeError:
                        pass
        return sorted(log, key=lambda x: x['start'])

    def find_segments(self, timestamp):
        log = self.read_log()
        target_timestamp = float(timestamp)

        # Find starting segment
        start_idx = -1
        for i, segment in enumerate(log):
            if segment['start'] <= target_timestamp < segment['stop']:
                start_idx = i
                break

        if start_idx == -1:
            rospy.logwarn(f"Timestamp {target_timestamp} not found in recordings log.")
            return None, []

        start_segment = log[start_idx]
        offset = target_timestamp - start_segment['start']

        # Build continuous playlist, stopping at gaps > 2.5 seconds (matches lg_videos validation)
        playlist = [start_segment]
        current_stop = start_segment['stop']

        for segment in log[start_idx + 1:]:
            gap = segment['start'] - current_stop
            if gap > 2.5:
                rospy.loginfo(f"Gap detected ({gap}s), stopping playlist accumulation.")
                break
            playlist.append(segment)
            current_stop = segment['stop']

        return offset, playlist

    def play(self, timestamp):
        self.stop()

        offset, playlist = self.find_segments(timestamp)
        if not playlist:
            rospy.logerr(f"Cannot play: no segments found for timestamp {timestamp}")
            return False

        rospy.loginfo(f"Playing {len(playlist)} segments starting at offset {offset:.2f}s")

        # Build mpv command
        cmd = ['mpv', '--fullscreen', '--fs-screen=all', '--no-terminal', '--no-osc', '--no-osd-bar']

        env = os.environ.copy()
        if 'DISPLAY' not in env:
            env['DISPLAY'] = ':0'

        # Add files to the mpv command, applying start offset only to the first
        first = True
        for segment in playlist:
            filepath = self.recordings_path / segment['filename']
            if not filepath.exists():
                rospy.logwarn(f"File missing: {filepath}, stopping playlist here")
                break

            if first:
                cmd.extend(['--start', str(offset)])
                first = False
            else:
                cmd.extend(['--start', '0'])

            cmd.append(str(filepath))

        rospy.loginfo(f"Running mpv command: {' '.join(cmd)}")
        self.process = subprocess.Popen(cmd, env=env)
        return True

    def stop(self):
        if self.process and self.process.poll() is None:
            rospy.loginfo("Stopping current playback...")
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
            self.process = None

