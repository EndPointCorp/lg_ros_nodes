import importlib.machinery
import importlib.util
import unittest
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[2]
    / 'scripts'
    / 'kb_mouse_activity.py'
)


def load_module():
    loader = importlib.machinery.SourceFileLoader(
        'kb_mouse_activity',
        str(SCRIPT_PATH),
    )
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


class FakeX:
    KeyPress = 2
    KeyRelease = 3
    ButtonPress = 4
    ButtonRelease = 5
    MotionNotify = 6


class KeyboardMouseActivityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_module()

    def test_only_press_and_motion_events_are_activity(self):
        self.assertTrue(
            self.module.is_activity_event(FakeX.KeyPress, FakeX),
        )
        self.assertTrue(
            self.module.is_activity_event(FakeX.ButtonPress, FakeX),
        )
        self.assertTrue(
            self.module.is_activity_event(FakeX.MotionNotify, FakeX),
        )
        self.assertFalse(
            self.module.is_activity_event(FakeX.KeyRelease, FakeX),
        )
        self.assertFalse(
            self.module.is_activity_event(FakeX.ButtonRelease, FakeX),
        )

    def test_activity_limiter_throttles_motion_bursts(self):
        limiter = self.module.ActivityLimiter(interval=0.25)
        self.assertTrue(limiter.ready(10.0))
        self.assertFalse(limiter.ready(10.1))
        self.assertTrue(limiter.ready(10.25))


if __name__ == '__main__':
    unittest.main()
