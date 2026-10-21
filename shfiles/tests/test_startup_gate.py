"""No ROS, network or systemd required: python3 -m unittest discover -s shfiles/tests."""
import importlib.util
from pathlib import Path
import socket
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

SPEC = importlib.util.spec_from_file_location(
    "startup_gate", Path(__file__).resolve().parents[1] / "wait_for_stable_clock.py")
gate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gate)


class PrerequisitesTest(unittest.TestCase):
    def setUp(self):
        self.settings = gate.Settings()
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.marker = Path(self.directory.name) / "synchronized"
        self.marker.touch()

    def test_network_and_this_boot_sync_required(self):
        run = Mock(side_effect=[(True, ""), (True, "active"), (True, "yes")])
        self.assertTrue(gate.prerequisites(self.settings, run, self.marker)[0])
        self.assertEqual(run.call_args_list[0].args[0][-3:],
                         [self.settings.host, "80", "5.0"])

    def test_offline_even_with_cached_sync_does_not_pass(self):
        run = Mock(return_value=(False, ""))
        self.assertFalse(gate.prerequisites(self.settings, run, self.marker)[0])
        self.assertEqual(run.call_count, 1)

    def test_persisted_date_or_kernel_flag_without_this_boot_marker_rejected(self):
        self.marker.unlink()
        run = Mock(side_effect=[(True, ""), (True, "active"), (True, "yes")])
        ok, reason = gate.prerequisites(self.settings, run, self.marker)
        self.assertFalse(ok)
        self.assertIn("this boot", reason)
        self.assertEqual(run.call_count, 2)

    def test_no_sync_and_inactive_daemon_are_rejected(self):
        for replies in ([(True, ""), (False, "inactive")],
                        [(True, ""), (True, "active"), (True, "no")],
                        [(True, ""), (True, "active"), (False, "")]):
            with self.subTest(replies=replies):
                self.assertFalse(gate.prerequisites(
                    self.settings, Mock(side_effect=replies), self.marker)[0])

    def test_missing_tools_and_timeouts_fail_closed(self):
        for error in (FileNotFoundError(), subprocess.TimeoutExpired("probe", 5)):
            with self.subTest(error=error), patch.object(gate.subprocess, "run", side_effect=error):
                self.assertEqual(gate.command(["missing"], 5), (False, ""))

    def test_tcp_probe_real_local_connection(self):
        with socket.socket() as listener:
            listener.bind(("127.0.0.1", 0))
            listener.listen(1)
            port = listener.getsockname()[1]
            self.assertTrue(gate.command(
                [gate.sys.executable, "-c", gate.TCP_PROBE, "127.0.0.1", str(port), "2"], 3)[0])

    def test_real_subprocess_is_bounded(self):
        self.assertEqual(gate.command(
            [gate.sys.executable, "-c", "import time; time.sleep(10)"], 0.1), (False, ""))

    def test_invalid_environment_rejected(self):
        for key, value in (("CLOCK_WAIT_TIMEOUT", "nan"), ("CLOCK_WAIT_TIMEOUT", "-1"),
                           ("CLOCK_STABLE_SECONDS", "0"), ("NETWORK_CHECK_PORT", "65536"),
                           ("NETWORK_CHECK_HOST", ""), ("NETWORK_CHECK_TIMEOUT", "inf"),
                           ("CLOCK_START_STEP_TOLERANCE", "-0.1")):
            with self.subTest(key=key), patch.dict(gate.os.environ, {key: value}, clear=True):
                with self.assertRaises(ValueError):
                    gate.Settings.from_env()


class StabilityTest(unittest.TestCase):
    def setUp(self):
        self.window = gate.StabilityWindow(gate.Settings())
        self.epoch = 1800000000.0

    def update(self, t, ready=True, offset=0):
        return self.window.update(self.epoch + t + offset, t, ready)[0]

    def test_ten_seconds_start_after_network_and_sync(self):
        self.assertFalse(self.update(0, False))
        self.assertFalse(self.update(30, False))
        self.assertFalse(self.update(40))
        self.assertFalse(self.update(49))
        self.assertTrue(self.update(50))

    def test_lost_network_or_sync_resets_window(self):
        self.update(0)
        self.update(9, False)
        self.assertFalse(self.update(10))
        self.assertFalse(self.update(19))
        self.assertTrue(self.update(20))

    def test_small_backward_ntp_step_below_old_half_second_threshold(self):
        self.update(0)
        self.assertFalse(self.update(9, offset=-0.42))
        self.assertFalse(self.update(10, offset=-0.42))
        self.assertFalse(self.update(19, offset=-0.42))
        self.assertTrue(self.update(20, offset=-0.42))

    def test_forward_step_resets_window(self):
        self.update(0)
        self.assertFalse(self.update(10, offset=1))
        self.assertFalse(self.update(11, offset=1))
        self.assertTrue(self.update(21, offset=1))

    def test_invalid_date_cannot_pass(self):
        self.assertFalse(self.window.update(0, 0, True)[0])
        self.assertFalse(self.window.update(20, 20, True)[0])

    def test_main_waits_then_returns_without_runtime_monitoring(self):
        settings = gate.Settings(stable_seconds=2)
        with patch.object(gate.Settings, "from_env", return_value=settings), \
                patch.object(gate, "prerequisites", side_effect=[(False, "offline"), (True, ""), (True, "")]) as probe, \
                patch.object(gate.time, "monotonic", side_effect=[0, 2, 4, 6]), \
                patch.object(gate.time, "time", side_effect=[self.epoch + t for t in (0, 2, 4, 6)]), \
                patch.object(gate.time, "sleep"), patch("builtins.print"):
            self.assertEqual(gate.main(), 0)
            self.assertEqual(probe.call_count, 3)

    def test_optional_timeout_does_not_launch(self):
        with patch.object(gate.Settings, "from_env", return_value=gate.Settings(wait_timeout=2)), \
                patch.object(gate, "prerequisites", return_value=(False, "offline")), \
                patch.object(gate.time, "monotonic", side_effect=[0, 3]), \
                patch.object(gate.time, "time", return_value=self.epoch), patch("builtins.print"):
            self.assertEqual(gate.main(), 1)


if __name__ == "__main__":
    unittest.main()
