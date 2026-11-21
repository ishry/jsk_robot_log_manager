#!/usr/bin/env python3

"""Save the hrpsys tmux pane output during robot log sessions."""

import json
import os
import re
import shlex
import subprocess
import threading

import rospy
from std_msgs.msg import String


class HrpsysConsoleLogger:
    def __init__(self):
        self.log_root = os.path.expanduser(rospy.get_param("~log_root", "~/jsk_robot_logs"))
        self.target = rospy.get_param("~pane", "")
        self.window = rospy.get_param("~window", "hrpsys")
        self.lock = threading.Lock()
        self.capture = None
        rospy.on_shutdown(self.close)
        self.subscriber = rospy.Subscriber(
            rospy.get_param("~event_topic", "jsk_robot_log_manager/event"),
            String, self.on_event, queue_size=20,
        )

    @staticmethod
    def tmux(*args):
        return subprocess.check_output(
            ["tmux"] + list(args), stderr=subprocess.STDOUT, timeout=3
        ).decode("utf-8", "replace").strip()

    def find_pane(self):
        if self.target:
            target = self.target
        else:
            source = os.environ.get("TMUX_PANE")
            if not source:
                raise ValueError("Launch from byobu or set hrpsys_console_pane")
            session = self.tmux("display-message", "-p", "-t", source, "#{session_id}")
            # Use the launch pane's session, even when another has an hrpsys window.
            target = session + ":=" + self.window
        panes = self.tmux("list-panes", "-t", target, "-F", "#{pane_id}").splitlines()
        if self.target.startswith("%") and self.target in panes:
            return self.target
        if len(panes) != 1:
            raise ValueError("Expected one hrpsys pane at " + target)
        return panes[0]

    def start(self, session):
        if self.capture:
            if self.capture[0] == session:
                return
            self.stop()
        directory = os.path.join(self.log_root, session, "hrpsys")
        if not os.path.isdir(directory):
            raise ValueError("Missing session log directory: " + directory)
        pane = self.find_pane()
        if self.tmux("display-message", "-p", "-t", pane, "#{pane_pipe}") == "1":
            raise ValueError("Pane " + pane + " already has an output pipe")
        path = os.path.join(directory, "console.log")
        with open(path, "ab"):
            pass
        self.tmux("pipe-pane", "-o", "-O", "-t", pane, "exec cat >> " + shlex.quote(path))
        self.capture = (session, pane)
        rospy.loginfo("Saving hrpsys pane %s to %s", pane, path)

    def stop(self):
        if self.capture:
            _, pane = self.capture
            self.tmux("pipe-pane", "-t", pane)
            self.capture = None

    def on_event(self, message):
        try:
            data = json.loads(message.data)
            session = data["session_id"]
            if not isinstance(session, str) or not re.fullmatch(r"[a-zA-Z0-9_-]+", session):
                raise ValueError("Invalid session ID")
            with self.lock:
                if data["event"] == "START":
                    self.start(session)
                elif data["event"] == "STOP" and self.capture and self.capture[0] == session:
                    self.stop()
        except subprocess.CalledProcessError as error:
            rospy.logwarn("Cannot capture hrpsys console: %s", error.output.decode("utf-8", "replace").strip())
        except (OSError, ValueError, KeyError, TypeError, subprocess.TimeoutExpired) as error:
            rospy.logwarn("Cannot capture hrpsys console: %s", error)

    def close(self):
        with self.lock:
            try:
                self.stop()
            except (OSError, subprocess.SubprocessError) as error:
                rospy.logwarn("Cannot close hrpsys console capture: %s", error)


if __name__ == "__main__":
    rospy.init_node("hrpsys_console_logger")
    logger = HrpsysConsoleLogger()
    rospy.spin()
