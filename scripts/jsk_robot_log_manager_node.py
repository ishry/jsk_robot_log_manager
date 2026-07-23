#!/usr/bin/env python3

"""ROS node coordinating JSK robot logging backends."""

import datetime
import glob
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import threading
import time

import rospy
from hrpsys_ros_bridge.srv import OpenHRP_DataLoggerService_clear
from hrpsys_ros_bridge.srv import OpenHRP_DataLoggerService_maxLength
from hrpsys_ros_bridge.srv import OpenHRP_DataLoggerService_save
from std_msgs.msg import String
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse

from jsk_robot_log_manager.srv import StartLogging
from jsk_robot_log_manager.srv import StartLoggingResponse


class JskRobotLogManager(object):
    HRPSYS_CLEAR_SERVICE = "/DataLoggerServiceROSBridge/clear"
    HRPSYS_MAX_LENGTH_SERVICE = "/DataLoggerServiceROSBridge/maxLength"
    HRPSYS_SAVE_SERVICE = "/DataLoggerServiceROSBridge/save"

    def __init__(self):
        self.log_root = os.path.abspath(os.path.expanduser(
            rospy.get_param("~log_root", "~/jsk_robot_logs")
        ))
        self.default_duration_sec = int(
            rospy.get_param("~default_duration_sec", 180)
        )
        self.hrpsys_rate_hz = int(rospy.get_param("~hrpsys_rate_hz", 500))
        self.record_all = bool(rospy.get_param("~record_all", True))
        self.topics = list(rospy.get_param("~topics", []))
        self.min_free_gb = int(rospy.get_param("~min_free_gb", 5))
        self.service_timeout_sec = float(
            rospy.get_param("~service_timeout_sec", 5.0)
        )
        self.neuro_timeout_sec = float(
            rospy.get_param("~neuro_timeout_sec", 3.0)
        )
        self.neuro_targets = self.parse_neuro_targets(
            rospy.get_param("~neuro_targets", [])
        )

        self.lock = threading.RLock()
        self.session = None
        self.bag_process = None
        self.bag_output = None
        self.last_status = "No logging session has been started."
        self.shutting_down = False

        self.hrpsys_clear = rospy.ServiceProxy(
            self.HRPSYS_CLEAR_SERVICE,
            OpenHRP_DataLoggerService_clear,
        )
        self.hrpsys_max_length = rospy.ServiceProxy(
            self.HRPSYS_MAX_LENGTH_SERVICE,
            OpenHRP_DataLoggerService_maxLength,
        )
        self.hrpsys_save = rospy.ServiceProxy(
            self.HRPSYS_SAVE_SERVICE,
            OpenHRP_DataLoggerService_save,
        )

        self.event_topic = rospy.resolve_name("~event")
        self.event_publisher = rospy.Publisher(
            "~event",
            String,
            queue_size=10,
            latch=True,
        )
        self.start_service = rospy.Service(
            "~start",
            StartLogging,
            self.handle_start,
        )
        self.stop_service = rospy.Service(
            "~stop",
            Trigger,
            self.handle_stop,
        )
        self.status_service = rospy.Service(
            "~status",
            Trigger,
            self.handle_status,
        )
        rospy.on_shutdown(self.on_shutdown)

    def now_utc(self):
        return datetime.datetime.now(
            datetime.timezone.utc
        ).isoformat(timespec="milliseconds")

    def sanitize_name(self, name):
        clean_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip())
        clean_name = clean_name.strip("._-")
        return clean_name or "session"

    def parse_neuro_targets(self, target_configs):
        if not isinstance(target_configs, list):
            raise ValueError("neuro_targets must be a list")

        targets = []
        target_names = set()
        for index, config in enumerate(target_configs):
            if not isinstance(config, dict):
                raise ValueError(
                    "neuro_targets[{}] must be a dictionary".format(index)
                )
            name = str(config.get("name", "")).strip()
            if not name:
                raise ValueError(
                    (
                        "neuro_targets[{}].name is required"
                    ).format(index)
                )
            if self.sanitize_name(name) != name:
                raise ValueError(
                    (
                        "Neuro target name '{}' contains unsupported "
                        "characters"
                    ).format(name)
                )
            if name in target_names:
                raise ValueError(
                    "Neuro target name '{}' is duplicated".format(name)
                )
            target_names.add(name)

            host = str(config.get("host", "127.0.0.1")).strip()
            port = int(config.get("port", 7000))
            if not host:
                raise ValueError(
                    "Neuro target '{}'.host is empty".format(name)
                )
            if port < 1 or port > 65535:
                raise ValueError(
                    "Neuro target '{}'.port is invalid".format(name)
                )
            if not bool(config.get("enabled", True)):
                continue
            targets.append({
                "name": name,
                "host": host,
                "port": port,
                "required": bool(config.get("required", False)),
            })
        return targets

    def make_session_directory(self, name):
        os.makedirs(self.log_root, exist_ok=True)
        free_bytes = shutil.disk_usage(self.log_root).free
        required_bytes = self.min_free_gb * 1024 * 1024 * 1024
        if free_bytes < required_bytes:
            raise RuntimeError(
                (
                    "free disk space is {:.1f} GiB; "
                    "at least {} GiB is required"
                ).format(
                    free_bytes / float(1024 ** 3),
                    self.min_free_gb,
                )
            )

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        base_name = "{}_{}".format(timestamp, self.sanitize_name(name))
        session_directory = os.path.join(self.log_root, base_name)
        suffix = 1
        while os.path.exists(session_directory):
            session_directory = os.path.join(
                self.log_root,
                "{}_{:02d}".format(base_name, suffix),
            )
            suffix += 1

        os.makedirs(os.path.join(session_directory, "hrpsys"))
        os.makedirs(os.path.join(session_directory, "neuro"))
        for target in self.neuro_targets:
            os.makedirs(os.path.join(
                session_directory,
                "neuro",
                target["name"],
            ))
        os.makedirs(os.path.join(session_directory, "ros"))
        return session_directory

    def write_manifest(self, session):
        manifest_path = os.path.join(session["directory"], "manifest.json")
        temporary_path = manifest_path + ".tmp"
        with open(temporary_path, "w") as stream:
            json.dump(session, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, manifest_path)

    def publish_event(self, event, session):
        message = {
            "event": event,
            "session_id": session["session_id"],
            "time_utc": self.now_utc(),
        }
        self.event_publisher.publish(
            String(data=json.dumps(message, sort_keys=True))
        )

    def wait_for_hrpsys(self):
        for service_name in [
            self.HRPSYS_CLEAR_SERVICE,
            self.HRPSYS_MAX_LENGTH_SERVICE,
            self.HRPSYS_SAVE_SERVICE,
        ]:
            rospy.wait_for_service(
                service_name,
                timeout=self.service_timeout_sec,
            )

    def send_neuro_command(self, target, command):
        connection = socket.create_connection(
            (target["host"], target["port"]),
            timeout=self.neuro_timeout_sec,
        )
        try:
            connection.settimeout(self.neuro_timeout_sec)
            connection.sendall((command + "\n").encode("utf-8"))
            response_data = bytearray()
            while b"\n" not in response_data:
                chunk = connection.recv(4096)
                if not chunk:
                    break
                response_data.extend(chunk)
                if len(response_data) > 65536:
                    raise RuntimeError(
                        "Neuro '{}' response is too large".format(
                            target["name"]
                        )
                    )
        finally:
            connection.close()

        response = response_data.decode("utf-8", errors="replace").strip()
        if not response:
            raise RuntimeError(
                "Neuro '{}' returned no response".format(target["name"])
            )
        if not response.startswith("OK "):
            raise RuntimeError(
                "Neuro '{}': {}".format(target["name"], response)
            )
        return response

    def check_neuro(self):
        available_targets = []
        unavailable_targets = []
        for target in self.neuro_targets:
            try:
                connection = socket.create_connection(
                    (target["host"], target["port"]),
                    timeout=self.neuro_timeout_sec,
                )
                connection.close()
                available_targets.append(target)
            except OSError as error:
                unavailable_targets.append((
                    target,
                    "Neuro {} is not reachable at {}:{} ({})".format(
                        target["name"],
                        target["host"],
                        target["port"],
                        error,
                    ),
                ))
        return available_targets, unavailable_targets

    def start_rosbag(self, session):
        rosbag_executable = shutil.which("rosbag")
        if rosbag_executable is None:
            raise RuntimeError("rosbag executable was not found")

        bag_prefix = os.path.join(session["directory"], "ros", "session")
        command = [
            rosbag_executable,
            "record",
            "--lz4",
            "--min-space={}G".format(self.min_free_gb),
            "-O",
            bag_prefix,
        ]
        if self.record_all:
            command.append("-a")
        else:
            topics = list(self.topics)
            if not topics:
                raise RuntimeError(
                    "record_all is false but no rosbag topics are configured"
                )
            if self.event_topic not in topics:
                topics.append(self.event_topic)
            command.extend(topics)

        output_path = os.path.join(
            session["directory"],
            "ros",
            "rosbag.log",
        )
        self.bag_output = open(output_path, "w")
        try:
            self.bag_process = subprocess.Popen(
                command,
                stdout=self.bag_output,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
        except Exception:
            self.bag_output.close()
            self.bag_output = None
            raise
        time.sleep(0.5)
        return_code = self.bag_process.poll()
        if return_code is not None:
            self.bag_output.close()
            self.bag_output = None
            self.bag_process = None
            raise RuntimeError(
                "rosbag exited during startup with code {}; see {}".format(
                    return_code,
                    output_path,
                )
            )
        session["rosbag_command"] = command

    def stop_rosbag(self):
        errors = []
        process = self.bag_process
        output = self.bag_output
        self.bag_process = None
        self.bag_output = None

        if process is not None and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
                process.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                errors.append(
                    "rosbag did not exit within 20 seconds after SIGINT"
                )
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    process.wait(timeout=5.0)
                except (OSError, subprocess.TimeoutExpired) as error:
                    errors.append(
                        "rosbag did not exit after SIGTERM: {}".format(error)
                    )
            except OSError as error:
                errors.append("failed to stop rosbag: {}".format(error))

        if output is not None:
            output.close()
        return errors

    def rollback_start(self, session, started_neuro):
        errors = []
        for target in reversed(started_neuro):
            try:
                self.send_neuro_command(target, "log stop")
            except Exception as error:
                errors.append(
                    "failed to stop Neuro {} during rollback: {}".format(
                        target["name"],
                        error,
                    )
                )
        errors.extend(self.stop_rosbag())
        session["errors"].extend(errors)

    def handle_start(self, request):
        with self.lock:
            if self.session is not None:
                return StartLoggingResponse(
                    success=False,
                    message="Logging is already active.",
                    directory=self.session["directory"],
                )

            duration_sec = int(request.duration_sec)
            if duration_sec == 0:
                duration_sec = self.default_duration_sec
            if duration_sec <= 0:
                return StartLoggingResponse(
                    success=False,
                    message="hrpsys buffer duration must be positive.",
                    directory="",
                )
            buffer_samples = duration_sec * self.hrpsys_rate_hz
            if buffer_samples > 0xFFFFFFFF:
                return StartLoggingResponse(
                    success=False,
                    message="requested hrpsys buffer is too large.",
                    directory="",
                )

            try:
                session_directory = self.make_session_directory(request.name)
            except Exception as error:
                return StartLoggingResponse(
                    success=False,
                    message="Could not create session directory: {}".format(
                        error
                    ),
                    directory="",
                )

            session = {
                "session_id": os.path.basename(session_directory),
                "directory": session_directory,
                "status": "starting",
                "requested_at_utc": self.now_utc(),
                "stopped_at_utc": None,
                "hrpsys_buffer_sec": duration_sec,
                "hrpsys_buffer_samples": buffer_samples,
                "hrpsys_rate_hz": self.hrpsys_rate_hz,
                "record_all_ros_topics": self.record_all,
                "ros_topics": self.topics,
                "neuro_targets": self.neuro_targets,
                "components": {
                    "neuro": {},
                },
                "errors": [],
                "warnings": [],
                "files": {},
            }
            self.write_manifest(session)
            started_neuro = []

            try:
                self.wait_for_hrpsys()
                available_neuro, unavailable_neuro = self.check_neuro()
                required_unavailable = [
                    item for item in unavailable_neuro
                    if item[0]["required"]
                ]
                if required_unavailable:
                    raise RuntimeError("; ".join([
                        item[1] for item in required_unavailable
                    ]))
                for target, warning in unavailable_neuro:
                    rospy.logwarn(warning)
                    session["warnings"].append(warning)
                    session["components"]["neuro"][target["name"]] = {
                        "status": "skipped",
                        "host": target["host"],
                        "port": target["port"],
                        "required": target["required"],
                        "reason": warning,
                    }
                self.start_rosbag(session)

                for target in available_neuro:
                    prefix = os.path.join(
                        session_directory,
                        "neuro",
                        target["name"],
                        "log",
                    )
                    response = self.send_neuro_command(
                        target,
                        "log start {}".format(prefix),
                    )
                    started_neuro.append(target)
                    session["components"]["neuro"][target["name"]] = {
                        "status": "recording",
                        "host": target["host"],
                        "port": target["port"],
                        "required": target["required"],
                        "response": response,
                        "acknowledged_at_utc": self.now_utc(),
                    }

                self.hrpsys_max_length(buffer_samples)
                clear_response = self.hrpsys_clear()
                if not clear_response.operation_return:
                    raise RuntimeError("hrpsys clear returned false")
                session["components"]["hrpsys"] = {
                    "status": "recording",
                    "cleared_at_utc": self.now_utc(),
                }
                session["components"]["rosbag"] = {
                    "status": "recording",
                    "pid": self.bag_process.pid,
                }

                session["status"] = "recording"
                session["started_at_utc"] = self.now_utc()
                self.session = session
                self.publish_event("START", session)
                self.write_manifest(session)
                self.last_status = "Logging {}".format(session_directory)
                rospy.loginfo(self.last_status)
                response_message = "Logging started."
                if session["warnings"]:
                    response_message += " " + " ".join(session["warnings"])
                return StartLoggingResponse(
                    success=True,
                    message=response_message,
                    directory=session_directory,
                )
            except Exception as error:
                session["status"] = "start_failed"
                session["errors"].append(str(error))
                self.rollback_start(session, started_neuro)
                session["stopped_at_utc"] = self.now_utc()
                self.write_manifest(session)
                self.last_status = "Start failed for {}: {}".format(
                    session_directory,
                    error,
                )
                rospy.logerr(self.last_status)
                return StartLoggingResponse(
                    success=False,
                    message=self.last_status,
                    directory=session_directory,
                )

    def stop_current_session(self, reason):
        session = self.session
        errors = []
        session["status"] = "stopping"
        session["stop_reason"] = reason
        self.publish_event("STOP", session)
        self.write_manifest(session)
        rospy.sleep(0.1)

        for target in self.neuro_targets:
            component = session["components"]["neuro"].get(
                target["name"], {}
            )
            if component.get("status") == "skipped":
                continue
            try:
                response = self.send_neuro_command(target, "log stop")
                component.update({
                    "status": "stopped",
                    "stop_response": response,
                    "stopped_at_utc": self.now_utc(),
                })
            except Exception as error:
                errors.append(
                    "failed to stop Neuro {}: {}".format(
                        target["name"],
                        error,
                    )
                )

        hrpsys_prefix = os.path.join(
            session["directory"],
            "hrpsys",
            "log",
        )
        try:
            save_response = self.hrpsys_save(hrpsys_prefix)
            if not save_response.operation_return:
                raise RuntimeError("save returned false")
            session["components"]["hrpsys"].update({
                "status": "saved",
                "saved_at_utc": self.now_utc(),
                "prefix": hrpsys_prefix,
            })
        except Exception as error:
            errors.append("failed to save hrpsys log: {}".format(error))

        errors.extend(self.stop_rosbag())
        session["components"]["rosbag"]["status"] = "stopped"

        hrpsys_files = self.nonempty_files(glob.glob(hrpsys_prefix + ".*"))
        neuro_files = {}
        for target in self.neuro_targets:
            neuro_files[target["name"]] = self.nonempty_files(glob.glob(
                os.path.join(
                    session["directory"],
                    "neuro",
                    target["name"],
                    "log_*.dat",
                )
            ))
        bag_files = self.nonempty_files(glob.glob(os.path.join(
            session["directory"],
            "ros",
            "*.bag",
        )))
        session["files"] = {
            "hrpsys": len(hrpsys_files),
            "neuro": {
                name: len(paths)
                for name, paths in neuro_files.items()
            },
            "rosbag": len(bag_files),
        }
        required_outputs = [
            ("hrpsys", hrpsys_files),
            ("rosbag", bag_files),
        ]
        for target in self.neuro_targets:
            status = session["components"]["neuro"].get(
                target["name"], {}
            ).get("status")
            if status != "skipped":
                required_outputs.append((
                    "Neuro {}".format(target["name"]),
                    neuro_files[target["name"]],
                ))
        for source_name, source_files in required_outputs:
            if not source_files:
                errors.append(
                    "{} produced no output files".format(source_name)
                )

        session["errors"].extend(errors)
        session["stopped_at_utc"] = self.now_utc()
        session["status"] = "complete" if not errors else "partial"
        self.write_manifest(session)
        self.session = None

        if errors:
            self.last_status = "{} stopped with errors: {}".format(
                session["directory"],
                "; ".join(errors),
            )
            rospy.logerr(self.last_status)
            return False, self.last_status

        self.last_status = "Logging stopped: {}".format(
            session["directory"]
        )
        rospy.loginfo(self.last_status)
        return True, self.last_status

    def nonempty_files(self, paths):
        return sorted([
            path for path in paths
            if os.path.isfile(path) and os.path.getsize(path) > 0
        ])

    def handle_stop(self, _request):
        with self.lock:
            if self.session is None:
                return TriggerResponse(
                    success=False,
                    message="Logging is not active. {}".format(
                        self.last_status
                    ),
                )
            success, message = self.stop_current_session("requested")
            return TriggerResponse(success=success, message=message)

    def handle_status(self, _request):
        with self.lock:
            if self.session is None:
                return TriggerResponse(
                    success=True,
                    message="inactive; {}".format(self.last_status),
                )
            return TriggerResponse(
                success=True,
                message="recording: {}".format(
                    self.session["directory"]
                ),
            )

    def on_shutdown(self):
        with self.lock:
            if self.shutting_down:
                return
            self.shutting_down = True
            if self.session is not None:
                self.stop_current_session("manager_shutdown")


def main():
    rospy.init_node("jsk_robot_log_manager")
    JskRobotLogManager()
    rospy.loginfo("JSK robot log manager is ready.")
    rospy.spin()


if __name__ == "__main__":
    main()
