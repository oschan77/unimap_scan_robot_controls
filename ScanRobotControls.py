# import glob
# import os
# import signal
# import subprocess
# import threading
# import time


# class ScanRobotControls:
#     def __init__(self):
#         self.rosbag_dir_path = "/home/u/rosbag"
#         self.proc_record_pid = None
#         self.record_thread = None
#         self.password = "u"

#     def get_latest_created_folder(self):
#         all_folders = [
#             os.path.basename(f)
#             for f in glob.glob(os.path.join(self.rosbag_dir_path, "*"))
#             if os.path.isdir(f)
#         ]

#         return max(all_folders, key=lambda x: os.path.getctime(x), default=None)

#     def run_with_sudo(self, cmd: str):
#         proc = subprocess.Popen(
#             f"sudo -S {cmd}",
#             shell=True,
#             stdin=subprocess.PIPE,
#             stdout=subprocess.PIPE,
#             stderr=subprocess.PIPE,
#             preexec_fn=os.setsid,
#         )
#         proc.stdin.write(self.password.encode())
#         proc.stdin.flush()
#         stdout, stderr = proc.communicate()

#         return proc.returncode, stdout, stderr, proc.pid

#     def record_process(self):
#         cmd_record = "docker exec rosbag bash -c 'source /opt/ros/humble/setup.bash ; ros2 bag record /ouster/points /ouster/imu /ouster/scan /odom'"
#         returncode, stdout, stderr, pid = self.run_with_sudo(cmd_record)
#         self.proc_record_pid = pid

#         return returncode, stdout, stderr

#     def record(self):
#         self.record_thread = threading.Thread(target=self.record_process)
#         self.record_thread.start()

#     def stop(self):
#         if self.proc_record_pid is not None:
#             os.killpg(os.getpgid(self.proc_record_pid), signal.SIGINT)
#             time.sleep(10)
#             self.proc_record_pid = None
#             if self.record_thread is not None:
#                 self.record_thread.join()
#                 self.record_thread = None

#     def convert(self):
#         ros2bag_folder = self.get_latest_created_folder()
#         if ros2bag_folder:
#             cmd_convert = f"docker exec rosbag bash -c 'source /opt/ros/humble/setup.bash ; rosbags-convert --src {ros2bag_folder} --dst {ros2bag_folder}.bag'"

#             return self.run_with_sudo(cmd_convert)

#         return 1, b"", b"Folder not found"

#     def test(self):
#         return self.run_with_sudo("ls")


import glob
import os
import signal
import subprocess
import time


class ScanRobotControls:
    def __init__(self):
        self.rosbag_dir_path = "/home/u/rosbag"
        self.proc_record = None

    def get_latest_created_folder(self):
        all_folders = [
            os.path.basename(f)
            for f in glob.glob(os.path.join(self.rosbag_dir_path, "*"))
            if os.path.isdir(f)
        ]

        return max(all_folders, key=lambda x: os.path.getctime(x), default=None)

    def run_command(self, cmd: str):
        proc = subprocess.Popen(
            cmd,
            shell=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,  # Start the process in a new session
        )
        stdout, stderr = proc.communicate()
        return proc.returncode, stdout, stderr, proc.pid

    def record(self):
        cmd_record = "docker exec rosbag bash -c 'source /opt/ros/humble/setup.bash; ros2 bag record /ouster/points /ouster/imu /ouster/scan /odom'"
        self.proc_record = subprocess.Popen(
            cmd_record,
            shell=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,  # Start the process in a new session
        )
        time.sleep(1)  # Give some time for the process to start

    def stop(self):
        if self.proc_record is not None:
            try:
                # Send SIGINT to the process group
                os.killpg(os.getpgid(self.proc_record.pid), signal.SIGINT)
                time.sleep(
                    10
                )  # Give more time for the process to handle SIGINT and exit
            except Exception as e:
                print(f"Error sending SIGINT: {e}")
            finally:
                # Ensure the process is terminated
                self.proc_record.terminate()
                self.proc_record.wait()  # Wait for the process to handle SIGINT and exit
                self.proc_record = None

    def convert(self):
        ros2bag_folder = self.get_latest_created_folder()
        if ros2bag_folder:
            cmd_convert = f"docker exec rosbag bash -c 'source /opt/ros/humble/setup.bash; rosbags-convert --src {ros2bag_folder} --dst {ros2bag_folder}.bag'"
            return self.run_command(cmd_convert)

        return 1, b"", b"Folder not found"

    def test(self):
        return self.run_command("ls")
