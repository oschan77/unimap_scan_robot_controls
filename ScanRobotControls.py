import glob
import os
import signal
import subprocess
import time


class ScanRobotControls:
    def __init__(self):
        self.rosbag_dir_path = "/home/u/rosbag"

    def get_latest_created_folder(self):
        all_folders = [
            os.path.basename(f)
            for f in glob.glob(os.path.join(self.rosbag_dir_path, "*"))
            if os.path.isdir(f)
        ]

        return max(all_folders, key=lambda x: os.path.getctime(x), default=None)

    def record(self):
        print("Starts rosbag record")
        cmd_record = "docker exec rosbag bash -c 'source /opt/ros/humble/setup.bash; ros2 bag record /ouster/points /ouster/imu /ouster/scan /odom'"
        proc_record = subprocess.Popen(
            cmd_record,
            shell=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )

        stdout, stderr = proc_record.communicate()

        return stdout, stderr

    def get_pid(self):
        cmd_pid = "docker exec rosbag ps aux | grep ros2"
        proc_pid = subprocess.Popen(
            cmd_pid,
            shell=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )

        stdout, _ = proc_pid.communicate()

        pid = stdout.decode("utf-8").split("\n")[0].split()[1]

        print(f"ros2 bag record PID: {pid}")

        return pid

    def stop(self):
        print("Stops rosbag record")
        pid = self.get_pid()
        os.kill(pid, signal.SIGINT)

    def convert(self):
        print("Starts rosbag convert")
        ros2bag_folder = self.get_latest_created_folder()
        if ros2bag_folder:
            cmd_convert = f"docker exec rosbag bash -c 'source /opt/ros/humble/setup.bash; rosbags-convert --src {ros2bag_folder} --dst {ros2bag_folder}.bag'"

            proc_convert = subprocess.Popen(
                cmd_convert,
                shell=True,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )

            stdout, stderr = proc_convert.communicate()

            return stdout, stderr
