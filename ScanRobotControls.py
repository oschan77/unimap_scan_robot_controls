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

    def record(self):
        cmd_record = "docker exec rosbag bash -c 'source /opt/ros/humble/setup.bash; ros2 bag record /ouster/points /ouster/imu /ouster/scan /odom'"
        self.proc_record = subprocess.Popen(
            cmd_record,
            shell=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )

        self.proc_record = subprocess.Popen(
            cmd_record,
            shell=True,
            stdout=subprocess.PIPE,
        )

        print(f"Started rosbag record with pid {self.proc_record.pid}")

    def stop(self):
        if self.proc_record is not None:
            print(f"Stopping rosbag record with pid {self.proc_record.pid}")
            os.killpg(os.getpgid(self.proc_record.pid), signal.SIGINT)
            self.proc_record.wait()

    def convert(self):
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

            return proc_convert.returncode, stdout, stderr
