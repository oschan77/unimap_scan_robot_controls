import glob
import os
import subprocess
import time


class ScanRobotControls:
    def __init__(self):
        self.rosbag_dir_path = "/home/u/rosbag"

    def get_latest_created_folder(self):
        all_folders = [
            f
            for f in glob.glob(os.path.join(self.rosbag_dir_path, "*"))
            if os.path.isdir(f)
        ]
        latest_folder = max(all_folders, key=os.path.getctime, default=None)
        return os.path.basename(latest_folder) if latest_folder else None

    def get_pid(self):
        cmd_pid = "docker exec rosbag bash -c \"ps aux | grep '[r]os2 bag record'\""
        proc_pid = subprocess.Popen(
            cmd_pid,
            shell=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )
        stdout, _ = proc_pid.communicate()
        ps_results = stdout.decode("utf-8").split("\n")[0]

        if not ps_results:
            print("ros2 bag record is not running")
            return None

        print("ros2 bag record is running")
        pid = int(ps_results.split()[1])
        print(f"ros2 bag record PID: {pid}")
        return pid

    def record(self):
        cmd_record = "docker exec rosbag bash -c 'source /opt/ros/humble/setup.bash; ros2 bag record /ouster/points /ouster/imu /ouster/scan /odom'"
        proc_record = subprocess.Popen(
            cmd_record,
            shell=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )
        print("ros2 bag record started")
        time.sleep(3)
        self.get_pid()

    def stop(self):
        pid = self.get_pid()
        if pid:
            cmd_stop = f"docker exec rosbag bash -c 'kill -2 {pid}'"
            proc_stop = subprocess.Popen(
                cmd_stop,
                shell=True,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )
            proc_stop.communicate()
            print("ros2 bag record stopped")
            self.chown()

    def convert(self):
        ros2bag_folder = self.get_latest_created_folder()
        if ros2bag_folder:
            cmd_convert = f"docker exec rosbag bash -c 'source /opt/ros/humble/setup.bash; rosbags-convert --src /rosbag/{ros2bag_folder} --dst /rosbag/{ros2bag_folder}.bag'"
            proc_convert = subprocess.Popen(
                cmd_convert,
                shell=True,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )
            proc_convert.communicate()
            print(f"ros2 bag convert completed for {ros2bag_folder}")
            self.chown()
        else:
            print("No ros2 bag is found")

    def chown(self):
        cmd_chown = f"sudo -S chown -R u {self.rosbag_dir_path}/*"
        proc_chown = subprocess.Popen(
            cmd_chown,
            shell=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )
        proc_chown.communicate(input=b"u")
        print("chown completed")
