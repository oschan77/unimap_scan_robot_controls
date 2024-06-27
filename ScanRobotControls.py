import glob
import os
import signal
import subprocess


class ScanRobotControls:
    def __init__(self):
        self.rosbag_dir_path = "/home/u/rosbag"
        self.proc_record_pid = None
        self.password = "u"

    def get_latest_created_folder(self):
        all_folders = [
            os.path.basename(f)
            for f in glob.glob(os.path.join(self.rosbag_dir_path, "*"))
            if os.path.isdir(f)
        ]
        return max(all_folders, key=lambda x: os.path.getctime(x), default=None)

    def run_with_sudo(self, cmd):
        proc = subprocess.Popen(
            f"sudo -S {cmd}",
            shell=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )
        proc.stdin.write(self.password.encode())
        proc.stdin.flush()
        stdout, stderr = proc.communicate()
        return proc.returncode, stdout, stderr, proc.pid

    def record(self):
        cmd_record = "docker exec rosbag bash -c 'ros2 bag record /ouster/points /ouster/imu /ouster/scan /odom'"
        returncode, stdout, stderr, pid = self.run_with_sudo(cmd_record)
        self.proc_record_pid = pid
        return returncode, stdout, stderr

    def stop(self):
        if self.proc_record_pid is not None:
            os.killpg(os.getpgid(self.proc_record_pid), signal.SIGTERM)
            self.proc_record_pid = None

    def convert(self):
        ros2bag_folder = self.get_latest_created_folder()
        if ros2bag_folder:
            cmd_convert = f"docker exec rosbag bash -c 'rosbags-convert --src {ros2bag_folder} --dst {ros2bag_folder}.bag'"
            return self.run_with_sudo(cmd_convert)
        return 1, b"", b"Folder not found"

    def test(self):
        return self.run_with_sudo("ls")
