#!/usr/bin/python

PKG = 'launch_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import subprocess
import signal
import shlex
import time

class Executer():
    """
    Simple class for executing a command in a subprocess.
    """
    def __init__(self, command, stdout, stderr):
        self.command = shlex.split(command)
        self.stdout = stdout
        self.stderr = stderr
    def execute(self):
        rospy.loginfo("Launching process: \"%s\"...", " ".join(self.command))
        rospy.loginfo("stdout/stderr: %s/%s", str(self.stdout), str(self.stderr))
        self.process = subprocess.Popen(self.command, stdout=self.stdout, stderr=self.stderr)
        rospy.loginfo("Process launched.")
    def interrupt(self):
        if self.process.poll() is None: # check if process is still running
            rospy.loginfo("Terminating process...")
            self.process.terminate()
            waiting_seconds = 0
            while self.process.poll() is None and waiting_seconds < 5:
                rospy.loginfo("Waiting for process to finish...")
                time.sleep(1)
                waiting_seconds = waiting_seconds + 1
            if self.process.poll() is None:
                rospy.loginfo("Killing process...")
                self.process.kill()
                self.process.wait()
        rospy.loginfo("Process has stopped with return code %i", self.process.returncode)
 
if __name__ == "__main__":
    rospy.init_node('executer', anonymous=True)
    command = rospy.get_param("~command")
    stdout_file = rospy.get_param("~stdout_file", "")
    stderr_file = rospy.get_param("~stderr_file", "")
    stdout = None if len(stdout_file) == 0 else open(stdout_file, 'w')
    stderr = None if len(stderr_file) == 0 else open(stderr_file, 'w')
    executer = Executer(command, stdout, stderr)
    executer.execute()
    rospy.on_shutdown(executer.interrupt)
    while executer.process.poll() is None and not rospy.is_shutdown():
        time.sleep(0.1)

