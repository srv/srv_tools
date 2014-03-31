#!/usr/bin/env python

"""
Copyright (c) 2013,
Systems, Robotics and Vision Group
University of the Balearican Islands
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Systems, Robotics and Vision Group, University of 
      the Balearican Islands nor the names of its contributors may be used to 
      endorse or promote products derived from this software without specific 
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""



import roslib; roslib.load_manifest('launch_tools')
import sys
import rospy
import rosservice
import threading


## Class for calling a service using a timer.
class TimedService(threading.Thread):

	## The constructor
	# @param self The object pointer.
	# @param name The service name this class is going to call
	# @param freq The desired timer period
	def __init__(self, name, period):
		threading.Thread.__init__(self)
		self._service_name = name
		self._service_period = period

	## Run function required by threading library
	def run(self):
		rospy.wait_for_service(self._service_name)
		rospy.Timer(rospy.Duration(self._service_period), self.callback)
		rospy.loginfo('Initialized timer for service: \n\t* Name: %s\n\t* Period: %f ', self._service_name, self._service_period)

	## Timer callback
	# @param event The event that has generated this callback
	def callback(self,event):
		rospy.wait_for_service(self._service_name)
		service_class = rosservice.get_service_class_by_name(self._service_name)
		try:
			service = rospy.ServiceProxy(self._service_name,service_class)
			service()
			rospy.loginfo('Service %s called.', self._service_name)
		except rospy.ServiceException, e:
			rospy.logwarn('Service %s call failed: %s',self._service_name,e)

	## @var _service_name 
	# The service name going to be called
	_service_name = "service"

	## @var _service_period
	# The timer period to call the service
	_service_period = 1.0

## Print usage for people that does not deserve to use this awesome python node.
def usage():
    return "%s service period [service period ...]"%sys.argv[0]

## main function
if __name__ == "__main__":
	rospy.init_node('services_timer')
	if len(sys.argv) >= 3:
		names = sys.argv[1:len(sys.argv):2]
		periods = sys.argv[2:len(sys.argv):2]
		rospy.loginfo('names   : %s', names)
		rospy.loginfo('periods : %s', periods)
		ts_list = []
		for name,period in zip(names,periods):
			ts_list.append(TimedService(str(name), float(period)))
		for ts in ts_list:
			ts.start()
	else:
		rospy.loginfo(usage())
		sys.exit(1)
	rospy.spin()