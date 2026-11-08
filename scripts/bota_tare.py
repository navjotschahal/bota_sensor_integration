#!/usr/bin/env python3
"""Call the running node's tare service:  rosrun bota_ft_sensor bota_tare.py [/bota/tare]"""
import sys
import rospy
from std_srvs.srv import Trigger

if __name__ == "__main__":
    name = sys.argv[1] if len(sys.argv) > 1 else "/bota/tare"
    rospy.init_node("bota_tare", anonymous=True)
    rospy.wait_for_service(name, timeout=5.0)
    r = rospy.ServiceProxy(name, Trigger)()
    print(("OK: " if r.success else "FAILED: ") + r.message)
    sys.exit(0 if r.success else 1)
