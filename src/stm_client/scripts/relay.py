#!/usr/bin/env python2

import rospy
from stm_client.srv import relay_control
#r1-7, r2-6, r3-5, r4-4, r5-3, r6-2


class Relay():
    def __init__(self):
        rospy.init_node("relay_client")
        self.relayKeys = {
            '0': 0,
            '1': 1,
            '2': 2,
            '3': 4,
            '4': 8,
            '5': 16,
            '6': 32,
            '7': 64,
            '8': 128,
        }

        self.relay_value = rospy.ServiceProxy(
            "/relay_toggle_channel", relay_control)

    def init_srv(self):
        rospy.logwarn("Waiting for server.")
        rospy.wait_for_service("relay_toggle_channel")
        rospy.loginfo("Server available.")
        print("------------------------------")
        print(
            "1 : umbratek\n2 : innfos\n3 : magnet\n4 : not_connected\n5 : bottom_light\n6 : pump\n7 : upper_led\n")
        print("------------------------------")

    def main(self, key):
        self.key = str(key)
        print("Key pressed: "+self.key)
        if self.key in self.relayKeys:
            self.val = self.relayKeys[self.key]
            self.response = self.relay_value(self.val)
            rospy.loginfo("Response: {}".format(self.response.response))
        print("################################")


if __name__ == "__main__":
    try:
        relay_twist = Relay()
        while not rospy.is_shutdown():
            relay_twist.init_srv()
            key = input("key: ")
            relay_twist.main(key)

    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" % e)


"""
self.relayKeys = {
            '1': "10000000",
            '2': "01000000",
            '3': "00100000",
            '4': "00010000",
            '5': "00001000",
            '6': "00000100",
            '7': "00000010",
            '8': "00000001",
        }
"""
