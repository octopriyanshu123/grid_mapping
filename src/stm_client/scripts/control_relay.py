#!/usr/bin/env python3


import rospy
from stm_client.srv import relay_control
from stm_client.msg import cam_array


# import keyboard
from pynput import keyboard
import threading  # Needed for Timer

import time
import sys

# r1-7, r2-6, r3-5, r4-4, r5-3, r6-2


class Relay:
    def __init__(self):
        rospy.init_node("relay_client")
        self.relay_press = 0 
        self.exit_press = 0
        self.gimble_press = 0
        self.relay_key = "0"
        self.exit_counter=0
        self.relayKeys = {
            "0": 0,
            "4": 4,
            "5": 5,
            "6": 6,
            "7": 7,
            "8": 8,
            "9": 9,
        }
        self.relay_value = rospy.ServiceProxy("/relay_toggle_channel", relay_control)

        # rospy.init_node("Camera_publisher")
        rospy.set_param("camera_frequency_1", 1)
        rospy.set_param("camera_frequency_2", 1)

        self.cam_pub = rospy.Publisher("/gimbal_twist", cam_array, queue_size=1)
        self.cam_data = cam_array()
        self.cam_data.data = [0, 0]


        with keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        ) as listener:
            listener.join()

        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

    def init_srv(self):
        try:
            if self.exit_counter==5:
                sys.exit()
            rospy.logwarn("Waiting for server.")
            rospy.wait_for_service("relay_toggle_channel", 1)
            rospy.loginfo("Server available.")
            print("------------------------------")
            print(
                "4 : pump\n5 : bottom_led\n6 : upper_led\n7 : magnet\n8 : arm\n9 : crawler\n"
            )
            print("------------------------------")
        except Exception as e:
            self.exit_counter+=1
            rospy.logerr(e)

    def on_press(self, key):
        try:
            # self.timer.cancel()
            self.relay_toggle(key)
            self.gimble_key_press(key)
            # self.timer.start()
        except Exception as e:
            self.relay_press = 0
            self.relay_key = "0"
            print(e)

    def on_release(self, key):
        self.quit_sequence(key)
        self.gimble_key_release(key)

        if self.exit_press == 4:
            self.exit_press = 0
            print("Succesfully exited")
            return False

    def relay_toggle(self, key):
        if (self.relay_press == 0) and (
            (key == keyboard.Key.ctrl) or (key == keyboard.Key.ctrl_r)
        ):
            # print("ctrl pressed")
            self.init_srv()
            self.relay_press = 1
        elif (self.relay_press == 2) and (key == keyboard.Key.enter):
            if self.relay_key != "0":
                self.main(self.relay_key)
            self.relay_press = 0
            self.relay_key = "0"
        elif (self.relay_press == 1) and (key.char in self.relayKeys):
            self.relay_key = key.char
            self.relay_press = 2
        else:
            self.relay_press = 0
            self.relay_key = "0"

    def gimble_key_press(self, key):

        if self.gimble_press == 2:
            # base
            if key == keyboard.Key.left:
                self.cam_data.data = [1, 0]
            elif key == keyboard.Key.right:
                self.cam_data.data = [-1, 0]
            # angle
            if key == keyboard.Key.up:
                self.cam_data.data = [0, 1]
            elif key == keyboard.Key.down:
                self.cam_data.data = [0, -1]

            self.cam_pub.publish(self.cam_data)

        if (self.gimble_press == 0) and (
            (key == keyboard.Key.alt) or (key == keyboard.Key.alt_r)
        ):
            # rospy.loginfo("Inside alt gimbal")
            self.gimble_press = 1

        elif (self.gimble_press == 1) and (
            (key == keyboard.Key.shift) or (key == keyboard.Key.shift_r)
        ):
            # rospy.loginfo("Inside shift gimbal")
            self.gimble_press = 2

    def gimble_key_release(self, key):
        if (
            (key == keyboard.Key.left)
            or (key == keyboard.Key.right)
            or (key == keyboard.Key.up)
            or (key == keyboard.Key.down)
        ):
            self.gimble_press = 0

    def quit_sequence(self, key):
        if (self.exit_press == 0) and (key == keyboard.Key.ctrl):
            self.exit_press = 1
        elif (self.exit_press == 1) and (key == keyboard.Key.shift):
            self.exit_press = 2
        elif (self.exit_press == 2) and (key == keyboard.Key.ctrl):
            self.exit_press = 3
        elif (self.exit_press == 3) and (key == keyboard.Key.shift):
            self.exit_press = 4
        else:
            self.exit_press = 0



    def twist_callback(self):

        self.cam_freq_1 = rospy.get_param("/camera_frequency_1")
        self.cam_freq_2 = rospy.get_param("/camera_frequency_2")

    def main(self, key):
        self.key = str(key)
        print("Key pressed: " + self.key)
        if self.key in self.relayKeys:
            self.val = self.relayKeys[self.key]
            self.relay_feedback = self.relay_value(self.val)
            rospy.loginfo("Response: {}".format(self.relay_feedback.response))
        print("################################")


if __name__ == "__main__":
    try:
        Relay()
        

    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" % e)


