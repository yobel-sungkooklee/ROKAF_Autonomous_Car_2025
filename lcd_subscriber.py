#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import I2C_LCD_driver
import time

lcd = I2C_LCD_driver.lcd()

def cb(data):
    str1, str2 = data.data.split(",")
    lcd.lcd_clear()
    lcd.lcd_display_string(str1, 1)
    lcd.lcd_display_string(str2, 2)

    
def lcd_sub():

    rospy.init_node('lcd_subscriber', anonymous=True)

    rospy.Subscriber("/lcd_str_1", String, cb)

    rospy.spin()

if __name__ == '__main__':
    lcd_sub()