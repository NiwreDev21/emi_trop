#!/usr/bin/env python3
import rospy
import random
from gazebo_msgs.msg import Wind

rospy.init_node("random_wind_controller")
pub = rospy.Publisher("/gazebo/wind", Wind, queue_size=10)

rate = rospy.Rate(1)  # cambiar cada 1 segundo

while not rospy.is_shutdown():
    wind = Wind()
    wind.wind_linear_velocity.x = random.uniform(-5, 5)  # m/s
    wind.wind_linear_velocity.y = random.uniform(-5, 5)
    wind.wind_linear_velocity.z = random.uniform(0, 2)
    pub.publish(wind)
    rospy.loginfo(f"🌬 Viento: {wind.wind_linear_velocity}")
    rate.sleep()
