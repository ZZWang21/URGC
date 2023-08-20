#!/usr/bin/env python

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from std_msgs.msg import String 
from geometry_msgs.msg import WrenchStamped, Vector3 #for publish force number

def set_target_wrench():
    # Create a WrenchStamped message
    wrench_msg = WrenchStamped()

    # Set the target wrench force values
    wrench_msg.wrench.force.x = 0.0
    wrench_msg.wrench.force.y = 0.0
    wrench_msg.wrench.force.z = -100.0  # Set the force to -100N

    # Publish the target_wrench message to the '/target_wrench' topic
    target_wrench_pub.publish(wrench_msg)
    #wrench_sub.unregister()  # Unsubscribe from the /wrench topic after setting target_wrench

def switch_controllers(start_controllers, stop_controllers):
    # Initialize ROS node
    #rospy.init_node('controller_switcher', anonymous=True)

    # Wait for the switch controller service to become available
    #rospy.wait_for_service('/controller_manager/switch_controller')
    rospy.wait_for_service('/controller_manager/switch_controller')

    try:
        # Create a service proxy for the switch controller service
        switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        

        # Build the request for the switch controller service
        #request = SwitchController()
        request = SwitchController._request_class()
        request.start_controllers = start_controllers
        request.stop_controllers = stop_controllers
        request.strictness = SwitchControllerRequest.BEST_EFFORT#'BEST_EFFORT'
        request.start_asap = True
        request.timeout = rospy.Duration.from_sec(0.0).to_sec() #rospy.Duration(0.0)

        # Call the switch controller service
        response = switch_controller(request)

        # Print the response
        if response.ok:
            rospy.loginfo("Controllers switched successfully!")
            print("Yes, controllers switched.")
        else:
            rospy.logerr("Failed to switch controllers. Error: %s", response.error_string)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))

#publish force to the topic
def publish_wrench():
    # Initialize ROS node
    #rospy.init_node('wrench_publisher', anonymous=True)

    # Create a publisher for the target wrench topic
    pub = rospy.Publisher('/target_wrench', WrenchStamped, queue_size=100)

    # Set the publishing rate (e.g., 10 Hz)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        # Create a message object
        message = WrenchStamped()
        message.header.seq = 0
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = ''  # Replace with the appropriate frame ID base_link

        # Set the force vector
        message.wrench.force = Vector3()
        message.wrench.force.x = 0.0
        message.wrench.force.y = 0.0
        message.wrench.force.z = -90.0

        # Set the torque vector
        message.wrench.torque = Vector3()
        message.wrench.torque.x = 0.0
        message.wrench.torque.y = 0.0
        message.wrench.torque.z = 0.0

        # Publish the message
        pub.publish(message)

        # Sleep to maintain the publishing rate
        rate.sleep()

def wrench_callback(data): #get the force value
    # This function will be called whenever new data is received on the '/wrench' topic
    force_z = data.wrench.force.z
    # Use the 'force_z' value here or perform any processing required
    print("Force Z:")
    if force_z < 0:
        print("force_z<0, It is time to switch to compliance")
        start_controllers = ['arm_controller_compliance']  # Modify with the controllers you want to start
        stop_controllers = ['arm_controller_force']
        switch_controllers(start_controllers, stop_controllers)
        #publish_wrench()
    else: 
        print("?")
        start_controllers = ['arm_controller_force']  # Modify with the controllers you want to start
        stop_controllers = ['arm_controller_compliance']
        switch_controllers(start_controllers, stop_controllers)

def main():
    rospy.init_node('wrench_subscriber', anonymous=True)
    # Initialize the publisher for '/target_wrench'
    global target_wrench_pub
    target_wrench_pub = rospy.Publisher('/target_wrench', WrenchStamped, queue_size=10)

    # Subscribe to the '/wrench' topic to read the measured data
    #global wrench_sub
    #wrench_sub = rospy.Subscriber('/wrench', WrenchStamped, wrench_callback)
    rospy.Subscriber('/wrench', WrenchStamped, wrench_callback)

    # Call the set_target_wrench function to set the target wrench force
    set_target_wrench()

    print("SO?")

    rospy.spin()

if __name__ == '__main__':
    try:
        # Specify the controllers to start and stop
        start_controllers = ['arm_controller_force']  # Modify with the controllers you want to start
        stop_controllers = ['arm_controller']  # Modify with the controllers you want to stop

        # Switch controllers
        switch_controllers(start_controllers, stop_controllers)

        #publish_wrench()

        main()

    except rospy.ROSInterruptException:
        pass
