#!/usr/bin/env python3

import numpy as np
import rospy
import serial
import traceback 
from pololu_drv8835_rpi import motors  	# MAX_SPEED is 480 (hard-coded)

# IMPORT the "encoders_and_motors" module so we can call its pieces as functions. 
import encoders_and_motors_v01 as encmot


# IMPORT the custom messages: 
# we import it "from" the ROS package we created it in (here "me439robot") with an extension of .msg ...
# and actually import the message type by name (here "ME439SensorsRaw" and others)
from mobrob_util.msg import ME439SensorsRaw, ME439WheelSpeeds, ME439MotorCommands, ME439WheelAngles, ME439WheelDisplacements, WeeblebotAngleData, WeeblebotVelocityData
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

#==============================================================================
# # Get parameters from rosparam
#==============================================================================
counts_per_encoder_revolution = rospy.get_param('/counts_per_encoder_revolution')  # 12.0
gear_ratio = rospy.get_param('/gear_ratio') # standard ME439 robot: 120.0   
wheel_radius = rospy.get_param('/wheel_diameter_actual')/2.0 # 0.030 # meters

e0_direction_sign = rospy.get_param('/left_encoder_sign')
e1_direction_sign = rospy.get_param('/right_encoder_sign')
m0_direction_sign = rospy.get_param('/left_motor_sign')
m1_direction_sign = rospy.get_param('/right_motor_sign')

integral_error_max = rospy.get_param('/vel_integral_limit')
integral_resetting = rospy.get_param('/integral_resetting')
cmd_rate_of_change_max = rospy.get_param('/cmd_rate_of_change_max')
motor_command_max = rospy.get_param('/motor_command_max')

Kf0 = rospy.get_param('/vel_left_f')  # 
Kp0 = rospy.get_param('/vel_left_p') 
Ki0 = rospy.get_param('/vel_left_i')
Kd0 = rospy.get_param('/vel_left_d') 
Kf1 = rospy.get_param('/vel_right_f')  # 
Kp1 = rospy.get_param('/vel_right_p') 
Ki1 = rospy.get_param('/vel_right_i')
Kd1 = rospy.get_param('/vel_right_d') 

# Presumed viable PID value, needed further tuning
AngKp0=500.
AngKi0=400.
AngKd0=2.
AngKp1=500.
AngKi1=400.
AngKd1=2.

# Max encoder increment (full speed) - useful for eliminating errors
enc_increment_max = 1000

# flag calibration completion
calibrated = False
e0_complete = False
e1_complete = False

#==============================================================================
# # Set up a system to coordinate the motor closed-loop control
#==============================================================================
#   (Motor control is not ROS-friendly using ordinary messages due to delays.) 
def talker(): 
    # Launch a node called "sensing_and_wheel_control_node"
    rospy.init_node('sensing_and_wheel_control_node', anonymous=False)
        
#==============================================================================
#     # Create two Quadrature Encoder instances, one for each wheel 
#==============================================================================
    #   constructor function call: new_variable = encmot.quadrature_encoder(serial_string_identifier, counts_per_encoder_revolution, gear_ratio, wheel_radius, forward_encoder_rotation_sign)
    qe0 = encmot.quadrature_encoder("E0", counts_per_encoder_revolution,gear_ratio, wheel_radius, e0_direction_sign)
    qe1 = encmot.quadrature_encoder("E1",counts_per_encoder_revolution,gear_ratio, wheel_radius, e1_direction_sign)
    
#==============================================================================
#     # Create two motor controller instances, one for each wheel
#==============================================================================
    #   constructor function call: FPID_controller (): new_object = encmot.FPID_controller(motor,Kf, Kp,Ki,Kd, error_integral_limit=np.inf, integral_resetting = True, motor_command_max_rate_of_change=1500., forward_motor_command_sign=1)
    #   swith 1,2,3,4 for different mode of controller
    controller_mode = 3
    if controller_mode == 0:#controller mode: 0, speed single PID loop;
        mc0 = encmot.FPID_controller(motor=motors.motor1, Kf=Kf0, Kp=6500., Ki=3000, Kd=200, error_integral_limit = integral_error_max, integral_resetting = integral_resetting, motor_command_max_rate_of_change = cmd_rate_of_change_max, motor_command_max = motor_command_max, forward_motor_command_sign = m0_direction_sign)
        mc1 = encmot.FPID_controller(motor=motors.motor2, Kf=Kf1, Kp=6500., Ki=3000, Kd=150, error_integral_limit = integral_error_max, integral_resetting = integral_resetting, motor_command_max_rate_of_change = cmd_rate_of_change_max, motor_command_max = motor_command_max, forward_motor_command_sign = m1_direction_sign)
    elif controller_mode == 1:#controller mode: 1, speed and angle PID loop;
        mc0 = encmot.FPID_controller_with_angleloop(motor=motors.motor1, Kf=Kf0, Kp=400., Ki=200., Kd=3., AngKp=500., AngKi=400., AngKd=1.5, error_integral_limit = integral_error_max, integral_resetting = integral_resetting, motor_command_max_rate_of_change = cmd_rate_of_change_max, motor_command_max = motor_command_max, forward_motor_command_sign = m0_direction_sign)
        mc1 = encmot.FPID_controller_with_angleloop(motor=motors.motor2, Kf=Kf1, Kp=400., Ki=200., Kd=3., AngKp=500., AngKi=400., AngKd=1.5, error_integral_limit = integral_error_max, integral_resetting = integral_resetting, motor_command_max_rate_of_change = cmd_rate_of_change_max, motor_command_max = motor_command_max, forward_motor_command_sign = m1_direction_sign)
    elif controller_mode == 2:#controller mode: 2, speed single PID loop with tuning sliders;
        mc0 = encmot.FPID_controller(motor=motors.motor1, Kf=Kf0, Kp=Kp0, Ki=Ki0, Kd=Kd0, error_integral_limit = integral_error_max, integral_resetting = integral_resetting, motor_command_max_rate_of_change = cmd_rate_of_change_max, motor_command_max = motor_command_max, forward_motor_command_sign = m0_direction_sign)
        mc1 = encmot.FPID_controller(motor=motors.motor2, Kf=Kf1, Kp=Kp1, Ki=Ki1, Kd=Kd1, error_integral_limit = integral_error_max, integral_resetting = integral_resetting, motor_command_max_rate_of_change = cmd_rate_of_change_max, motor_command_max = motor_command_max, forward_motor_command_sign = m1_direction_sign)
    elif controller_mode == 3:#controller mode: 3, speed and angle PID loop with tuning sliders;
        mc0 = encmot.FPID_controller_with_angleloop(motor=motors.motor1, Kf=Kf0, Kp=Kp0, Ki=Ki0, Kd=Kd0, AngKp=AngKp0, AngKi=AngKi0, AngKd=AngKd0, error_integral_limit = integral_error_max, integral_resetting = integral_resetting, motor_command_max_rate_of_change = cmd_rate_of_change_max, motor_command_max = motor_command_max, forward_motor_command_sign = m0_direction_sign)
        mc1 = encmot.FPID_controller_with_angleloop(motor=motors.motor2, Kf=Kf1, Kp=Kp1, Ki=Ki1, Kd=Kd1, AngKp=AngKp1, AngKi=AngKi1, AngKd=AngKd1, error_integral_limit = integral_error_max, integral_resetting = integral_resetting, motor_command_max_rate_of_change = cmd_rate_of_change_max, motor_command_max = motor_command_max, forward_motor_command_sign = m1_direction_sign)

#==============================================================================
#     # Two subscriber to update pitch angle and PID parameters
#==============================================================================    
    # subscriber to update pitch
    sub_pitch = rospy.Subscriber('/robot_imu/imu/rpy/pitch', Float32, update_pitch,[mc0,mc1])  
    # subscriber to update PID tuning parameters from slider
    sub_pid = rospy.Subscriber('/pid_parameter', JointState, update_pid,[mc0,mc1])
    
#==============================================================================
#     # Here start a Subscriber to the "wheel_speeds_desired" topic.
#==============================================================================
    #   NOTE the Callback to the set_wheel_speed_targets function, which will update the setting of a motor controller that is called from the main loop below. 
    #   NOTE also the extra arguments to that callback: the Motor Encoders (both in a list)
    sub_wheel_speeds = rospy.Subscriber('/wheel_speeds_desired', ME439WheelSpeeds, set_wheel_speed_targets,[mc0,mc1])  
    

#==============================================================================
#     # Start the loop that listens and publishes. 
#==============================================================================
    # Give it an argument of the [qe0,qe1] encoders and the [mc0,mc1] motor controllers so it can update them all. 
    serial_port_publisher([qe0,qe1],[mc0,mc1])
    

    
#==============================================================================
# # Function to update the yaw angle of the robot
#==============================================================================
#   These commands come from the angle Subscriber above. 
def update_pitch(msg_in, motor_controllers):
    global calibrated
    calibrated = True
    #print(msg_in.data)
    motor_controllers[0].update_angle(msg_in.data)
    motor_controllers[1].update_angle(msg_in.data)
    
        
#==============================================================================
# # Function to update the pid tuning parameter on each controller
#==============================================================================
#   These commands come from the pid Subscriber above. 
def update_pid(msg_in, motor_controllers):
    global Kp0,Ki0,Kd0,Kp1,Ki1,Kd1,AngKp0,AngKi0,AngKd0,AngKp1,AngKi1,AngKd1
    #print(msg_in.position)
    motor_controllers[0].update_pid(msg_in.position)
    motor_controllers[1].update_pid(msg_in.position)
    

#==============================================================================
# # Function to send updated Commands directly to the motor controllers.
#==============================================================================
#   These commands come from the Subscriber above. 
def set_wheel_speed_targets(msg_in, motor_controllers):
    global e0_complete, e1_complete
    #print(msg_in.v_left)
    #print(msg_in.v_right)
    if(motor_controllers[0].target != 0) and (msg_in.v_left == 0):
        e0_complete = True
    if(motor_controllers[1].target != 0) and (msg_in.v_right == 0):
        e1_complete = True    
    motor_controllers[0].update_target_value(msg_in.v_left)
    motor_controllers[1].update_target_value(msg_in.v_right)
    
#==============================================================================
# # Main loop function that listens to the serial port, calls motor control, and publishes the sensor data
#==============================================================================
def serial_port_publisher(quad_encoders, mot_controllers):
    global e0_complete, e1_complete, calibrated
# Create the publisher for the topic "/sensors_data_raw", with message type "ME439SensorsRaw"
    pub_sensors = rospy.Publisher('/sensors_data_raw', ME439SensorsRaw, queue_size=10)
# Create the publisher for the topic "/motor_commands", with message type "ME439MotorCommands"
    pub_motorcommands = rospy.Publisher('/motor_commands', ME439MotorCommands, queue_size=10)
# Create the publisher for the topic "/robot_wheel_angles", with message type "ME439WheelAngles"    
    pub_robot_wheel_angles = rospy.Publisher('/robot_wheel_angles', ME439WheelAngles, queue_size = 10)
# Create the publisher for the topic "/robot_wheel_displacements", with message type "ME439WheelDisplacements"    
    pub_robot_wheel_displacements = rospy.Publisher('/robot_wheel_displacements', ME439WheelDisplacements, queue_size = 10)
# Create the publisher for the topic "/robot_velocity_data", with message type "WeeblebotVelocityData"    
    pub_robot_velocity_data = rospy.Publisher('/robot_velocity_data', WeeblebotVelocityData, queue_size = 1)
# Create the publisher for the topic "/robot_angle_data", with message type "WeeblebotAngleData"    
    pub_robot_angle_data = rospy.Publisher('/robot_angle_data', WeeblebotAngleData, queue_size = 1)
# Create the publisher for the each topic on robot_velocity_data with message type "Float32"
    pub_v_current_left = rospy.Publisher('/plot/v_current_left', Float32, queue_size=1)
    pub_v_target_left = rospy.Publisher('/plot/v_target_left', Float32, queue_size=1)
    pub_v_current_right = rospy.Publisher('/plot/v_current_right', Float32, queue_size=1)
    pub_v_target_right = rospy.Publisher('/plot/v_target_right', Float32, queue_size=1)
# Create the publisher for the each topic on robot_velocity_data with message type "Float32"
    pub_pitch_current_left = rospy.Publisher('/plot/pitch_current_left', Float32, queue_size=1)
    pub_pitch_target_left = rospy.Publisher('/plot/pitch_target_left', Float32, queue_size=1)
    pub_motor_command_left = rospy.Publisher('/plot/motor_command_left', Float32, queue_size=1)
    pub_pitch_current_right = rospy.Publisher('/plot/pitch_current_right', Float32, queue_size=1)
    pub_pitch_target_right = rospy.Publisher('/plot/pitch_target_right', Float32, queue_size=1)
    pub_motor_command_right = rospy.Publisher('/plot/motor_command_right', Float32, queue_size=1)
    
    # Data comes in on the Serial port. Set that up and start it. 
    #----------setup serial--------------
    ser = serial.Serial('/dev/ttyUSB0')  #serial port to alamode
    ser.baudrate = 115200 
    ser.bytesize = 8
    ser.parity = 'N'
    ser.stopbits = 1
    ser.timeout = 1 # one second time out.

    ser.flushInput()
    ser.readline()
    
    
    # Declare the message that will go on that topic. 
    # Here we use one of the message name types we Imported, and add parentheses to call it as a function. 
    # We could also put data in it right away - here initializing to zero. 
    pub_sensors_message = ME439SensorsRaw()
    pub_sensors_message.e0 = 0
    pub_sensors_message.e1 = 0
    pub_sensors_message.a0 = 0
    pub_sensors_message.a1 = 0
    pub_sensors_message.a2 = 0
    pub_sensors_message.a3 = 0
    pub_sensors_message.a4 = 0
    pub_sensors_message.a5 = 0
    pub_sensors_message.u0 = 0
    pub_sensors_message.u1 = 0
    pub_sensors_message.u2 = 0
    
    pub_motorcommands_message = ME439MotorCommands()
    pub_motorcommands_message.cmd0 = 0
    pub_motorcommands_message.cmd1 = 0

    robot_wheel_angles_message = ME439WheelAngles()
    robot_wheel_angles_message.ang_left = 0. 
    robot_wheel_angles_message.ang_right = 0. 

    robot_wheel_displacements_message = ME439WheelDisplacements()
    robot_wheel_displacements_message.d_left = 0.
    robot_wheel_displacements_message.d_right = 0.
    7
    robot_velocity_data_message = WeeblebotVelocityData()
    robot_velocity_data_message.t_left = 0. 
    robot_velocity_data_message.v_current_left = 0. 
    robot_velocity_data_message.v_target_left = 0. 
    robot_velocity_data_message.t_right = 0. 
    robot_velocity_data_message.v_current_right = 0. 
    robot_velocity_data_message.v_target_right = 0. 

    robot_angle_data_message = WeeblebotAngleData()
    robot_angle_data_message.t_left = 0.
    robot_angle_data_message.pitch_current_left = 0.
    robot_angle_data_message.pitch_target_left = 0.
    robot_angle_data_message.motor_command_left = 0.
    robot_angle_data_message.t_right = 0.
    robot_angle_data_message.pitch_current_right = 0.
    robot_angle_data_message.pitch_target_right = 0.
    robot_angle_data_message.motor_command_right = 0.
    
    t0_previous = t1_previous = rospy.get_rostime()
    
    init0 = 1
    init1 = 1
    n_e0 = 0
    n_e1 = 0
    # MAIN LOOP to keep loading the message with new data. 
    # NOTE that at the moment the data are coming from a separate thread, but this will be replaced with the serial port line reader in the future. 
    while not rospy.is_shutdown():
        # set all the "new data" variables to zero. 
        newe0 = 0
        newe1 = 0
        newa0 = 0
        newa1 = 0
        newa2 = 0
        newa3 = 0
        newa4 = 0
        newa5 = 0
        newu0 = 0
        newu1 = 0
        newu2 = 0
        
#        new_data_packet = 0
        try: 
            # Here we read the serial port for a string that looks like "e0:123456", which is an Encoder0 reading. 
            # When we get a reading, update the associated motor command
            line = ser.readline().decode().strip() #blocking function, will wait until read entire line
#            print(line)
            #rospy.loginfo(line)
            line = line.split(":")
            data_type = line[0]
            data_value = int(line[1])
#            print(data_type)
#            print(line)
            if data_type == 'E0':    # only use it as an encoder0 reading if it is one. 
                if init0 or np.abs(int(data_value)-e0) < enc_increment_max:  # runs if it's the initialization step or if the encoder increment is reasonable (not obviously erroneous)
                    e0 = data_value	# Here is the actual encoder reading. 
                    init0 = 0
    
                    pub_sensors_message.e0 = e0
                    t0 = rospy.get_rostime()   # or time.time()
                    dt0 = (t0-t0_previous).to_sec()
                    t0_previous = t0 # store it for the next round
                    quad_encoders[0].update(e0, dt0)
                    mot_controllers[0].update_calibration(calibrated)
                    mot_controllers[0].update_current_value(quad_encoders[0].meters_per_second, dt0) 
                    #print("E0 Target: " + str(mot_controllers[0].target))
                    #print("E0 command: " + str(mot_controllers[0].motor_command))
                    pub_motorcommands_message.cmd0 = int(mot_controllers[0].motor_command)
                    robot_wheel_angles_message.ang_left = quad_encoders[0].radians
                    robot_wheel_displacements_message.d_left = quad_encoders[0].meters
                    # Below was used for interactive tuning using data_plot.py, but not yet working
                    #robot_velocity_data_message.t_left = mot_controllers[0].timenow
                    #robot_velocity_data_message.v_current_left = mot_controllers[0].current
                    #robot_velocity_data_message.v_target_left = mot_controllers[0].target
                    #robot_angle_data_message.pitch_current_left = mot_controllers[0].pitch
                    #robot_angle_data_message.pitch_target_left = mot_controllers[0].pitch_target
                    #robot_angle_data_message.motor_command_left = mot_controllers[0].motor_command
                    newe0 = 1   # set a flag that says this is a new encoder reading
    #                print(e0)
            elif data_type == 'E1':    #only use it as an encoder1 reading if it is one. 
                if init1 or np.abs(int(data_value)-e1) < enc_increment_max:  # runs if it's the initialization step or if the encoder increment is reasonable (not obviously erroneous)
                    e1 = data_value	# Here is the actual encoder reading. 
                    init1 = 0
                    
                    pub_sensors_message.e1 = e1
                    t1 = rospy.get_rostime()   # or time.time()
                    dt1 = (t1-t1_previous).to_sec()
                    t1_previous = t1 # store it for the next round
                    ## ******* Update the 
                    quad_encoders[1].update(e1, dt1)
                    mot_controllers[1].update_calibration(calibrated)
                    mot_controllers[1].update_current_value(quad_encoders[1].meters_per_second, dt1)
                    #print("E1 Target: " + str(mot_controllers[1].target))
                    #print("E1 command: " + str(mot_controllers[1].motor_command))
                    pub_motorcommands_message.cmd1 = int(mot_controllers[1].motor_command)
                    robot_wheel_angles_message.ang_right = quad_encoders[1].radians
                    robot_wheel_displacements_message.d_right = quad_encoders[1].meters
                    # Below was used for interactive tuning using data_plot.py, but not yet working
                    #robot_velocity_data_message.t_right = mot_controllers[1].timenow
                    #robot_velocity_data_message.v_current_right = mot_controllers[1].current
                    #robot_velocity_data_message.v_target_right = mot_controllers[1].target
                    #robot_angle_data_message.pitch_current_right = mot_controllers[1].pitch
                    #robot_angle_data_message.pitch_target_right = mot_controllers[1].pitch_target
                    #robot_angle_data_message.motor_command_right = mot_controllers[1].motor_command
                    newe1 = 1   # set a flag that says this is a new encoder reading
    #                print(cnt1)
            elif data_type == 'A0':
                pub_sensors_message.a0 = data_value
                newa0 = 1
#                print(a0)
#                print("A0 = {0}").format(a0)
            elif data_type == 'A1':
                pub_sensors_message.a1 = data_value
                newa1 = 1
#                print(a1)
            elif data_type == 'A2':
                pub_sensors_message.a2 = data_value
                newa2 = 1
#                print(a2)
            elif data_type == 'A3':
                pub_sensors_message.a3 = data_value
                newa3 = 1
#                print(a3)
            elif data_type == 'A4':
                pub_sensors_message.a4 = data_value
                newa4 = 1
#                print(a4)
            elif data_type == 'A5':
                pub_sensors_message.a5 = data_value
                newa5 = 1
#                print(a5)
            elif data_type == 'U0':
                pub_sensors_message.u0 = data_value
                newu0 = 1
#                print(u0)
            elif data_type == 'U1':
                pub_sensors_message.u1 = data_value
                newu1 = 1
#                print(u1)
            elif data_type == 'U2':
                pub_sensors_message.u2 = data_value
                newu2 = 1
#                print(u2)
            
#==============================================================================
#             # Publish a Message IFF it is full (i.e., all signals have been read including u2: u2 is the last)
#==============================================================================
            # Other logic could also be applied
            if newe1:
                pub_sensors_message.t = rospy.get_rostime()
                
                # Publish a message to the "/sensors_data_raw" topic.
                pub_sensors.publish(pub_sensors_message)
                # Publish a message to the "/motor_commands" topic. 
                pub_motorcommands.publish(pub_motorcommands_message)
                # Publish a message to the "/robot_wheel_angles" topic
                pub_robot_wheel_angles.publish(robot_wheel_angles_message)
                # Publish a message to the "/robot_wheel_displacements" topic
                pub_robot_wheel_displacements.publish(robot_wheel_displacements_message)

                # Below is used for interactive tuning using rqt_plot
                # Publish each data of robot velocity
                pub_v_current_left.publish(mot_controllers[0].current)
                pub_v_target_left.publish(mot_controllers[0].target)
                pub_v_current_right.publish(mot_controllers[1].current)
                pub_v_target_right.publish(mot_controllers[1].target)
                # Publish each data of robot angle
                pub_pitch_current_left.publish(mot_controllers[0].pitch)
                pub_pitch_target_left.publish(mot_controllers[0].pitch_target)
                pub_motor_command_left.publish(mot_controllers[0].motor_command)
                pub_pitch_current_right.publish(mot_controllers[1].pitch)
                pub_pitch_target_right.publish(mot_controllers[1].pitch_target)
                pub_motor_command_right.publish(mot_controllers[1].motor_command) 
                
                # Below was used for interactive tuning using data_plot.py, but not yet working
                # Publish a message to the "/robot_velocity_data" topic
                #pub_robot_velocity_data.publish(robot_velocity_data_message)
                # Publish a message to the "/robot_angle_data" topic
                #pub_robot_angle_data.publish(robot_angle_data_message)
                
                # Log the info (optional)
                #rospy.loginfo(pub_sensors)       
            
            # flags for plotting data, when motor control completes / was replaced later with interactive plotting
            if(e0_complete == True):
                n_e0 += 1
                if n_e0 > 500:
                    #mot_controllers[0].plot()
                    e0_complete = False
            if(e1_complete == True):
                n_e1 += 1
                if n_e1 > 500:
                    #mot_controllers[1].plot()
                    e1_complete = False
        
        
        except Exception:
            traceback.print_exc()
            pass
            



if __name__ == '__main__':
    try: 
        rospy.loginfo("starting sensing and wheel node")
        talker()
    except rospy.ROSInterruptException: 
        traceback.print_exc()
        # Stop the wheels if this crashes or otherwise ends. 
        #stop_wheel_speeds_message = ME439WheelSpeeds()
        #stop_wheel_speeds_message.v_left = stop_wheel_speeds_message.v_right = 0
        #set_wheel_speed_targets(stop_wheel_speeds_message, [motors.motor1,motors.motor2])
#        motors.motor1.setSpeed(0)
#        motors.motor2.setSpeed(0)
        pass
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)