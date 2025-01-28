  #!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt


class quadrature_encoder:
    def __init__(self, serial_string_identifier, counts_per_encoder_revolution, gear_ratio, wheel_radius, forward_encoder_rotation_sign) :  
        # serial_string_identifier is e.g. "E0" or "E1"
        self.serial_string_identifier = serial_string_identifier  # does nothing, just an identifier that we hope the user will keep as a unique link to the sensor data streaming in. 
        self.counts_per_encoder_revolution = counts_per_encoder_revolution
        self.gear_ratio = gear_ratio
        self.wheel_radius = wheel_radius  # e.g. a wheel radius to get linear displacement and speed
        self.forward_encoder_rotation_sign = forward_encoder_rotation_sign
        
        self.counts_to_radians_multiplier = float(1)/self.counts_per_encoder_revolution / self.gear_ratio * 2*np.pi * self.forward_encoder_rotation_sign 
        
#        self.t = rospy.get_rostime()  # NOTE the ROS time function. Was time.time() for general python use. 
#        self.t_previous = self.t
        self.dt = 0.01 #(self.t - self.t_previous).to_sec()
        self.counts = 0
        self.counts_previous = 0
        self.radians = 0.
        self.meters = 0.
        self.counts_per_second = 0.
        self.radians_per_second = 0.
        self.meters_per_second = 0. 
        
        self.initializing = True  # this is a flag that will make "update" run twice (to set both position and per_second) the first time it is called. 

        
    def update(self, counts_measured, dt):
        self.dt = dt
        
        if self.initializing:
            self.counts_offset = counts_measured
            self.counts = counts_measured - self.counts_offset
            self.initializing = False
            return #    Prevent the rest of the function from running on the Initialization call. 
        
        self.counts_previous = self.counts
        self.counts = counts_measured - self.counts_offset
#        self.radians_previous = self.radians
        self.radians = self.counts * self.counts_to_radians_multiplier
#        self.meters_previous = self.meters 
        self.meters = self.radians * self.wheel_radius 
        
#        self.counts_per_second_previous = self.counts_per_second
        self.counts_per_second = float(self.counts - self.counts_previous)/self.dt
        # self.radians_per_second_previous = self.radians_per_second
        self.radians_per_second = self.counts_per_second * self.counts_to_radians_multiplier
        # self.meters_per_second_previous = self.meters_per_second
        self.meters_per_second = self.radians_per_second * self.wheel_radius 


# a general purpose FPID Velocity controller, to be instantiated in whatever way the user desires. 
class FPID_controller ():   
    def __init__(self, motor=[], Kf=0., Kp=1., Ki=0., Kd=0., error_integral_limit=np.inf, integral_resetting = True, motor_command_max_rate_of_change=100000., motor_command_max=480., forward_motor_command_sign=1): 
        self.motor = motor
        self.Kf = Kf
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.error_integral_limit = error_integral_limit
        self.integral_resetting = integral_resetting
        self.motor_command_max_rate_of_change = motor_command_max_rate_of_change
        self.motor_command_max = motor_command_max
        self.forward_motor_command_sign = forward_motor_command_sign
        
        # dummy values
        self.dt = 0.01     
        self.target = 0.
        self.error = 0.
        self.error_previous = 0.
        self.error_integral = 0.
        self.error_derivative = 0.
        
        # motor command
        self.motor_command = 0
        self.motor_command_previous = 0
        
        # angle values 
        self.pitch = 0
        self.pitch_target = 0.
        
        # calibration completion flag
        self.calibrated = False     
        
        # storing data for plot
        self.a=[]
        self.b=[]
        self.timenow = 0
        self.t =[]
        
    def compute_error(self):
        self.error_previous = self.error 
        self.error = self.target - self.current
        
    def compute_error_integral(self):
        self.error_integral += self.error * self.dt
        # apply the error_integral_limit
        self.error_integral = np.clip(self.error_integral, -self.error_integral_limit, self.error_integral_limit)
        # Special case of sitting still: do not allow Integral error to exist.
        if float(self.target) == 0. :
            self.error_integral = 0.
            
    def compute_error_derivative(self): 
        self.error_derivative = float(self.error - self.error_previous)/self.dt
        
    def compute_command(self): 
        ## With FeedForward Gain
        self.motor_command = self.forward_motor_command_sign * (self.Kf*self.target + self.Kp*self.error + self.Ki*self.error_integral + self.Kd*self.error_derivative)
    
    def limit_command_rate_of_change(self): 
        motor_command_rate_requested = (self.motor_command - self.motor_command_previous)/self.dt
        motor_command_rate = np.clip(motor_command_rate_requested, -self.motor_command_max_rate_of_change, self.motor_command_max_rate_of_change)
        self.motor_command = self.motor_command_previous + self.dt*motor_command_rate    

    def limit_command(self): 
        self.motor_command = np.clip(self.motor_command, -self.motor_command_max, self.motor_command_max) 
            
    def command_motor(self): 
        self.motor.setSpeed(int(self.motor_command))
        self.motor_command_previous = self.motor_command
        
    # Function to update the current value    
    def update_current_value(self, current_value, dt):
        self.current = current_value
        self.dt = dt
        self.compute_error()
        self.compute_error_integral()
        self.compute_error_derivative()
        self.compute_command()
        self.limit_command_rate_of_change()
        self.limit_command()
        self.timenow += self.dt
        #if(self.target != 0) and (self.current != 0):
        #    self.store_data()
        self.command_motor()
        
    def update_angle(self, pitch):
        self.pitch = pitch
        
    def update_pid(self, parameters):
        self.Kp = parameters[0]
        self.Ki = parameters[1]
        self.Kd = parameters[2]
        
    def update_target_value(self, target):
        if target != self.target:
            self.target = target
            if self.integral_resetting:
                self.error_integral = 0.    # At one time we would reset the error integral with every command, but that doesn't work when you reset the target speed frequently. Now only do it with new commands, and only if using integral_resetting
#            self.update()         

    def update_calibration(self, flag):
        self.calibrated = flag

    def store_data(self):
        self.a.append(self.current)
        self.b.append(self.target)
        self.t.append(self.timenow)
        
    def plot(self):
        plt.plot(self.t,self.a,self.t,self.b)
        plt.xlabel('Time(s)')
        plt.ylabel('Velocity')
        #plt.ylabel('X and Y Velocity Rotation(degrees)')
        plt.title('Encoder1_oscillations_velocity')
        plt.gca().legend(('current_velocity','target_velocity'))
        #plt.savefig('imu_oscillations.png')
        plt.show()
        plt.close()
        

# new FPID Velocity and angle double loop controller, to be instantiated in whatever way the user desires. 
class FPID_controller_with_angleloop ():   
    def __init__(self, motor=[], Kf=0., Kp=1., Ki=0., Kd=0., AngKp=1., AngKi=0., AngKd=0., error_integral_limit=np.inf, integral_resetting = True, motor_command_max_rate_of_change=100000., motor_command_max=480., forward_motor_command_sign=1): 
        self.motor = motor
        self.Kf = Kf
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.AngKp = AngKp
        self.AngKi = AngKi
        self.AngKd = AngKd
        self.error_integral_limit = error_integral_limit
        self.integral_resetting = integral_resetting
        self.motor_command_max_rate_of_change = motor_command_max_rate_of_change
        self.motor_command_max = motor_command_max
        self.forward_motor_command_sign = forward_motor_command_sign
        
        # dummy velocity values
        self.dt = 0.01    
        self.current = 0.
        self.target = 0.
        self.error = 0.
        self.error_previous = 0.
        self.error_integral = 0.
        self.error_derivative = 0.
        
        # motor command
        self.motor_command = 0
        self.motor_command_previous = 0
        
        # angle values 
        self.pitch = 0
        self.pitch_target = 0.
        self.pitch_error = 0.
        self.pitch_error_previous = 0.
        self.pitch_error_integral = 0.
        self.pitch_error_derivative = 0.
        
        # anlge calibration completion flag
        self.calibrated = False
        
        # storing data for plot
        self.a=[]
        self.b=[]
        self.timenow = 0
        self.t =[]
        self.pitch_set=[]
        self.pitch_target_set=[]
        self.command_set=[]
        
        # another way for interactive plotting, but not effective
        #plt.ion()
        #plt.show()
        
    def compute_error(self):
        self.error_previous = self.error 
        self.error = self.target - self.current
        
    def compute_error_integral(self):
        self.error_integral += self.error * self.dt
        # apply the error_integral_limit
        self.error_integral = np.clip(self.error_integral, -self.error_integral_limit, self.error_integral_limit)
        # Special case of sitting still: do not allow Integral error to exist.
        if float(self.target) == 0. :
            self.error_integral = 0.
            
    def compute_error_derivative(self): 
        self.error_derivative = float(self.error - self.error_previous)/self.dt
        
    def compute_command(self): 
        self.pitch_target = 1 * (self.Kf*self.target + self.Kp*self.error + self.Ki*self.error_integral + self.Kd*self.error_derivative)
        if(self.pitch <= 55 and self.pitch >= -55):
            self.pitch_error_previous = self.pitch_error 
            self.pitch_error = self.pitch_target + self.pitch       # change + - based on the sign of the angle, but need to slow down if tilted backwards & speed up if tilted forwards
            self.pitch_error_integral += self.pitch_error * self.dt
            if float(self.target) == 0. :
                self.pitch_error_integral = 0.
            self.pitch_error_derivative = float(self.pitch_error - self.pitch_error_previous)/self.dt
            self.motor_command = self.forward_motor_command_sign * (self.AngKp*self.pitch_error + self.AngKi*self.pitch_error_integral + self.AngKd*self.pitch_error_derivative)
        else:
            self.motor_command = 0
            
    def limit_command_rate_of_change(self): 
        motor_command_rate_requested = (self.motor_command - self.motor_command_previous)/self.dt
        motor_command_rate = np.clip(motor_command_rate_requested, -self.motor_command_max_rate_of_change, self.motor_command_max_rate_of_change)
        self.motor_command = self.motor_command_previous + self.dt*motor_command_rate    

    def limit_command(self): 
        self.motor_command = np.clip(self.motor_command, -self.motor_command_max, self.motor_command_max) 
            
    def command_motor(self): 
        self.motor.setSpeed(int(self.motor_command))
        self.motor_command_previous = self.motor_command
        
    # Function to update the current value    
    def update_current_value(self, current_value, dt):
        self.current = current_value
        self.dt = dt
        self.compute_error()
        self.compute_error_integral()
        self.compute_error_derivative()
        self.compute_command()
        self.limit_command_rate_of_change()
        self.limit_command()
        self.timenow += self.dt
        #if(self.target != 0) and (self.current != 0):
            #self.store_data()
        self.command_motor()
        
    def update_angle(self, pitch):
        self.pitch = pitch  
        
    def update_pid(self, parameters):
        self.Kp = parameters[0]
        self.Ki = parameters[1]
        self.Kd = parameters[2]
        self.AngKp = parameters[3]
        self.AngKi = parameters[4]
        self.AngKd = parameters[5]
        
    def update_target_value(self, target):
        if target != self.target:
            self.target = target
            if self.integral_resetting:
                self.error_integral = 0.    # At one time we would reset the error integral with every command, but that doesn't work when you reset the target speed frequently. Now only do it with new commands, and only if using integral_resetting
#            self.update()    

    def update_calibration(self, flag):
        self.calibrated = flag

    def store_data(self):
        if (self.calibrated == True):
            self.a.append(self.current)
            self.b.append(self.target)
            self.t.append(self.timenow)
            self.pitch_set.append(self.pitch)
            self.pitch_target_set.append(self.pitch_target)
            self.command_set.append(self.motor_command)
            # when using below interactive plotting, uncomment to use it (other methods such as rqt_plot are better)
#            if len(self.t) > 400:
#                self.a.pop(1)
#                self.b.pop(1)
#                self.t.pop(1)
#                self.pitch_set.pop(1)
#                self.pitch_target_set.pop(1)
#                self.command_set.pop(1)
#            if len(self.t) % 100 == 0:
#                self.interactive_plot()
        
    def plot(self):
        # method using normal plt, plotinng data points for certain duration
        plt.subplot(2, 1, 1)
        #plt.ylim(-0.1,0.8)
        #plt.plot(t,x)
        plt.plot(self.t,self.a,self.t,self.b)
        plt.xlabel('Time(s)')
        plt.ylabel('Velocity')
        #plt.ylabel('X and Y Velocity Rotation(degrees)')
        plt.title('Encoder_oscillations_velocity')
        plt.gca().legend(('current_velocity','target_velocity'))
        print(self.motor)
        plt.subplot(2, 1, 2)
        #plt.ylim(-150,350)
        plt.plot(self.t,self.pitch_set,self.t,self.pitch_target_set,self.t,self.command_set)
        plt.xlabel('Time(s)')
        plt.ylabel('Pitch Rotation(degrees)')
        plt.title('imu_oscillations_pitchAngle')
        plt.gca().legend(('current_pitch','calc_angle_target','motor_command'))
        plt.show()
        #plt.savefig('imu_oscillations.png')
        plt.close()
        
    def interactive_plot(self):
        # method using interactive plt
        plt.subplot(2, 1, 1)
        plt.plot(self.t,self.a,label = "current_velocity") 
        plt.plot(self.t,self.b,label = "target_velocity") 
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        plt.legend(by_label.values(), by_label.keys())
        plt.axis("equal")
        plt.xlabel('Time(s)')
        plt.ylabel('Velocity')
        plt.title('Encoder_oscillations_velocity')
        plt.subplot(2, 1, 2)
        plt.plot(self.t,self.pitch_set,label = "current_pitch") 
        plt.plot(self.t,self.pitch_target_set, label = "calc_angle_target") 
        plt.plot(self.t,self.command_set, label = "motor_command") 
        handles2, labels2 = plt.gca().get_legend_handles_labels()
        by_label2 = dict(zip(labels2, handles2))
        plt.legend(by_label2.values(), by_label2.keys())
        plt.title('imu_oscillations_pitchAngle')
        plt.draw()
        plt.pause(0.0001)