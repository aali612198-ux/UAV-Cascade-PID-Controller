# Import any necessary libraries
import numpy as np

max_vel = 8 # m/s

#Limited to 4 ~ only tuned up to 3
x_layers_of_cascade = 3
y_layers_of_cascade = 3
z_layers_of_cascade = 3


## PID Tuning Parameters ##

# X Direction - Manual Tuning

# 4th layer
x_Kp_pos = 1.45
x_Ki_pos = 0.03
x_Kd_pos = 0.2
x_Kisat_pos = 0.5

# 3rd layer
x_Kp_jer = 0.1
x_Ki_jer = 0.03
x_Kd_jer = 0.01
x_Kisat_jer = 0.1

# 2nd layer
x_Kp_acc = 35
x_Ki_acc = 0.05
x_Kd_acc = 2.5
x_Kisat_acc = 0.01

# 1st layer
x_Kp_vel = 1.7 # 2.1 for a faster looser response 
x_Ki_vel = 0.001
x_Kd_vel = 0.25
x_Kisat_vel = 0.01



# Y Direction - Manual Tuning - All Equal to X values

# 4th layer
y_Kp_pos = x_Kp_pos
y_Ki_pos = x_Ki_pos
y_Kd_pos = x_Kd_pos
y_Kisat_pos = x_Kisat_pos

# 3rd layer
y_Kp_jer = x_Kp_jer
y_Ki_jer = x_Ki_jer
y_Kd_jer = x_Kd_jer
y_Kisat_jer = x_Kisat_jer

# 2nd layer
y_Kp_acc = x_Kp_acc
y_Ki_acc = x_Ki_acc
y_Kd_acc = x_Kd_acc
y_Kisat_acc = x_Kisat_acc

# 1st layer
y_Kp_vel = x_Kp_vel
y_Ki_vel = x_Ki_vel
y_Kd_vel = x_Kd_vel
y_Kisat_vel = x_Kisat_vel




# Z Direction - Ziegler Nichols Tuning

# 4th layer
z_Kp_pos = 1.45
z_Ki_pos = 0.05
z_Kd_pos = 0.2
z_Kisat_pos = 0.5

# 3rd layer
z_Kp_jer = 3.176470588235294
z_Ki_jer = 0.257096700000000
z_Kd_jer = 0.064274175000000
z_Kisat_jer = 0.5

# 2nd layer
z_Kp_acc = 7.941176470588236
z_Ki_acc = 0.411812735294118
z_Kd_acc = 0.102953183823529
z_Kisat_acc = 0.5

# 1st layer
z_Kp_vel = 11.764705882352942
z_Ki_vel = 1.122574705882353
z_Kd_vel = 0.280643676470588
z_Kisat_vel = 0.5


# Yaw - Manual Tuning
Kp_yaw = 1
Ki_yaw = 0.05
Kd_yaw = 0.1
Kisat_yaw = 0.5



class PID:
    def __init__(self, Kp, Ki, Kd, Kisat):

        # Define variables
        self.Kp = np.array(Kp) # Proportional gain
        self.Ki = np.array(Ki) # Integral gain
        self.Kd = np.array(Kd) # Derivative gain
        self.Kisat = np.array(Kisat) # Saturation of integral error - to prevent integral windup
        
        # Initialise Variables
        self.integral_error = np.zeros_like(Kp, dtype=float)
        self.derivative_error = np.zeros_like(Kp)
        self.previous_error = np.zeros_like(Kp)

    # Reset method
    def reset(self):
        self.integral_error = np.zeros_like(self.integral_error)
        self.previous_error = np.zeros_like(self.previous_error)

    # Get PID output
    def getPID_output(self, error, dt):
        
        # Define the proportional error 
        self.proportional_error = error

        # Find the integral error
        # Sum of error and previous error times time-step
        self.integral_error += error*dt # equivalent mathematically to int_err = int_err + err*dt

        # Prevent integral wind up by clipping the integral error
        min_integral_error = np.negative(self.Kisat)
        max_integral_error = np.positive(self.Kisat)
        self.integral_error = np.clip(self.integral_error, min_integral_error, max_integral_error)
        
        # Find the derivative error
        self.derivative_error = (error - self.previous_error)/dt

        # Compute the PID output
        PID_output = (self.Kp * self.proportional_error +
                      self.Ki * self.integral_error +
                      self.Kd * self.derivative_error)
        
        # Update the previous error for the next iteration
        self.previous_error = error
        return PID_output
    
    ## Cascade PID Controller ##
    def do_4layer_cascade(prev_pos, prev_vel, tar_pos, pos, pos_PID, vel_PID ,acc_PID, jer_PID, dt):
        # Calculate the positional error
        pos_error = tar_pos - pos

        # Get positional error from PID
        pos_error_from_PID = pos_PID.getPID_output(pos_error, dt)

        # Get velocity from PID
        vel_global = vel_PID.getPID_output(pos_error_from_PID, dt)

        # calculate the actual velocity
        vel_measured = (pos - prev_pos)/dt

        # calculate the veloctiy error
        vel_error =  vel_global - vel_measured
        
        # Get acceleration from PID
        acc_global = acc_PID.getPID_output(vel_error, dt)

        # Calculate the actual acceleration
        acc_measured = (x_vel_global - prev_vel)/dt
        
        # calculate the acceleration error
        acc_error = acc_global - acc_measured

        # Get jerk from PID
        jer_correction = jer_PID.getPID_output(acc_error, dt)

        # Calculate acceleration and velocity
        acc_global += jer_correction * dt # a = a + jerk * dt
        vel_global += acc_global * dt # v = v + a * dt

        # Constrain to the drones known maximum velocity
        # I will explain clip once but it is used more later
        # np.clip just clips the values to be with in the range you specify following the input
        # In this case -max_vel and +max_vel
        vel_global = np.clip(vel_global, -max_vel, max_vel)

        # Return the output values
        return(pos, vel_global, pos_error, pos_error_from_PID, vel_error, acc_error, jer_correction)

    def do_3layer_cascade(prev_pos, prev_vel, tar_pos, pos, vel_PID ,acc_PID, jer_PID, dt):
        # Calculate the positional error
        pos_error = tar_pos - pos

        # Get velocity from PID
        vel_global = vel_PID.getPID_output(pos_error, dt)

        # Calculate the actual velocity
        vel_measured = (pos - prev_pos)/dt
    
        # Calculate the velocity error
        vel_error =  vel_global - vel_measured
        
        # Get acceleration from PID
        acc_global = acc_PID.getPID_output(vel_error, dt)

        # Calculate the actual acceleration
        acc_measured = (x_vel_global - prev_vel)/dt

        # Calculate the acceleration error
        acc_error = acc_global - acc_measured

        # Get jerk from PID
        jer_correction = jer_PID.getPID_output(acc_error, dt)

        # Calculate acceleration and velocity
        acc_global += jer_correction * dt # a = a + jerk * dt
        vel_global += acc_global * dt # v = v + a * dt

        # Constrain to the drones known maximum velocity
        vel_global = np.clip(vel_global, -max_vel, max_vel)

        # Return the output values
        return(pos, vel_global, pos_error, vel_error, acc_error, jer_correction)
    

    def do_2layer_cascade(prev_pos, prev_vel, tar_pos, pos, vel_PID, acc_PID, dt):

        # Calculate the positional error
        pos_error = tar_pos - pos

        # Get velocity from PID
        vel_global = vel_PID.getPID_output(pos_error, dt)

        # Calculate the actual velocity
        vel_measured = (pos - prev_pos)/dt

        # Calculate the velocity error
        vel_error =  vel_global - vel_measured

        # Get velocity from PID
        vel_global = vel_PID.getPID_output(pos_error, dt)

        # calculate the actual velocity
        vel_measured = (pos - prev_pos)/dt

        # calculate the veloctiy error
        vel_error =  vel_global - vel_measured
        
        # Get acceleration from PID
        acc_global = acc_PID.getPID_output(vel_error, dt)

        # Calculate the measured acceleration
        acc_measured = (x_vel_global - prev_vel)/dt

        # Calculate the acceleration error
        acc_error = acc_global - acc_measured

        # Calculate velocity
        vel_global += acc_global * dt

        # Constrain to the drones known maximum velocity
        vel_global = np.clip(vel_global, -max_vel, max_vel)
        
        # Return the output values
        return(pos, vel_global, pos_error, vel_error, acc_error)
    
    def do_1layer_cascade(prev_pos, tar_pos, pos, vel_PID, dt):
        # Having only one layer of cascade means that this controller is simply doing the processing after the PID controller
        # ie -> this bit is not actually a cascade controller I was just using a similar name so it flows logically
        # Calculate the positional error
        pos_error = tar_pos - pos

        # Get velocity from PID
        vel_global = vel_PID.getPID_output(pos_error, dt)

        # calculate the actual velocity
        vel_measured = (pos - prev_pos)/dt

        # calculate the veloctiy error
        vel_error =  vel_global - vel_measured

        # Constrain to the drones known maximum velocity
        vel_global = np.clip(vel_global, -max_vel, max_vel)

        # Return the output values
        return(pos, vel_global, pos_error, vel_error)



# Initialise the functions with the relevant variables so that when we call the methods the parameters are already defined
# What this is doing is defining Kp, Ki, Kd, Ksat for the class and assigning the class a new name

# X Direction PID Controllers

x_jer_PID = PID(
                Kp    = x_Kp_jer,
                Ki    = x_Ki_jer,
                Kd    = x_Kd_jer,
                Kisat = x_Kisat_jer,)  

x_acc_PID = PID(
                Kp    = x_Kp_acc,
                Ki    = x_Ki_acc,
                Kd    = x_Kd_acc,
                Kisat = x_Kisat_acc,)  

x_vel_PID = PID(
                Kp    = x_Kp_vel,
                Ki    = x_Ki_vel,
                Kd    = x_Kd_vel,
                Kisat = x_Kisat_vel)  

x_pos_PID = PID(
                Kp    = x_Kp_pos,
                Ki    = x_Ki_pos,
                Kd    = x_Kd_pos,
                Kisat = x_Kisat_pos)  

# Y Direction PID Controllers

y_jer_PID = PID(
                Kp    = y_Kp_jer,
                Ki    = y_Ki_jer,
                Kd    = y_Kd_jer,
                Kisat = y_Kisat_jer,)  

y_acc_PID = PID(
                Kp    = y_Kp_acc,
                Ki    = y_Ki_acc,
                Kd    = y_Kd_acc,
                Kisat = y_Kisat_acc,)  

y_vel_PID = PID(
                Kp    = y_Kp_vel,
                Ki    = y_Ki_vel,
                Kd    = y_Kd_vel,
                Kisat = y_Kisat_vel)  

y_pos_PID = PID(
                Kp    = y_Kp_pos,
                Ki    = y_Ki_pos,
                Kd    = y_Kd_pos,
                Kisat = y_Kisat_pos) 

# Z Direction PID Controllers

z_jer_PID = PID(
                Kp    = z_Kp_jer,
                Ki    = z_Ki_jer,
                Kd    = z_Kd_jer,
                Kisat = z_Kisat_jer,)  

z_acc_PID = PID(
                Kp    = z_Kp_acc,
                Ki    = z_Ki_acc,
                Kd    = z_Kd_acc,
                Kisat = z_Kisat_acc,)   

z_vel_PID = PID(
                Kp    = z_Kp_vel,
                Ki    = z_Ki_vel,
                Kd    = z_Kd_vel,
                Kisat = z_Kisat_vel)  

z_pos_PID = PID(
                Kp    = z_Kp_pos,
                Ki    = z_Ki_pos,
                Kd    = z_Kd_pos,
                Kisat = z_Kisat_pos)  

# Yaw PID Controller
# The yaw controller is not a cascade controller, it is just a normal PID controller

yaw_PID = PID(
                Kp    = Kp_yaw,
                Ki    = Ki_yaw,
                Kd    = Kd_yaw,
                Kisat = Kisat_yaw)

# Initialise the previous target position to None so that we can check if the target has changed
prev_target_pos = None

# Initialise the previous position and velocity to 0 so that we can check if the target has changed
# If this was not done then the code would not run as there would be no previous position or velocity to compare to

x_vel_global = 0
y_vel_global = 0
z_vel_global = 0

x_prev_vel = 0
y_prev_vel = 0
z_prev_vel = 0

x_prev_pos = 0
y_prev_pos = 0
z_prev_pos = 0

# The controller function is the main function that will be called by the simulation
# It takes inputs of the state, target and dt (time step) and returns the velocity in (x, y, z) and yaw rate
def controller(state, target, dt):
    # Define the global variables so that we can use them in the function
    # This is necessary because we are using the global variables in the function and we need to tell python that we are using them
    global prev_target_pos, max_vel, x_vel_global, y_vel_global, z_vel_global, x_prev_vel, y_prev_vel, z_prev_vel, x_prev_pos, y_prev_pos, z_prev_pos, layers_of_cascade
    ## \/ See below for definitions of inputs \/ ##

    # State is in the format
    # state = [x position, y position, z position
    #          roll angle, pitch angle, yaw angle]
    # FYI - z is up and down and x and y are left and right and forward and back
    # x and y's influence change depending how you look at the drone so be careful
    # aaand
    # Target is in the format
    # target = [x position, y position, z position, y angle]
    # aaaaaaand
    # dt is time step :))


    # Extract the values from the state list - some of these are not used but are left in for debugging purposes
    pos = np.array(state[0:3]) # Position in the global reference frame
    yaw = state[5] # Yaw angle

    x_pos = state[0] # X position in the global reference frame
    y_pos = state[1] # Y position in the global reference frame
    z_pos = state[2] # Z position in the global reference frame
    cur_rol = state[3] # Roll angle
    cur_pit = state[4] # Pitch angle
    cur_yaw = state[5] # Yaw angle

    tar_x_pos = target[0] # X target position in the global reference frame
    tar_y_pos = target[1] # Y target position in the global reference frame
    tar_z_pos = target[2] # Z target position in the global reference frame
    
    # Extract the values from the target list
    target_pos = np.array(target[0:3]) # Target position in the global reference frame
    target_yaw = target[3] # Target yaw angle
    
    # Debugging Print statements
    """
    # Before returning the PID outputs
    print(f"[DEBUG] last_target: {prev_target_pos}")
    print(f"[DEBUG] target: {target}")
    print(f"[DEBUG] position (global): {pos}")
    """
    
    # Check if the target has changed - if it has then reset the PID controllers
    # Is None checks if the target has never been set before
    # Is not np.allclose checks if the target is close to the previous target
    if prev_target_pos is None or not np.allclose(target_pos, prev_target_pos):
        # Info print statements along with some smiley faces to cheer you up when you are knee-deep in debugging the code
        print("[INFO] The target has changed, resetting PID controllers and last target :D")
        print(f"[INFO] Target changed to: {target} :D")

        # Set the new target
        prev_target_pos = np.copy(target_pos)
        
        # Reset the PID controllers
        x_pos_PID.reset()
        y_pos_PID.reset()
        z_pos_PID.reset()

        x_vel_PID.reset()
        y_vel_PID.reset()
        z_vel_PID.reset()

        x_acc_PID.reset()
        y_acc_PID.reset()
        z_acc_PID.reset()

        x_jer_PID.reset()
        y_jer_PID.reset()
        z_jer_PID.reset()

        yaw_PID.reset()

        # Return 0s for the velocity and yaw rate to reset the drone's movement
        return (0.0, 0.0, 0.0, 0.0)

    
    # This next section of code performs the PID calculations for axis
    # The if statements mean that is is easy to change the number of layers of cascade while tuning
    # For continuity in the code the outputs that are not used are set to 0

    # X axis Cascade PID Calculations
    if x_layers_of_cascade == 1:
        (x_pos, x_vel_global, x_pos_error, x_vel_error) = PID.do_1layer_cascade(x_prev_pos, tar_x_pos, x_pos, x_vel_PID, dt)
        x_jer_correction = 0
        x_acc_error = 0
        x_pos_error_from_PID = 0

    elif x_layers_of_cascade == 2:
        (x_pos, x_vel_global, x_pos_error, x_vel_error, x_acc_error) = PID.do_2layer_cascade(x_prev_pos, x_prev_vel, tar_x_pos, x_pos, x_vel_PID, x_acc_PID, dt)
        x_jer_correction = 0
        x_pos_error_from_PID = 0

    elif x_layers_of_cascade == 3:
        (x_pos, x_vel_global, x_pos_error, x_vel_error, x_acc_error, x_jer_correction) = PID.do_3layer_cascade(x_prev_pos, x_prev_vel, tar_x_pos, x_pos, x_vel_PID, x_acc_PID, x_jer_PID, dt)
        x_pos_error_from_PID = 0

    elif x_layers_of_cascade == 4:
        (x_pos, x_vel_global, x_pos_error, x_pos_error_from_PID, x_vel_error, x_acc_error, x_jer_correction) = PID.do_4layer_cascade(x_prev_pos, x_prev_vel, tar_x_pos, x_pos, x_pos_PID, x_vel_PID, x_acc_PID, x_jer_PID, dt)
    
    # Update the previous position and velocity for the next iteration
    x_prev_pos = x_pos
    x_prev_vel = x_vel_global


    # Y axis Cascade PID Calculations
    if y_layers_of_cascade == 1:
        (y_pos, y_vel_global, y_pos_error, y_vel_error) = PID.do_1layer_cascade(y_prev_pos, tar_y_pos, y_pos, y_vel_PID, dt)
        y_jer_correction = 0
        y_acc_error = 0
        y_pos_error_from_PID = 0

    elif y_layers_of_cascade == 2:
        (y_pos, y_vel_global, y_pos_error, y_vel_error, y_acc_error) = PID.do_2layer_cascade(y_prev_pos, y_prev_vel, tar_y_pos, y_pos, y_vel_PID, y_acc_PID, dt)
        y_jer_correction = 0
        y_pos_error_from_PID = 0

    elif y_layers_of_cascade == 3:
        (y_pos, y_vel_global, y_pos_error, y_vel_error, y_acc_error, y_jer_correction) = PID.do_3layer_cascade(y_prev_pos, y_prev_vel, tar_y_pos, y_pos, y_vel_PID, y_acc_PID, y_jer_PID, dt)
        y_pos_error_from_PID = 0

    elif y_layers_of_cascade == 4:
        (y_pos, y_vel_global, y_pos_error, y_pos_error_from_PID, y_vel_error, y_acc_error, y_jer_correction) = PID.do_4layer_cascade(y_prev_pos, y_prev_vel, tar_y_pos, y_pos, y_pos_PID, y_vel_PID, y_acc_PID, y_jer_PID, dt)

    # Update the previous position and velocity for the next iteration
    y_prev_pos = y_pos
    y_prev_vel = y_vel_global




    # Z axis Cascade PID Calculations
    if z_layers_of_cascade == 1:
        (z_pos, z_vel_global, z_pos_error, z_vel_error) = PID.do_1layer_cascade(z_prev_pos, tar_z_pos, z_pos, z_vel_PID, dt)
        z_jer_correction = 0
        z_acc_error = 0
        z_pos_error_from_PID = 0

    elif z_layers_of_cascade == 2:
        (z_pos, z_vel_global, z_pos_error, z_vel_error, z_acc_error) = PID.do_2layer_cascade(z_prev_pos, z_prev_vel, tar_z_pos, z_pos, z_vel_PID, z_acc_PID, dt)
        z_jer_correction = 0
        z_pos_error_from_PID = 0

    elif z_layers_of_cascade == 3:
        (z_pos, z_vel_global, z_pos_error, z_vel_error, z_acc_error, z_jer_correction) = PID.do_3layer_cascade(z_prev_pos, z_prev_vel, tar_z_pos, z_pos, z_vel_PID, z_acc_PID, z_jer_PID, dt)
        z_pos_error_from_PID = 0

    elif z_layers_of_cascade == 4:
        (z_pos, z_vel_global, z_pos_error, z_pos_error_from_PID, z_vel_error, z_acc_error, z_jer_correction) = PID.do_4layer_cascade(z_prev_pos, z_prev_vel, tar_z_pos, z_pos, z_pos_PID, z_vel_PID, z_acc_PID, z_jer_PID, dt)

    # Update the previous position and velocity for the next iteration
    z_prev_pos = z_pos
    z_prev_vel = z_vel_global

    # Calculate the yaw error
    yaw_error = target_yaw - yaw

    # The following line makes sure the drone turns the shortest way
    # For example if your error was 460 then this would return 100 as the remaining
    # (But it's in radians so it would be like your input is 8.03 and you would get 1.75)
    # So basically the smallest measurable error from the yaw target (rather that looping around)
    yaw_error = (yaw_error + np.pi) % (2*np.pi) - np.pi

    # Get yaw rate from PID
    yaw_rate = yaw_PID.getPID_output(yaw_error, dt)
    # Constrain the drone to a reasonable maximum yaw rate
    max_yaw_rate = np.pi
    
    yaw_rate = np.clip(yaw_rate, -max_yaw_rate, max_yaw_rate)

    # Debugging Print statements
    """
    # After returning the PID outputs
    print(f"[DEBUG] pos_error: {pos_error}")
    print(f"[DEBUG] yaw_error: {yaw_error}")
    print(f"[DEBUG] PID_output (global): {vel_global}")
    """

    # Need to convert from local to global reference frame
    # We know what the drone is doing globally from its change in position using the vicon
    # But we need to send commands in terms of the local/body from so the drone knows
    # How much rpm etc to use on each rotor
    
    # Negative because when it was positive it was yeeting in the wrong direction
    cos_yaw = np.cos(-yaw)
    sin_yaw = np.sin(-yaw)


    # Below is a diagramatic example for converting from global to body frame
    #                     vx
    # ------------------------------------------->
    # \             \                           /
    #    \    yaw    )                         /
    #       \       )                         /
    #          \ ___/                        /
    #             \                         / 
    #                \                     /
    #       v1          \           ______/
    #   (component)        \      /      /    
    #                         \  /   90 /
    #                            \     /
    #                               \ /
    
    # Local velocities in the body frame
    v1 = cos_yaw*x_vel_global - sin_yaw*y_vel_global
    v2 = sin_yaw*x_vel_global + cos_yaw*y_vel_global
    v3 = z_vel_global


    # Debugging Print statements
    """
    print(f"[STATE] pos: {pos}, yaw: {yaw:.2f} | [TARGET] pos: {target_pos}, yaw_target: {target_yaw:.2f}")
    print(f"[GLOBAL VEL] x_vel_global: {vx:.2f}, y_vel_global: {vy:.2f}, z_vel_global: {vz:.2f}, yaw_rate: {yaw_rate:.2f}")
    print(f"[COMMAND VEL] v1: {v1:.2f}, v2: {v2:.2f}, v3: {v3:.2f}, yaw_rate: {yaw_rate:.2f}")
    """
    
    # Create a new file called 'output' if it does not exist and append to it if it does
    with open('output.csv','a') as file:

        # Save the data for tuning and analysis
        new_data = np.hstack((x_pos, y_pos, z_pos, # position
                              cur_rol, cur_pit, cur_yaw, # angles
                              tar_x_pos, tar_y_pos, tar_z_pos, # target position
                              target_yaw, # target yaw angle
                              x_vel_global, y_vel_global, z_vel_global, # global velocity
                              yaw_rate, # yaw rate
                              v1, v2, v3, # local velocity
                              x_pos_error, y_pos_error, z_pos_error, # position error
                              x_pos_error_from_PID, y_pos_error_from_PID, z_pos_error_from_PID, # position error from PID
                              x_vel_error, y_vel_error, z_vel_error, # velocity error
                              x_acc_error, y_acc_error, z_acc_error, # acceleration error
                              x_jer_correction, y_jer_correction, z_jer_correction, # jerk correction
                              yaw_error, # yaw error
                              dt, # time step
                              ))
        np.savetxt(file,[new_data],delimiter=',',fmt='%.6f')


    # The output of this controller should be:
    # [x velocity, y velocity, z velocity, yaw rate] (Velocities in the local reference frame)
    return(v1, v2, v3, yaw_rate)


# I hope you enjoyed reading the code, thank you and have a nice day :)
