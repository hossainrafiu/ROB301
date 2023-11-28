#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt32MultiArray, Float64MultiArray, UInt32
import numpy as np
import colorsys

actual_index = 320
desired_index = 0
follow_line = True

class Controller(object):
    def __init__(self):
        # publish motor commands
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # subscribe to detected line index
        self.color_sub = rospy.Subscriber(
            "line_idx", UInt32, self.camera_callback, queue_size=1
        )

    def camera_callback(self, msg):
        """Callback for line index."""
        global desired_index
        desired_index = msg.data
        #print(msg.data)

    def follow_the_line(self, max_speed):
        global follow_line
        integral = 0
        derivative = 0
        lasterror = 0
        kp = 0.002
        ki = 0.000005
        kd = 0.001
        twist = Twist()
        
        error = desired_index - actual_index
        if abs(integral) > 10000:
            integral = 0
        integral = integral + error
        derivative = error - lasterror
        
        twist.angular.z = max_speed*1.5*((- kp * error) + (- ki * integral) + (-kd * derivative)) if follow_line else 0
        twist.linear.x = max_speed/(1+math.exp(abs(error)/60-2)) if follow_line else max_speed
        
        self.cmd_pub.publish(twist)
        lasterror = error

        if abs(error) > 280:
            self.find_line(error)       # MIGHT HAVE TO DISABLE!!!!!!!!!!!!!!!!!!!1
        
        return None
    
    def delivery(self):
        twist = Twist()
        rate = rospy.Rate(10)
        time = rospy.get_time()
        while rospy.get_time() - 1 < time:
            twist.angular.z = math.pi/4
            self.cmd_pub.publish(twist)
            rate.sleep()
        rospy.sleep(2)
        time = rospy.get_time()
        while rospy.get_time() - 1 < time:
            twist.angular.z = -math.pi/4
            self.cmd_pub.publish(twist)
            rate.sleep()
        
        return None
    
    def find_line(self, error):
        turn_time = 0.25
        direction = 1
        twist = Twist()
        rate = rospy.Rate(10)
        time = rospy.get_time()
        while abs(error) > 150:
            while rospy.get_time - turn_time < time:
                twist.angular.z = 0.15 * direction
                rate.sleep()
            direction *= -1
            turn_time += 0.25
        return None


class BayesLoc:
    def __init__(self, p0, colour_codes, colour_map):
        self.colour_sub = rospy.Subscriber("mean_img_rgb", Float64MultiArray, self.colour_callback)
        #self.line_sub = rospy.Subscriber("line_idx", UInt32, self.line_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.num_states = len(p0)
        self.colour_codes = colour_codes
        self.colour_map = colour_map
        self.probability = p0
        self.state_prediction = np.zeros(self.num_states)
        
        self.cur_colour = None  # most recent measured colour
        self.cur_colour_rgb = [0,0,0]
        self.colour_prop = [0,0,0,0,0]
        self.prev_colours = [4 for i in range(25)]
        self.last_colour = rospy.get_time()
        self.prev_colour = None
        self.colour_list = [
                        "blue",
                        "green",
                        "yellow",
                        "orange",
                        "line"
                        ]
        self.meas_model = [
                    [0.6,0.2,0.05,0.05],
                    [0.2,0.6,0.05,0.05],
                    [0.05,0.05,0.65,0.2],
                    [0.05,0.05,0.15,0.6],
                    [0.1,0.1,0.1,0.1]
                    ]

    def colour_callback(self, msg):
        """
        callback function that receives the most recent colour measurement from the camera.
        """
        self.cur_colour_rgb = np.array(msg.data)  # [r, g, b]
        global follow_line
        follow_line = True
        diffs = []
        for i in range(5):
            colour = colour_codes[i]
            diff = 0
            diff += abs(self.cur_colour_rgb[0] - colour[0])**2
            diff += abs(self.cur_colour_rgb[1] - colour[1])**2
            diff += abs(self.cur_colour_rgb[2] - colour[2])**2
            diffs.append(diff**(1/2))
        
        min_diff_index = diffs.index(min(diffs))
        if min_diff_index < 4:
            #print(self.colour_list[min_diff_index], diffs)
            self.cur_colour = min_diff_index
            follow_line = False
            self.prev_colours.pop(0)
            self.prev_colours.append(min_diff_index)
            self.last_colour = rospy.get_time()
        else:
            self.cur_colour = None
            if rospy.get_time() - self.last_colour > 0.5 and self.prev_colour != 4.5:
                self.prev_colours[:] = [4.5] * len(self.prev_colours)
                self.prev_colour = 4.5
                print("PREV COLOR RESET")
        #print(self.cur_colour)

        self.colour_prop = np.array(diffs)


    def line_callback(self, msg):
        """
        TODO: Complete this with your line callback function from lab 3.
        """
        return

    def wait_for_colour(self):
        """Loop until a colour is received."""
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self.cur_colour is None:
            rate.sleep()

    def state_model(self, u):
        """
        State model: p(x_{k+1} | x_k, u)

        TODO: complete this function
        """
        
        if u == -1:
            ratio = [0.85, 0.1, 0.05]
        elif u == 0:
            ratio = [0.05, 0.9, 0.05]
        else:
            ratio = [0.05, 0.1, 0.85]

        return ratio
            

    def measurement_model(self, x=0):
        """
        Measurement model p(z_k | x_k = colour) - given the pixel intensity,
        what's the probability that of each possible colour z_k being observed?
        """
        cur_colour = np.average(self.prev_colours)
        #print(cur_colour, cur_colour//1, self.prev_colour)
        if not((cur_colour//1)-cur_colour == 0.0 and round(cur_colour) != self.prev_colour):
            return None
        
        cur_colour = round(cur_colour)
        self.prev_colour = cur_colour

        meas_arr = self.meas_model[int(cur_colour)]

        """
        TODO: You need to compute the probability of states. You should return a 1x5 np.array
        Hint: find the euclidean distance between the measured RGB values (self.cur_colour)
            and the reference RGB values of each colour (self.ColourCodes).
        """

        return meas_arr
        
    def state_predict(self):
        rospy.loginfo("predicting state")
        """
        TODO: Complete the state prediction function: update
        self.state_prediction with the predicted probability of being at each
        state (office)
        """
        ratio = self.state_model(u=1)

        new_prob = [0 for i in range(len(self.probability))]

        prob_plus = list(self.probability[:])
        prob_plus.insert(0, prob_plus.pop())
        prob_minus = list(self.probability[:])
        prob_minus.append(prob_minus.pop(0))
        
        for i in range(len(self.probability)):
            new_prob[i] = (ratio[2]*prob_plus[i] + ratio[1]*self.probability[i] + ratio[0]*prob_minus[i])
        
        self.state_prediction = new_prob

    def state_update(self):
        rospy.loginfo("updating state")
        """
        TODO: Complete the state update function: update self.probabilities
        with the probability of being at each state
        """
        # prob = self.measurement_model()
        # colour = prob.index(max(prob))
        # meas_arr = self.meas_model[colour]
        
        meas_arr = self.measurement_model()
        if meas_arr is None:
            return None
            #meas_arr = self.measurement_model()
        self.state_predict()
        new_prob = [0 for i in range(len(self.probability))]
        for i in range(len(self.state_prediction)):
            new_prob[i] = self.state_prediction[i]*meas_arr[self.colour_map[i]]
        new_prob = [i/sum(new_prob) for i in new_prob]
        
        self.probability = new_prob
        print("PROBABILITY", self.probability.index(max(self.probability)))
        
        return None




if __name__ == "__main__":

    # This is the known map of offices by colour
    # 0: blue, 1: green, 2: yellow, 3: orange, 4: line
    # current map starting at cell #2 and ending at cell #12
    colour_map = [2, 1, 0, 3, 3, 1, 0, 3, 2, 1, 0]

    # TODO calibrate these RGB values to recognize when you see a colour
    # NOTE: you may find it easier to compare colour readings using a different
    # colour system, such as HSV (hue, saturation, value). To convert RGB to
    # HSV, use:
    # h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
    colour_codes = [
        [194.1, 173.1, 188.6],  # blue
        [175.8, 196.1, 168.0],  # green
        [191.2, 185.5, 179.0],  # yellow
        [228.7, 119.5, 110.3],  # orange
        [148.7, 137.9, 141.1],  # line
    ]

    colour_codes = [
        [194.1, 181.1, 198.6],  # blue  #FIX!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        [144.8, 175.1, 160.0],  # green
        [191.2, 174.5, 163.0],  # yellow
        [244.7, 131.5, 100.3],  # orange
        [148.7, 137.9, 141.1],  # line
    ]

    # initial probability of being at a given office is uniform
    p0 = np.ones_like(colour_map) / len(colour_map)

    localizer = BayesLoc(p0, colour_codes, colour_map)
    controller = Controller()
    
    rospy.init_node("final_project")
    rospy.sleep(0.5)
    rate = rospy.Rate(10)

    delivery_locations = [8, -1, 3, -1] # Calibration
    delivery_locations.append([1, 2, 3]) # Delivery Locations

    while not rospy.is_shutdown():
        """
        TODO: complete this main loop by calling functions from BayesLoc, and
        adding your own high level and low level planning + control logic
        """
        if localizer.probability == delivery_locations[0]:
            if delivery_locations[1] == -1:     # Calibration
                delivery_locations.pop(0)
                delivery_locations.pop(0)
            else:    
                rospy.sleep(1)
                controller.delivery()
                delivery_locations.pop(0)
                delivery_locations.append(-2)   # never empty delivery list
        controller.follow_the_line(0.15 if follow_line else 0.05)
        localizer.state_update()
        rate.sleep()

    rospy.loginfo("finished!")
    rospy.loginfo(localizer.probability)
