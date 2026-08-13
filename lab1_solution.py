# Required libraries
import numpy as np
import matplotlib.image as mpimg
from scipy import linalg
import matplotlib.pyplot as plt
from spatialmath import SE2
from spatialmath.base import trplot2
from pathlib import Path
from ir_support import functions as ir
from pathlib import Path
# Useful variables
from math import pi

def lab1_solution_run(): #runs the shit
    plt.close("all")
    lab1_solution = Lab1Solution()
    lab1_solution.question1()
    lab1_solution.question2()
    lab1_solution.questions3_and_4()

    # Uncomment this line below to keep the figure after the last question finished
    # plt.show()

# ---------------------------------------------------------------------------------------#
class Lab1Solution:
    def __init__(self):
        image_path = Path(__file__).parent / "Lab1CircularRaceTrack.jpg" #sets path file to find image
        self.img = mpimg.imread(str(image_path)) #reads image into self.image as numpy array

        self.fig = None #placeholder
        self.next_question = False  # Flag can be used to move to the next question (e.g. if closing a window)

        # Constants
        # Track radii (in pixels) based on measurements from image:
        self.RADIUS_OUTER = (550 - 66)/2  # outer lane: track edge to center
        # => Approx circumference = 2πr ≈ 2π × 242 ≈ 1521 pixels
        self.RADIUS_INNER = (500 - 125)/2  # inner lane: track edge to center
        # => Approx circumference = 2πr ≈ 2π × 187.5 ≈ 1178 pixels

    # ---------------------------------------------------------------------------------------#
    def question1(self):
        print('Download and setup the Robotics Toolbox in Python. See videos and links in Canvas.')
        input("Press Enter to continue") #just writes shit in terminal

    # ---------------------------------------------------------------------------------------#
    def question2(self):
        self.fig = plt.figure()  # create a new figure
        self.fig.canvas.mpl_connect('close_event', self.on_close)  # attach a close event to this window
        self.fig.canvas.manager.set_window_title('Question 2')  # set the window title of the figure
        plt.imshow(self.img) #draws track image as background

        car1_tr = SE2(300, 550, 0)  # initial pose of car 1 x=300, y=550, theta=0 radians
        trplot2(car1_tr.A, frame='1', color='b', length=50, width=0.05) #draws car 1's frame axes ontop of the image

        total_steps = 360  # steps per revolution

        # So the transform each step is
        car1_move_tr = SE2((2 * pi * self.RADIUS_OUTER)/total_steps, 0, 0) #transformation matrix for moving forward along the track
        car1_turn_tr = SE2(0, 0, -2*pi/total_steps) #transformation matrix for turning counter-clockwise along the track

        for _ in range(total_steps):
            # Update figure
            plt.cla()  # Clear the current axes
            plt.imshow(self.img) #redraws track image (otherwise old frames/text would pile up)

            # Update car1 pose: move forward, then turn slightly (counter-clockwise)
            car1_tr = ir.clean_SE2(car1_tr * car1_move_tr * car1_turn_tr) #does the maths for each point
            trplot2(car1_tr.A, frame='1', color='b', length=50, width=0.05) #draws point with axes displayed

            message = '\n'.join(['  '.join([f"{val:.2g} " for val in row]) for row in car1_tr.A]) #creates matrix
            plt.text(10, 50, message, fontsize=10, color=[.6, .2, .6]) #writes matrix in terminal

            plt.draw() #redraws the figure on terminal
            plt.pause(0.01) #pause for 0.01 seconds to allow the figure to update

            # Move to the next question if prompted
            if self.next_question:
                self.next_question = False  # Reset the flag
                break  # Exit the loop

    # ---------------------------------------------------------------------------------------#
    def questions3_and_4(self):
        for question in [3, 4]:  # question will take value 3 and 4, done in same loop becuase 4 takes alot of values from 3
            self.fig = plt.figure()  # create a new figure
            self.fig.canvas.manager.set_window_title('Question ' + str(question))  # Set the window title of the figure
            self.fig.canvas.mpl_connect('close_event', self.on_close)  # attach a close event to this window
            if question == 4: #if question 4, make a subplot for the distance plot
                plt.subplot(1, 2, 1) #gang idfk

            plt.imshow(self.img) #draws track image as background

            car1_tr = SE2(300, 550, 0)  # initial pose of car 1 (x, y, theta (radians))
            car2_tr = SE2(300, 125, 0)  # initial pose of car 2

            # For distance plot (i.e. question 4)
            if question == 4:
                plt.subplot(1,2,2) #now if question 4 select the second subplot just ti label its axes once before the loop starts
                plt.xlabel('Timestep') 
                plt.ylabel("Sensor reading - distance between cars")

            total_steps = 360 #360 steps per lap same as before
            # So the transform each step is
            car1_move_tr = SE2((2 * pi * self.RADIUS_OUTER)/total_steps, 0, 0) #math for car 1, same as question 2
            car1_turn_tr = SE2(0, 0, -2*pi/total_steps) #math for car 1, same as question 2
            car2_move_tr = SE2((2 * pi * self.RADIUS_INNER)/total_steps, 0, 0) #math for car 2
            car2_turn_tr = SE2(0, 0, 2*pi/total_steps) #math for car 2
            dist = np.zeros(total_steps) #HELP OH GOD HELP, i mean preallocated array to store the intercar distance at each timestep for question 4

            for i in range(total_steps): #per step loop logic
                car1_tr = ir.clean_SE2(car1_tr * car1_move_tr * car1_turn_tr) #update car 1's pose by moving and turning
                car2_tr = ir.clean_SE2(car2_tr * car2_move_tr * car2_turn_tr) #update car 2's pose by moving and turning

                print("car1_to_2_tr = \n", car1_tr.inv() * car2_tr) #prints car1 relative to car2
                print("car2_to_1_tr = \n", car2_tr.inv() * car1_tr) #prints car2 relative to car1

                if question == 4:  # Switch to update subplot 1
                    plt.subplot(1,2,1) #select the track subplot to draw the cars on

                plt.cla() #clear the current axes
                plt.imshow(self.img) #redraws track image so old frames don't pile up

                trplot2(car1_tr.A, frame='1', color='b', length=50, width=0.05) #draws car 1's frame axes
                trplot2(car2_tr.A, frame='2', color='r', length=50, width=0.05) #draws car 2's frame axes

                if question == 4:
                    plt.subplot(1,2,2) #switch to the distance subplot
                    plt.xlabel('Timestep')
                    plt.ylabel("Sensor reading - distance between cars")

                    dist[i] = linalg.norm(car1_tr.t - car2_tr.t) #distance between the two cars this timestep
                    dist_point_h = plt.plot(range(1, i+1), dist[:i], 'b-') #plots distance history so far

                plt.draw() #redraws the figure
                plt.pause(0.01) #pause for 0.01 seconds to allow the figure to update

                # Move to the next question if prompted
                if self.next_question:
                    self.next_question = False  # Reset the flag
                    break  # Exit the loop

    # ---------------------------------------------------------------------------------------#
    # This function will be executed when a figure is closed
    def on_close(self, event):
        self.next_question = True #set flag so the running loop knows to move on

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    lab1_solution_run()