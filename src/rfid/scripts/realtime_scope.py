import matplotlib.pyplot as plt
import numpy as np
import time
import random
import rospy
from rfid.msg import rfid_msg


def scope1(): # accumulated view

    plt.ion()  # Enable interactive mode
    fig, ax = plt.subplots()

    # x = np.linspace(1, 100, 100)  # Initial x-axis values (1 to 100)
    x = np.array([0]) # total version

    line, = ax.plot(x, np.zeros_like(x)) # hundred 0s
    # print(len(np.zeros_like(x)))

    y = np.zeros_like(x)

    while True:
        # Update your real-time variables here
        # y = np.delete(y, [0]) # limited scope version
        y = np.append(y, random.random())

        line.set_ydata(y)

        # Update the x-axis values dynamically
        # x += 1  # Example: Increase x-axis values by 1, [1,2,3] to [2,3,4]
        x = np.append(x, x[-1]+1)

        # Update the plot with the new x-axis values
        line.set_xdata(x)

        # Adjust the x-axis limits
        # ax.set_xlim(x[0], x[-1])

        ax.relim()
        ax.autoscale_view()

        fig.canvas.draw()
        fig.canvas.flush_events()

        time.sleep(0.1)



def scope2():
    plt.ion()  # Enable interactive mode
    fig, ax = plt.subplots()
    x = np.linspace(0, 10, 100)  # from 0 to 10, total length = 100


    line, = ax.plot(x, np.zeros_like(x))

    while True:
        # Update your real-time variables here
        y = np.random.randn(len(x))  # randomly generated, total 100

        # Update the plot with the new data
        line.set_ydata(y)
        fig.canvas.draw()
        fig.canvas.flush_events()

        # Optional: Add a delay to control the update rate
        time.sleep(0.1)



if __name__ == '__main__':

    scope2()