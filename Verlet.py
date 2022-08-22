# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

def run():
    # mass, spring constant, initial position and velocity
    m = 1
    k = 1
    x = 0
    v = 1

    # simulation time, timestep and time
    t_max = 1000
    dt = 1 #! inital 0.1 is too small, values increase 
    t_array = np.arange(0, t_max, dt)

    # initialise empty lists to record trajectories
    x_list = []
    v_list = []

    # Verlet integration
    for t in t_array:
        
        # append current state to trajectories
        x_list.append(x)
        v_list.append(v)
        
        # This is for the first operation of the verlet to utilise initial velocity
        if t == 0:
            a = x
            x = x + v*dt + 0.5 * dt**2 * -k * x
            v = (x-a)/dt
        # else calculate new position and velocity
        else:
            a=x
            x = 2 * x - x_list[-2] + dt**2 * -k * x / m
            v = (x-a)/dt
        

    # convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
    x_array = np.array(x_list)
    v_array = np.array(v_list)

    # plot the position-time graph
    plt.figure(1)
    plt.clf()
    plt.xlabel('time (s)')
    plt.grid()
    plt.plot(t_array, x_array, label='x (m)')
    plt.plot(t_array, v_array, label='v (m/s)')
    plt.legend()
    plt.show()


if __name__ == "__main__":
    run()
