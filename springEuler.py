import numpy as np
import matplotlib.pyplot as plt

def runMassSpring():
    # mass, spring constant, initial position and velocity
    m = 1
    k = 1
    x = 1
    v = 0

    # simulation time, timestep and time
    t_max = 10
    dt = 0.0001 #! inital 0.1 is too small, values increase 
    t_array = np.arange(0, t_max, dt)

    # initialise empty lists to record trajectories
    x_list = []
    v_list = []

    # Euler integration
    for t in t_array:

        # append current state to trajectories
        x_list.append(x)
        v_list.append(v)

        # calculate new position and velocity
        a = -k * x / m
        x = x + dt * v
        v = v + dt * a

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

def runThreeD():
    # Gravitational constant, mass of planet (mars), mass of moving body and position relative to planet
    G = 6.6743e-11
    M = 6.42e23
    m = 1 
    r = np.array([[0],[0],[1]])
    v = np.array([[0],[0],[0]])

    # simulation time, timestep and time
    t_max = 10
    dt = 0.0001 #! inital 0.1 is too small, values increase 
    t_array = np.arange(0, t_max, dt)

    # initialise empty lists to record trajectories
    position = np.array([],[],[])
    velocity = np.array([],[],[])

    # Euler integration
    for t in t_array:

        # append current state to trajectories
        position += r
        velocity += v

        # calculate new position and velocity
        Force_no_mass = - G * M * r / np.linalg.norm(r)**2  # does not include mass calculation as this is later divided 
        r = r + dt * v
        v = v + dt * Force_no_mass


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
    #runThreeD()
    position = np.array([[],[],[]])
    r = np.array([[3],[4],[1]])
    G = 6.6743e-11
    M = 6.42e23
    Force_no_mass = - G * M * r / np.linalg.norm(r)**2
    print(G)
    print(Force_no_mass)