// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"

void autopilot (void)
  // Autopilot to adjust the engine throttle. This does not control the parachute and leaves the attitude stabilizer engaged
{
  // These are constants defined to run the autopilot. These are manually tuned
    double Kh = 0.02; // positive constant that controlls the descent
    double Kp = 2; // positive constant known as the controller gain
    double lander_mass = UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY;
    double delta = (GRAVITY * MARS_MASS * lander_mass / position.abs2()) / MAX_THRUST;

    if (parachute_status == NOT_DEPLOYED && safe_to_deploy_parachute() && (position.abs() - MARS_RADIUS) < 40000) { // activation of the parachute requirements
        parachute_status = DEPLOYED;
    }
    /*
    The idea here is that the descent rate should decrease linearly as the lander approaches the surface.
    The error should be positive if the lander is approaching too quickly, and negative when the lander is decending too slowly.
    */
    double error = -(0.5 + Kh * (position.abs() - MARS_RADIUS) + velocity * position.norm());
    double Pout = Kp * error;

    if (Pout <= -delta) throttle = 0;
    else if (Pout < 1 - delta) throttle = delta + Pout;
    else throttle = 1;
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
  // 
{
    static vector3d previous_position;
    vector3d force_on_lander;
    vector3d new_position;
    double lander_mass = UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY;


    //First we calculate the forces currently on the lander. This includes thrust, drag and gravity
    vector3d drag = -0.5 * atmospheric_density(position) * DRAG_COEF_LANDER * 3.1315926536 * LANDER_SIZE * LANDER_SIZE * velocity.abs2() * velocity.norm();
    if (parachute_status == DEPLOYED) {
        drag += -0.5 * atmospheric_density(position) * DRAG_COEF_CHUTE * 5 * 4 * LANDER_SIZE * LANDER_SIZE * velocity.abs2() * velocity.norm();
    }
    vector3d gravity = -GRAVITY * MARS_MASS * lander_mass * position.norm() / position.abs2();
    force_on_lander = thrust_wrt_world() + drag + gravity;

    // now a verlet algorithm is performed to find the next steps in position and velocity
    if (simulation_time == 0.0) {
        new_position = position + velocity * delta_t + 0.5 * (delta_t * delta_t * force_on_lander) / lander_mass;
        velocity = 0.5 * (new_position - position) / delta_t;
    }
    else {
        new_position = 2 * position - previous_position + (delta_t * delta_t * force_on_lander) / lander_mass;
        velocity = (new_position - position) / delta_t;
    }

    previous_position = position;
    position = new_position;

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "literally land the lander";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = true;
    break;

  case 6:
    // a descent from 1 m up
    position = vector3d(0.0, (MARS_RADIUS + 0.5), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}

