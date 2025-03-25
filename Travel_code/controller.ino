#include <Arduino.h>
#include <math.h>

// Physical Parameters
#define L 0.5   // length of pendulum (meters)
#define M 0.7   // mass of cart (kg)
#define m 0.05  // mass at end of pendulum (kg)
#define g 9.81  // gravity (m/s^2)

// Defining control strategies
bool LQR = true;
bool PolePlacement = false;
bool SMC = false;
bool PID = false;

// LQR gain matrix
float K_LQR[4] = {0, 0, -185.29343, 7.98934};

// Pole Placement Gain matrix
float K_PolePlacement[4] = {0, 0, 150.05389, 5.10431};

// SMC parameters
float lambda_theta = 2;
float eta = 2;
float lambda_X = 1;
float alpha = 2;
float phi = 0.8;
float cart_damp = 0.1;
float pendulum_damp = 0.01;

// PID Parameters
const int PID_control_state_idx = 2;
float Kp = 490;
float Ki = 0;
float Kd = 110;
float previous_Error = 0;
float integral = 0;
float previous_Control = 0;
float previous_Time = micros();

// Position Control Gains
const float Kp_position = 50.0;  // Proportional gain for position
const float Kd_position = 10.0;  // Derivative gain for position stabilization

// Utility Functions
float sign(float x) {
  return (x > 0) - (x < 0);
}

float controller(float *state, float *setpoint, float target_distance) {
  // Calculate position error and control
  float position_error = target_distance - state[0];
  float position_velocity = -state[1];  // Negative to help dampen movement

  // Create a position control term
  float position_control = Kp_position * position_error + Kd_position * position_velocity;

  // Modify setpoint to incorporate position control
  float modified_setpoint[4] = {setpoint[0], setpoint[1], setpoint[2], setpoint[3]};
  modified_setpoint[0] += position_control;  // Adjust cart position setpoint

  float control_signal = 0;

  if (LQR) {
    for (int i = 0; i < 4; i++) {
      control_signal -= K_LQR[i] * (modified_setpoint[i] - state[i]);
    }
  } else if (PolePlacement) {
    for (int i = 0; i < 4; i++) {
      control_signal -= K_PolePlacement[i] * (modified_setpoint[i] - state[i]);
    }
  } else if (SMC) {
    // Extract state variables
    float x = state[0], x_dot = state[1], theta = state[2], theta_dot = state[3];

    // Compute trigonometric functions
    float sin_theta = sin(theta);
    float cos_theta = cos(theta);

    // Define reference values
    float x_ref = modified_setpoint[0], theta_ref = modified_setpoint[2];

    // Compute sliding surfaces
    float s_theta = theta_dot + lambda_theta * (theta - theta_ref);
    float s_x = x_dot + lambda_X * (x - x_ref);
    float s = s_theta + alpha * s_x;

    // Compute the equivalent control term u_eq
    float numerator = lambda_X * L * (((M + m) - m * (cos_theta * cos_theta)) * (theta - theta_ref))
                      + (M + m) * g * sin_theta
                      - m * L * theta_dot * theta_dot * sin_theta * cos_theta;
    float denominator = cos_theta;

    float u_eq = numerator / denominator;

    // Compute the switching control term using tanh for smoothness
    float u_sw = -eta * tanh(s / phi);

    // Compute total control input
    control_signal -= u_eq + u_sw;
  } else if (PID) {
    float dt_micros = micros() - previous_Time;
    float error = modified_setpoint[PID_control_state_idx] - state[PID_control_state_idx];
    
    integral *= sign(error) == sign(previous_Error);
    integral += error * dt_micros / 1000000;
    float derivative = 1000000 * (error - previous_Error) / dt_micros;
    
    control_signal = Kp * error + Ki * integral - Kd * derivative;
    previous_Error = error;
    previous_Time = micros();
  }

  previous_Control = control_signal;
  return control_signal;
}