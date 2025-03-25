#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <RotaryEncoder.h>
#include <PinChangeInterrupt.h>

// ========== Pin Definitions ==========
// Motor Control Pins
#define LMOTOR_DIRECTION_PIN 4
#define LMOTOR_SPEED_PIN 5
#define RMOTOR_SPEED_PIN 6
#define RMOTOR_DIRECTION_PIN 7

// Motor(position) encoder
#define MOTOR_ENCODER_PIN_A 8
#define MOTOR_ENCODER_PIN_B 9

// External(angle) encoders - Pendulum
#define PENDULUM_ENCODER_PIN_A 2
#define PENDULUM_ENCODER_PIN_B 3

// ========== Constants ==========
#define MOTOR_ENCODER_CPR 116.4
#define PENDULUM_ENCODER_CPR 2000
#define WHEEL_DIAMETER 0.08 // meters

// Calculate how many radians per tick for the pendulum
const float rad_per_pendulum_tick = (2.0f * PI) / PENDULUM_ENCODER_CPR;
const float target_distance = 1.85;  // 6ft in meters
// ========== Forward Declarations ==========
// A simple placeholder controller function
float controller(const float *state, const float *setpoint, const float target_distance);

float filtered_pendulum_angle = 0.0f;
float filtered_pendulum_velocity = 0.0f;
float angle_filter_alpha = 0.2f;
float velocity_filter_alpha = 0.1f;

// ========== Cart Struct ==========
struct Cart
{
  // Motor encoder to track cart position
  RotaryEncoder motor_encoder;

  // Constructor
  Cart(int encA, int encB) : motor_encoder(encA, encB) {}

  // Set speed for both motors (±255)
  void set_speed(int speed)
  {
    /*
      speed: int from -255 to +255
      Positive => forward movement
      Negative => reverse movement
    */

    if (speed > 0)
    {
      // Forward direction
      digitalWrite(LMOTOR_DIRECTION_PIN, LOW);
      digitalWrite(RMOTOR_DIRECTION_PIN, HIGH);
      speed = min(speed, 255);
    }
    else
    {
      // Reverse direction
      digitalWrite(LMOTOR_DIRECTION_PIN, HIGH);
      digitalWrite(RMOTOR_DIRECTION_PIN, LOW);
      speed = max(speed, -255);
    }

    // The absolute value is used for PWM
    analogWrite(LMOTOR_SPEED_PIN, abs(speed));
    analogWrite(RMOTOR_SPEED_PIN, abs(speed));
  }

  // Return cart position in meters based on the motor encoder
  float get_position()
  {
    float enc_val = motor_encoder.getPosition();
    // Convert encoder ticks to distance:
    return (WHEEL_DIAMETER * PI) * (enc_val / MOTOR_ENCODER_CPR);
  }

  // Return cart velocity in m/s based on the motor encoder
  float get_velocity()
  {
    static int sign = 1;
    int direction = (int)motor_encoder.getDirection();
    if (direction != 0 && direction != sign)
    {
      sign = -sign;
    }

    float angular_velocity = 0.0f;
    unsigned long lastTickTime = motor_encoder.getLastTickTime();

    // If we had a tick in the last 100 ms, compute speed from microsBetweenRotations
    if (millis() - lastTickTime < 100)
    {
      unsigned long mbr = max((unsigned long)1, motor_encoder.getMicrosBetweenRotations());
      float raw_av = (2.0f * PI * 1000000.0f) / (MOTOR_ENCODER_CPR * (float)mbr);
      // clamp to some max to avoid noise
      if (raw_av > 20.0f * PI)
      {
        raw_av = 20.0f * PI;
      }
      angular_velocity = sign * raw_av;
    }
    // Convert rad/s to linear speed: v = ω * r
    return angular_velocity * (WHEEL_DIAMETER * 0.5f);
  }

  // Convert a desired torque (N·m) to ±255 PWM
  int torque_to_PWM(float desiredTorque)
  {
    // Example constants - adjust based on your motor specs
    const float tau_stall = 13.0f; // in kg·mm
    const float I_stall = 2.0f;    // in A
    const float V_nominal = 6.0f;  // in V

    // Torque constant Kt = stall_torque / stall_current
    float K_t = tau_stall / I_stall;

    // Required current
    float I_required = desiredTorque / K_t;

    // Required voltage
    float V_required = (I_required / I_stall) * V_nominal;

    // Convert voltage to PWM (range ±255)
    int pwm_value = (int)(255.0f * (V_required / V_nominal));

    // Clamp PWM value to valid range
    if (pwm_value > 255)
      pwm_value = 255;
    if (pwm_value < -255)
      pwm_value = -255;
    return pwm_value;
  }
};

float get_pendulum_angle(RotaryEncoder &pendulum_encoder, float init_angle)
{
  // Calculate raw angle
  float raw_angle = (pendulum_encoder.getPosition() * rad_per_pendulum_tick) - init_angle;

  // Apply low pass filter
  filtered_pendulum_angle = angle_filter_alpha * raw_angle + (1.0f - angle_filter_alpha) * filtered_pendulum_angle;

  return filtered_pendulum_angle;
}

float get_pendulum_angular_velocity(RotaryEncoder &pendulum_encoder)
{
  static int sign = 1;
  // getDirection() returns CW=1 or CCW=-1 (depending on library)
  int direction = (int)pendulum_encoder.getDirection();
  if (direction != 0 && direction != sign)
  {
    sign = -sign;
  }

  float raw_velocity = 0.0f;

  // If no tick for too long, velocity = 0
  if (millis() - pendulum_encoder.getLastTickTime() < 50)
  {
    // max(1, microsBetweenRotations()) to avoid divide-by-zero
    unsigned long mbr = max((unsigned long)1, pendulum_encoder.getMicrosBetweenRotations());
    raw_velocity = (2.0f * PI * 1000000.0f) / (PENDULUM_ENCODER_CPR * mbr);
    // clamp to some max velocity to avoid weird spikes
    if (raw_velocity > 4.0f * PI)
    {
      raw_velocity = 4.0f * PI;
    }
    raw_velocity *= sign;
  }

  // Apply low pass filter
  filtered_pendulum_velocity = velocity_filter_alpha * raw_velocity +
                               (1.0f - velocity_filter_alpha) * filtered_pendulum_velocity;

  return filtered_pendulum_velocity;
}

// ========== Global Objects ==========
// Cart with motor encoder on pins 8 and 9
Cart cart(MOTOR_ENCODER_PIN_A, MOTOR_ENCODER_PIN_B);

// Pendulum encoder on pins 2 and 3
RotaryEncoder pendulum_encoder(PENDULUM_ENCODER_PIN_A, PENDULUM_ENCODER_PIN_B);

// initial pendulum angle offset
float init_pendulum_angle = 0.0f;

// We'll store [x, x_dot, theta, theta_dot]
float state[4] = {0, 0, 0, 0};
float setpoint[4] = {0, 0, 0, 0};

// ========== Setup ==========
void setup()
{
  PCICR |= B00000100;
  PCMSK2 |= B00111111;

  // Motor direction pins
  pinMode(LMOTOR_DIRECTION_PIN, OUTPUT);
  pinMode(RMOTOR_DIRECTION_PIN, OUTPUT);
  // Motor speed (PWM) pins
  pinMode(LMOTOR_SPEED_PIN, OUTPUT);
  pinMode(RMOTOR_SPEED_PIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }

  float angle_total = 0;
  Serial.print("Initial pendulum angle: ");
  Serial.println(init_pendulum_angle, 6);
}

// ========== Main Loop ==========
void loop()
{
  // ---------- 1) Update state ----------
  // 1) cart position
  state[0] = cart.get_position();
  // 2) cart velocity
  state[1] = cart.get_velocity();
  // 3) pendulum angle
  state[2] = get_pendulum_angle(pendulum_encoder, init_pendulum_angle);
  // 4) pendulum angular velocity
  state[3] = get_pendulum_angular_velocity(pendulum_encoder);

  // ---------- 2) Calculate control signal ----------
  // Example: call your controller to get the commanded torque
  float desiredTorque = controller(state, setpoint, target_distance);

  // ---------- 3) Apply control to motors ----------
  // Convert torque -> PWM -> set motor speeds
  int pwm_val = cart.torque_to_PWM(desiredTorque);
  if (pwm_val < 50 && pwm_val > -50)
  {
    pwm_val = 0;
  }
  cart.set_speed(pwm_val);

  // ---------- 4) Debug Output ----------
  static int printCounter = 0;
  if (printCounter++ % 20 == 0)
  {
    // Print every 20 loops to make the Serial more readable
    Serial.print(state[2]); // angle theta
    Serial.print(",");
    Serial.print(state[0]); // linear displacement x
    Serial.print(",");
    Serial.println(pwm_val); // PWM value
  }
  // Short delay for stable execution
  delay(10);
}

// ========== Interrupt Service Routines ==========
ISR(PCINT2_vect)
{
  // Motor encoder tick
  cart.motor_encoder.tick();

  // Pendulum encoder tick
  pendulum_encoder.tick();
}
