#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <RotaryEncoder.h>
#include <PinChangeInterrupt.h>

// ======= Pin Definitions =======
#define LMOTOR_DIR 4
#define LMOTOR_PWM 5
#define RMOTOR_PWM 6
#define RMOTOR_DIR 7

#define MOTOR_ENC_A 8
#define MOTOR_ENC_B 9
#define PEND_ENC_A 2
#define PEND_ENC_B 3

// ======= Constants =======
#define MOTOR_CPR 116.4f
#define PEND_CPR 2000.0f
#define WHEEL_DIAMETER 0.08f

const float rad_per_pend_tick = (2.0f * PI) / PEND_CPR;

// ======= Filter Parameters =======
float filtered_angle = 0.0f;
float filtered_ang_vel = 0.0f;
const float alpha_angle = 0.2f;
const float alpha_velocity = 0.1f;

// ======= Cart Class =======
struct Cart {
  RotaryEncoder encoder;

  Cart(int a, int b) : encoder(a, b) {}

  void set_speed(int speed) {
    if (speed > 0) {
      digitalWrite(LMOTOR_DIR, LOW);
      digitalWrite(RMOTOR_DIR, HIGH);
    } else {
      digitalWrite(LMOTOR_DIR, HIGH);
      digitalWrite(RMOTOR_DIR, LOW);
    }
    analogWrite(LMOTOR_PWM, abs(speed));
    analogWrite(RMOTOR_PWM, abs(speed));
  }

  float get_position() {
    float ticks = encoder.getPosition();
    return (PI * WHEEL_DIAMETER) * (ticks / MOTOR_CPR);
  }

  float get_velocity() {
    static int sign = 1;
    int dir = encoder.getDirection();
    if (dir != 0 && dir != sign) sign = -sign;

    unsigned long lastTick = encoder.getLastTickTime();
    if (millis() - lastTick < 100) {
      unsigned long mbr = max((unsigned long)1, encoder.getMicrosBetweenRotations());
      float omega = (2.0f * PI * 1000000.0f) / (MOTOR_CPR * mbr);
      omega = min(omega, 20.0f * PI);
      return omega * (WHEEL_DIAMETER / 2.0f) * sign;
    }
    return 0.0f;
  }

  int torque_to_pwm(float torque) {
    const float tau_stall = 13.0f;
    const float I_stall = 2.0f;
    const float V_nom = 6.0f;
    float Kt = tau_stall / I_stall;
    float I_req = torque / Kt;
    float V_req = (I_req / I_stall) * V_nom;
    int pwm = (int)(255.0f * (V_req / V_nom));
    return constrain(pwm, -255, 255);
  }
};

// ======= Pendulum Functions =======
float get_pendulum_angle(RotaryEncoder &encoder, float offset) {
  float raw = (encoder.getPosition() * rad_per_pend_tick) - offset;
  filtered_angle = alpha_angle * raw + (1.0f - alpha_angle) * filtered_angle;
  return filtered_angle;
}

float get_pendulum_velocity(RotaryEncoder &encoder) {
  static int sign = 1;
  // int dir = encoder.getDirection();
  if (dir != 0 && dir != sign) sign = -sign;

  float vel = 0.0f;
  if (millis() - encoder.getLastTickTime() < 50) {
    unsigned long mbr = max((unsigned long)1, encoder.getMicrosBetweenRotations());
    vel = (2.0f * PI * 1000000.0f) / (PEND_CPR * mbr);
    vel = min(vel, 4.0f * PI);
    vel *= sign;
  }
  filtered_ang_vel = alpha_velocity * vel + (1.0f - alpha_velocity) * filtered_ang_vel;
  return filtered_ang_vel;
}

// ======= Global Objects & State =======
Cart cart(MOTOR_ENC_A, MOTOR_ENC_B);
RotaryEncoder pendulum(PEND_ENC_A, PEND_ENC_B);
float init_angle = 0.0f;
float state[4] = {0};
float setpoint[4] = {0};

void setup() {
  PCICR |= B00000100;
  PCMSK2 |= B00111111;
  pinMode(LMOTOR_DIR, OUTPUT);
  pinMode(RMOTOR_DIR, OUTPUT);
  pinMode(LMOTOR_PWM, OUTPUT);
  pinMode(RMOTOR_PWM, OUTPUT);
  Serial.begin(115200);
  delay(500);
  Serial.println("System starting...");
}

void loop() {
  // Check if cart has traveled less than 2 meters
  if (cart.get_position() >= 2.0f) {
    cart.set_speed(0);
    Serial.println("Limit reached: 2 meters");
    return;
  }
  state[0] = cart.get_position();
  state[1] = cart.get_velocity();
  state[2] = get_pendulum_angle(pendulum, init_angle);
  state[3] = get_pendulum_velocity(pendulum);

  float u = controller(state, setpoint);
  int pwm = cart.torque_to_pwm(u);
  if (abs(pwm) < 50) pwm = 0;
  cart.set_speed(pwm);

  static int printCount = 0;
  if (++printCount % 20 == 0) {
    Serial.print(state[2]); Serial.print(",");
    Serial.print(state[0]); Serial.print(",");
    Serial.println(pwm);
  }

  delay(10);
}

ISR(PCINT2_vect) {
  cart.encoder.tick();
  pendulum.tick();
}
