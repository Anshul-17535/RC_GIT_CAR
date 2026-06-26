# One ESP32 telemetry frame (decoded from the UART JSON packet).
std_msgs/Header header
float32 battery     # volts
float32 roll        # degrees  (IMU; 0 until the ESP32 IMU is wired)
float32 pitch       # degrees  (IMU; 0 until the ESP32 IMU is wired)
bool    busy        # true while a timed maneuver OR motor test is running ("auto")
int16   pwm_left    # signed PWM last applied to the left side  (-255..255)
int16   pwm_right   # signed PWM last applied to the right side (-255..255)
