#ifndef CONFIG_H
#define CONFIG_H

// Wi-Fi Credentials

#define WIFI_SSID "Stage V(ericu)"
#define WIFI_PASSWORD "parolaelamata"

//#define WIFI_SSID "Casa_Mica"
//#define WIFI_PASSWORD "C3q7nKp4xD"

//#define WIFI_SSID "DIGI_f94868"
//#define WIFI_PASSWORD "a7ee06b7"

#define DSHOT_THROTTLE_MIN 100
#define DSHOT_THROTTLE_MAX 600

#define INTERN_REG_FREQ 150  // Hz
#define EXTERN_REG_FREQ 100  // Hz
#define MOTOR_FREQ 250       // Hz
#define DSHOT300 300

#define MAC_ADDRESS "a0:fa:9c:1b:4c:8b"

#define DSHOT_MODE DSHOT300

#define ROLL_FACTOR 1.0f
#define PITCH_FACTOR 1.0f
#define YAW_FACTOR 1.0f
#define ALT_FACTOR 1.0f

// UDP Configuration
#define UDP_ADDRESS "192.168.1.37"
#define UDP_PORT 5005

#define WINDOW_SIZE 20

#define MAX_PITCH 10
#define MAX_ROLL 10
#define YAW_INC_RATE 2
#define ALT_INC_RATE 1

#define MAX_ALT 2000.0f
#define MIN_ALT 0.0f

#define STICK_DEADZONE 0.05

//valori key pentru constantele fiecarui PID din bucla interna
#define ROLL_PID_KP "roll_pid_kp"
#define ROLL_PID_KI "roll_pid_ki"
#define ROLL_PID_KD "roll_pid_kd"

#define PITCH_PID_KP "pitch_pid_kp"
#define PITCH_PID_KI "pitch_pid_ki"
#define PITCH_PID_KD "pitch_pid_kd"

#define YAW_PID_KP "yaw_pid_kp"
#define YAW_PID_KI "yaw_pid_ki"
#define YAW_PID_KD "yaw_pid_kd"

#define ALT_PID_KP "alt_pid_kp"
#define ALT_PID_KI "alt_pid_ki"
#define ALT_PID_KD "alt_pid_kd"

//valori key pentru constantele fiecarui PID din bucla externa
#define ROLL_PID_KP_E "roll_pid_kp_e"
#define ROLL_PID_KI_E "roll_pid_ki_e"
#define ROLL_PID_KD_E "roll_pid_kd_e"

#define PITCH_PID_KP_E "pitch_pid_kp_e"
#define PITCH_PID_KI_E "pitch_pid_ki_e"
#define PITCH_PID_KD_E "pitch_pid_kd_e"

#define YAW_PID_KP_E "yaw_pid_kp_e"
#define YAW_PID_KI_E "yaw_pid_ki_e"
#define YAW_PID_KD_E "yaw_pid_kd_e"

#define ALT_PID_KP_E "alt_pid_kp_e"
#define ALT_PID_KI_E "alt_pid_ki_e"
#define ALT_PID_KD_E "alt_pid_kd_e"

#define BATTERY_PIN 34

#define FL_MOTOR_PIN GPIO_NUM_18
#define BL_MOTOR_PIN GPIO_NUM_19
#define FR_MOTOR_PIN GPIO_NUM_26
#define BR_MOTOR_PIN GPIO_NUM_25

#define GYRO_LED_PIN 32
#define BT_CONN_PIN  33
#define LOW_BATTERY_PIN 27

#define Resistor1 10000.0  // 10kΩ
#define Resistor2 10000.0  // 10kΩ

#endif  // CONFIG_H
