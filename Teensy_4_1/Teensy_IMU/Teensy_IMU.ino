#include <Adafruit_BNO08x.h>
#include <vector>

#define USE_SPI
// #define DEBUG_TIMER

#define SERIAL_USB Serial
#define SERIAL_TEENSY Serial6
#define SERIAL_BAUDRATE_TEENSY 500000   // Teens 4.1 serial connection baudrate
#define WIRE_IMU Wire                   // Using I2C bus 1: SCL, SDA
#define SPI_IMU SPI1                    // Using SPI bus 1: MISO1, MOSI1

#define PI 3.1415926535897932384626433832795    // pi
#define RAD2DEG 180/PI                          // radians to degrees conversion factor

#define MAX_DATA_SIZE 128
#define BNO08X_CS 38
#define BNO08X_INT 35
#define BNO08X_RESET 34
#define LED 13

const uint8_t k_dtPrint = 50;               // USB print period in ms
const uint8_t k_dtSerial = 5;               // serial write period in ms

uint32_t t_last_print = 0;                  // timestamp at last USB print
uint32_t t_last_serial = 0;                 // timestamp at last serial write

std::vector<float> quat = {0, 0, 0, 0};     // quaternion vector given by the IMU algorithm
uint8_t quat_accuracy = 0;
std::vector<float> rpy = {0, 0, 0};         // roll pitch yaw calculated from the quaternion in degrees
std::vector<float> gyro = {0, 0, 0};        // calibrated gyroscope output values

char sent_data[MAX_DATA_SIZE] = "";

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t imu_sensor_value;

void setup() {
  delay(100);   // short delay after the start to ensure the first serial print is not missed over USB
  SERIAL_USB.println("Teensy started.\n");
  SERIAL_TEENSY.begin(SERIAL_BAUDRATE_TEENSY);

  pinMode(LED, OUTPUT);
  initIMU();
}


void loop() {
  uint32_t t_current = millis();
  readIMU();
  if (t_current - t_last_serial >= k_dtSerial) {
    snprintf(sent_data, sizeof(sent_data), "%.2f:%.2f:%.2f:%.2f:%.2f:%.2f ", rpy[1], -rpy[0], rpy[2], RAD2DEG*gyro[1], RAD2DEG*-gyro[0], RAD2DEG*gyro[2]);
    SERIAL_TEENSY.print(sent_data);
    t_last_serial = t_current;
  }

  if (t_current - t_last_print >= k_dtPrint) {
    SERIAL_USB.println(sent_data);
    t_last_print = t_current;
  }

}

void initIMU() {
  uint32_t t_current = millis();
  bool success = false;

  while (!success && millis() - t_current < 1000) {
    #ifdef USE_SPI
    SPI1.setMISO(39);
    if (bno08x.begin_SPI(BNO08X_CS, BNO08X_INT, &SPI1)) {
      success = true;
      bno08x.wasReset();                                    // do NOT delete this line; necessary to clear the reset flag after power on
    }

    #else
    if (bno08x.begin_I2C(0x4A, &WIRE_IMU, 0)) {  // begin_I2C(uint8_t i2c_addr = BNO08x_I2CADDR_DEFAULT, TwoWire *wire = &Wire, int32_t sensor_id = 0);
      success = true;
      bno08x.wasReset();                    // do NOT delete this line; necessary to clear the reset flag after power on
    }
    #endif
  }

  if (!success) {      
    digitalWrite(LED, HIGH);
    SERIAL_USB.println("Couldn't find BNO08x chip.\n");
    while (true) {}

  } else {
    bno08x.wasReset();                    // do NOT delete this line; necessary to clear the reset flag after power on
    SERIAL_USB.println("BNO08x successfully connected.\n\n");
  }

  setReports();
  SERIAL_USB.println("BNO08x initialized.\n---------------------------------------------\n\n");
}

// fetch IMU data updates and raise stop flag is IMU is disconnected
void readIMU() {
  if (bno08x.wasReset()) {
    SERIAL_USB.println("IMU was reset.\n\n");
    setReports();
  }

  #ifdef DEBUG_TIMER
  elapsedMicros timer;
  #endif

  bool available = bno08x.getSensorEvent(&imu_sensor_value);

  #ifdef DEBUG_TIMER
  SERIAL_USB.print("fetching report took ");
  SERIAL_USB.print(timer);
  SERIAL_USB.println(" microseconds");
  if (available) {
    SERIAL_USB.println("report is available");
  }
  #endif

  if (available) {
    switch (imu_sensor_value.sensorId) {
      case SH2_ROTATION_VECTOR:
        quat[0] = imu_sensor_value.un.rotationVector.real;
        quat[1] = imu_sensor_value.un.rotationVector.i;
        quat[2] = imu_sensor_value.un.rotationVector.j;
        quat[3] = imu_sensor_value.un.rotationVector.k;
        quat_accuracy = imu_sensor_value.un.rotationVector.accuracy;
        quat2rpy();
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        gyro[0] = imu_sensor_value.un.gyroscope.x;
        gyro[1] = imu_sensor_value.un.gyroscope.y;
        gyro[2] = imu_sensor_value.un.gyroscope.z;
        break;
    }
  }
}

// set desired IMU reports
void setReports() {
  SERIAL_USB.println("Setting desired reports.\n\n");
  if (! bno08x.enableReport(SH2_ROTATION_VECTOR, 2500)) {
    SERIAL_USB.println("Could not enable rotation vector.\n\n");
    while (true) {}
  }

  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 2500)) {
    SERIAL_USB.println("Could not enable gyroscope.\n\n");
    while (true) {}
  }
}

// convert quaternion to roll pitch yaw
void quat2rpy() {
  float sinr_cosp = 2 * (quat[0] * quat[1] + quat[2] * quat[3]);
  float cosr_cosp = 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]);
  rpy[0] = atan2(sinr_cosp, cosr_cosp) * 180 / PI;

  float sinp = 2 * (quat[0] * quat[2] - quat[3] * quat[1]);
  if (abs(sinp) >= 1) {
    rpy[1] = 90;
    if (sinp < 0) {
      rpy[1] *= -1;
    }
  } else {
    rpy[1] = asin(sinp) * 180 / PI;
  }

  float siny_cosp = 2 * (quat[0] * quat[3] + quat[1] * quat[2]);
  float cosy_cosp = 1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]);
  rpy[2] = atan2(siny_cosp, cosy_cosp) * 180 / PI;
}
