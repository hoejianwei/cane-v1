/*
  MPU-9250 / MPU-6500 Data Reader

  This sketch reads all available data from the MPU-9250 sensor
  (3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer)
  and prints it to the Serial Monitor in a simple, parsable
  Comma-Separated Value (CSV) format.

  If you are using an MPU-6500 (which has no magnetometer),
  the magnetometer values (mx, my, mz) will simply read as 0.

  OUTPUT FORMAT (single line, separated by commas):
  ax,ay,az,gx,gy,gz,mx,my,mz

  - Accel data (ax, ay, az) is in m/s^2
  - Gyro data (gx, gy, gz) is in rad/s
  - Mag data (mx, my, mz) is in µT (micro-Teslas)

  LIBRARY:
  Uses "MPU9250" by Bolder Flight Systems
  (Install via Arduino Library Manager)

  WIRING (I2C):
  - VCC: 5V
  - GND: GND
  - SCL: A5
  - SDA: A4
*/

// Use quotes for user-installed libraries.
// This was changed from <mpu9250.h> to "MPU9250.h" to fix
// the "cstddef: No such file or directory" error.
// And changed to "mpu9250.h" (lowercase) to fix "No such file".
#include "mpu9250.h"

// The 'Wire' object handles I2C communication.
// 0x68 is the default I2C address for the MPU-9250.
//
// We use "bfs::Mpu9250" because the library wraps the class
// in a "bfs" namespace and names the class "Mpu9250" (not "MPU9250").
//
// 1. Pass a pointer to Wire (use &Wire).
// 2. Use the library's I2cAddr enum for the address.
bfs::Mpu9250 mpu(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

// Variable to store the status of the sensor connection
int status;

void setup() {
  // Start Serial communication at a fast baud rate
  // This is important for sending lots of data without delay
  Serial.begin(115200);
  while (!Serial) {} // Wait for serial to connect

  Serial.println("Attempting to initialize MPU-9250...");

  // Start the I2C bus
  Wire.begin();

  // Try to initialize the MPU-9250
  // 3. Use Begin() with a capital 'B'
  status = mpu.Begin();

  // Check if initialization was successful
  if (status < 0) {
    Serial.println("MPU-9250 setup failed!");
    Serial.print("Status: ");
    Serial.println(status);
    Serial.println("Check wiring and I2C address.");
    
    // Halt the program if setup fails
    while (true) {}
  }

  Serial.println("MPU-9250 setup successful.");
  Serial.println("Now streaming 9-axis data as CSV...");
  Serial.println("Format: ax,ay,az,gx,gy,gz,mx,my,mz");
}

void loop() {
  // Read all sensor data from the MPU-9250
  // This one function reads all 9 axes.
  // 4. Use Read() with a capital 'R'
  mpu.Read();

  // --- Print Accelerometer Data (m/s^2) ---
  // 5. Use accel_x_mps2()
  Serial.print(mpu.accel_x_mps2(), 4); // Print with 4 decimal places
  Serial.print(",");
  // 6. Use accel_y_mps2()
  Serial.print(mpu.accel_y_mps2(), 4);
  Serial.print(",");
  // 7. Use accel_z_mps2()
  Serial.print(mpu.accel_z_mps2(), 4);
  Serial.print(",");

  // --- Print Gyroscope Data (rad/s) ---
  // 8. Use gyro_x_radps()
  Serial.print(mpu.gyro_x_radps(), 4);
  Serial.print(",");
  // 9. Use gyro_y_radps()
  Serial.print(mpu.gyro_y_radps(), 4);
  Serial.print(",");
  // 10. Use gyro_z_radps()
  Serial.print(mpu.gyro_z_radps(), 4);
  Serial.print(",");

  // --- Print Magnetometer Data (µT) ---
  // 11. Use mag_x_ut()
  Serial.print(mpu.mag_x_ut(), 4);
  Serial.print(",");
  // 12. Use mag_y_ut()
  Serial.print(mpu.mag_y_ut(), 4);
  Serial.print(",");
  
  // Use println() for the LAST value
  // This sends the data AND a newline character ('\n'),
  // which your C++ or Python program will use to know
  // when one full line of data has been received.
  // 13. Use mag_z_ut()
  Serial.println(mpu.mag_z_ut(), 4);

  // Delay for 50ms (20 samples per second)
  // This prevents flooding the serial port and gives
  // your computer program time to process the data.
  delay(50);
}