
/**
 * Register addresses for the gyroscope chip.
 * LSM6DS33
 */
#define FUNC_CFG_ACCESS  0x01

// FIFO queue configuration.
#define FIFO_CTRL1  0x06
#define FIFO_CTRL2  0x07
#define FIFO_CTRL3  0x08
#define FIFO_CTRL4  0x09
#define FIFO_CTRL5  0x0A

// Sign and orientation for pitch/roll/yaw.
#define ORIENT_CFG_G  0x0B

// Pad control registers.
#define INT1_CTRL  0x0D
#define INT2_CTRL  0x0E

// Reads the ID. Fixed at 0x69.
#define G_WHO_AM_I  0x0F

// Accel and gyro control
#define CTRL1_XL  0x10 // Accel: data rate power mode; full-scale selection; anti-aliasing filter bandwidth selection
#define CTRL2_G  0x11 // Gyro: data rate; full-scale selection; full-scale at 125dps
#define CTRL3_C  0x12 // Reboot memory; block data update; interrupt activation level; etc.
#define CTRL4_C  0x13 // Accel bandwidth; gyro sleep mode; interrupts; temperature in fifo; disable i2c; etc.
#define CTRL5_C  0x14 // Circular burst mode rounding; self tests
#define CTRL6_C  0x15 // Gyro: edge-sensitive trigger; data level trigger; level-sensitive latch; high-perf disable for accel
#define CTRL7_G  0x16 // Gyro: high-perf disable; high-pass filter enable; high-pass filter reset; rounding; high-pass cutoff
#define CTRL8_XL  0x17 // Accel: low-pass filter selection; slope filter high-pass filter; slope filter; low-pass on 6D
#define CTRL9_XL  0x18 // Accel: axis output enable
#define CTRL10_C  0x19 // Gyro axis enable; embedded functions etc.

#define WAKE_UP_SRC  0x1B
#define TAP_SRC    0x1C
#define D6D_SRC    0x1D
#define G_STATUS_REG  0x1E

// Temperature registers (2 bytes)
#define OUT_TEMP_L  0x20
#define OUT_TEMP_H  0x21

// Accel/gyro outputs
#define OUTX_L_G   0x22 // Gyro: x-axis (2 bytes)
#define OUTX_H_G   0x23
#define OUTY_L_G   0x24 // Gyro: y-axis (2 bytes)
#define OUTY_H_G   0x25
#define OUTZ_L_G   0x26 // Gyro: z-axis (2 bytes)
#define OUTZ_H_G   0x27
#define OUTX_L_XL  0x28 // Accel: x-axis (2 bytes)
#define OUTX_H_XL  0x29
#define OUTY_L_XL  0x2A // Accel: y-axis (2 bytes)
#define OUTY_H_XL  0x2B
#define OUTZ_L_XL  0x2C // Accel: z-axis (2 bytes)
#define OUTZ_H_XL  0x2D

// FIFO stuff
#define FIFO_STATUS1  0x3A
#define FIFO_STATUS2  0x3B
#define FIFO_STATUS3  0x3C
#define FIFO_STATUS4  0x3D
#define FIFO_DATA_OUT_L  0x3E
#define FIFO_DATA_OUT_H  0x3F

// Timestamp (24-bit word)
#define TIMESTAMP0_REG  0x40
#define TIMESTAMP1_REG  0x41
#define TIMESTAMP2_REG  0x42

// Step counter and timestamp
#define STEP_TIMESTAMP_L  0x49
#define STEP_TIMESTAMP_H  0x4A
#define STEP_COUNTER_L  0x4B
#define STEP_COUNTER_H  0x4C

// Detect significant motion
#define FUNC_SRC   0x53

// Tap-related
#define TAP_CFG   0x58
#define TAP_THS_6D  0x59
#define INT_DUR2   0x5A
#define WAKE_UP_THS  0x5B
#define WAKE_UP_DUR  0x5C

// Free fall duration.
#define FREE_FALL  0x5D

// Etc.
#define MD1_CFG    0x5E
#define MD2_CFG    0x5F

/**
 * Register addresses for the magnetometer chip.
 * LIS3MDL
 */

// Reads the device ID. Fixed at 0x3D
#define M_WHO_AM_I  0x0F

#define CTRL_REG1  0x20 // Temp sensor enable; x and y operative mode; output data rate; fast odr; self-test
#define CTRL_REG2  0x21 // Full-scale config; reboot memory; config/user reset function
#define CTRL_REG3  0x22 // Low-powe mode config; spi mode select; operating mode select (continuous single etc.)
#define CTRL_REG4  0x23 // z-axis operative mode select; big/little endian select.
#define CTRL_REG5  0x24 // Fast-read; block-data update (continuous vs. no update until MSB and LSB have been read)

#define M_STATUS_REG  0x27

#define OUT_X_L  0x28 // Output x-axis (2 bytes)
#define OUT_X_H  0x29
#define OUT_Y_L  0x2A // Output y-axis (2 bytes)
#define OUT_Y_H  0x2B
#define OUT_Z_L  0x2C // Output z-axis (2 bytes)
#define OUT_Z_H  0x2D
#define TEMP_OUT_L   0x2E // Temperature (2 bytes)
#define TEMP_OUT_H  0x2F

#define INT_CFG  0x30
#define INT_SRC  0x31
#define INT_THS_L  0x32
#define INT_THS_H  0x33

/**
 * Constants for configuring the gyroscope chip.
 * LSM6DS33
 */

// CTRL2_G
// Gyro data rate.
#define GDR_OFF  0b0000
#define GDR_13Hz  0b0001
#define GDR_26Hz  0b0010
#define GDR_52Hz  0b0011
#define GDR_104Hz  0b0100
#define GDR_208Hz  0b0101
#define GDR_416Hz  0b0110
#define GDR_833Hz  0b0111
#define GDR_1660Hz  0b1000

// Gyro full-scale selection
#define GFS_245dps  0b00
#define GFS_500dps  0b01
#define GFS_1000dps  0b10
#define GFS_2000dps  0b11

// Gyro full-scale at 125dps
#define GFS_125dps_DISABLED  0b0
#define GFS_125dps_ENABLED  0b1

// CTRL1_XL
// Accel data rate.
#define ADR_OFF  0b0000
#define ADR_13Hz  0b0001
#define ADR_26Hz  0b0010
#define ADR_52Hz  0b0011
#define ADR_104Hz  0b0100
#define ADR_208Hz  0b0101
#define ADR_416Hz  0b0110
#define ADR_833Hz  0b0111
#define ADR_1660Hz  0b1000
#define ADR_3330Hz  0b1001
#define ADR_6660Hz  0b1010

// Accel full-scale selection.
#define AFS_2g  0b00
#define AFS_4g  0b01// TODO: Assuming the datasheet settings are wrong. This is my interpretation.
#define AFS_8g  0b10
#define AFS_16g  0b11

// Accel anti-aliasing filter bandwidth. Interacts with data rate; see table 45.
#define AAFB_400Hz  0b00
#define AAFB_200Hz  0b01
#define AAFB_100Hz  0b10
#define AAFB_50Hz  0b11

// CTRL3_C
// Auto increment addresses.
#define AUTO_INC_ON  0b1
#define AUTO_INC_OFF  0b0

