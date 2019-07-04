// javascript driver for LSM9DS1, port of
// https://github.com/adafruit/Adafruit_CircuitPython_LSM9DS1

// Internal constants and register values:
const _LSM9DS1_ADDRESS_ACCELGYRO       = 0x6B;
const _LSM9DS1_ADDRESS_MAG             = 0x1E;
const _LSM9DS1_XG_ID                   = 0b01101000;
const _LSM9DS1_MAG_ID                  = 0b00111101;
const _LSM9DS1_ACCEL_MG_LSB_2G         = 0.061;
const _LSM9DS1_ACCEL_MG_LSB_4G         = 0.122;
const _LSM9DS1_ACCEL_MG_LSB_8G         = 0.244;
const _LSM9DS1_ACCEL_MG_LSB_16G        = 0.732;
const _LSM9DS1_MAG_MGAUSS_4GAUSS       = 0.14;
const _LSM9DS1_MAG_MGAUSS_8GAUSS       = 0.29;
const _LSM9DS1_MAG_MGAUSS_12GAUSS      = 0.43;
const _LSM9DS1_MAG_MGAUSS_16GAUSS      = 0.58;
const _LSM9DS1_GYRO_DPS_DIGIT_245DPS   = 0.00875;
const _LSM9DS1_GYRO_DPS_DIGIT_500DPS   = 0.01750;
const _LSM9DS1_GYRO_DPS_DIGIT_2000DPS  = 0.07000;
const _LSM9DS1_TEMP_LSB_DEGREE_CELSIUS = 8; // 1°C = 8, 25° = 200, etc.
const _LSM9DS1_REGISTER_WHO_AM_I_XG    = 0x0F;
const _LSM9DS1_REGISTER_CTRL_REG1_G    = 0x10;
const _LSM9DS1_REGISTER_CTRL_REG2_G    = 0x11;
const _LSM9DS1_REGISTER_CTRL_REG3_G    = 0x12;
const _LSM9DS1_REGISTER_TEMP_OUT_L     = 0x15;
const _LSM9DS1_REGISTER_TEMP_OUT_H     = 0x16;
const _LSM9DS1_REGISTER_STATUS_REG     = 0x17;
const _LSM9DS1_REGISTER_OUT_X_L_G      = 0x18;
const _LSM9DS1_REGISTER_OUT_X_H_G      = 0x19;
const _LSM9DS1_REGISTER_OUT_Y_L_G      = 0x1A;
const _LSM9DS1_REGISTER_OUT_Y_H_G      = 0x1B;
const _LSM9DS1_REGISTER_OUT_Z_L_G      = 0x1C;
const _LSM9DS1_REGISTER_OUT_Z_H_G      = 0x1D;
const _LSM9DS1_REGISTER_CTRL_REG4      = 0x1E;
const _LSM9DS1_REGISTER_CTRL_REG5_XL   = 0x1F;
const _LSM9DS1_REGISTER_CTRL_REG6_XL   = 0x20;
const _LSM9DS1_REGISTER_CTRL_REG7_XL   = 0x21;
const _LSM9DS1_REGISTER_CTRL_REG8      = 0x22;
const _LSM9DS1_REGISTER_CTRL_REG9      = 0x23;
const _LSM9DS1_REGISTER_CTRL_REG10     = 0x24;
const _LSM9DS1_REGISTER_OUT_X_L_XL     = 0x28;
const _LSM9DS1_REGISTER_OUT_X_H_XL     = 0x29;
const _LSM9DS1_REGISTER_OUT_Y_L_XL     = 0x2A;
const _LSM9DS1_REGISTER_OUT_Y_H_XL     = 0x2B;
const _LSM9DS1_REGISTER_OUT_Z_L_XL     = 0x2C;
const _LSM9DS1_REGISTER_OUT_Z_H_XL     = 0x2D;
const _LSM9DS1_REGISTER_WHO_AM_I_M     = 0x0F;
const _LSM9DS1_REGISTER_CTRL_REG1_M    = 0x20;
const _LSM9DS1_REGISTER_CTRL_REG2_M    = 0x21;
const _LSM9DS1_REGISTER_CTRL_REG3_M    = 0x22;
const _LSM9DS1_REGISTER_CTRL_REG4_M    = 0x23;
const _LSM9DS1_REGISTER_CTRL_REG5_M    = 0x24;
const _LSM9DS1_REGISTER_STATUS_REG_M   = 0x27;
const _LSM9DS1_REGISTER_OUT_X_L_M      = 0x28;
const _LSM9DS1_REGISTER_OUT_X_H_M      = 0x29;
const _LSM9DS1_REGISTER_OUT_Y_L_M      = 0x2A;
const _LSM9DS1_REGISTER_OUT_Y_H_M      = 0x2B;
const _LSM9DS1_REGISTER_OUT_Z_L_M      = 0x2C;
const _LSM9DS1_REGISTER_OUT_Z_H_M      = 0x2D;
const _LSM9DS1_REGISTER_CFG_M          = 0x30;
const _LSM9DS1_REGISTER_INT_SRC_M      = 0x31;
const _MAGTYPE                         = true;
const _XGTYPE                          = false;
const _SENSORS_GRAVITY_STANDARD        = 9.80665;

// User facing constants/module globals.
const ACCELRANGE_2G                = (0b00 << 3);
const ACCELRANGE_16G               = (0b01 << 3);
const ACCELRANGE_4G                = (0b10 << 3);
const ACCELRANGE_8G                = (0b11 << 3);
const MAGGAIN_4GAUSS               = (0b00 << 5);  // +/- 4 gauss
const MAGGAIN_8GAUSS               = (0b01 << 5);  // +/- 8 gauss
const MAGGAIN_12GAUSS              = (0b10 << 5);  // +/- 12 gauss
const MAGGAIN_16GAUSS              = (0b11 << 5);  // +/- 16 gauss
const GYROSCALE_245DPS             = (0b00 << 3);  // +/- 245 degrees/s rotation
const GYROSCALE_500DPS             = (0b01 << 3);  // +/- 500 degrees/s rotation
const GYROSCALE_2000DPS            = (0b11 << 3);  // +/- 2000 degrees/s rotation

/**
 * Calculate the 2s complement of int:val
 */
function twos_comp(val, bits) {
    if (val & (1 << (bits - 1))) {
        val = val - (1 << bits);
    }
    return val;
}

const struct = {
    unpack_from(str, buff) {
        if (str !== '<hhh') {
            throw new Error(`I only support <hhh unpacking`);
        }
        const raw_x = twos_comp(buff[1] << 8 | buff[0], 16);
        const raw_y = twos_comp(buff[3] << 8 | buff[2], 16);
        const raw_z = twos_comp(buff[5] << 8 | buff[4], 16);
        return [raw_x, raw_y, raw_z];
    }
};

/**
 * Driver for the LSM9DS1 accelerometer, magnetometer, gyroscope.
 */
class LSM9DS1 {
    constructor(options) {
        Object.assign(this, options);

        // soft reset & reboot accel/gyro
        this._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG8, 0x05);
        // soft reset & reboot magnetometer
        this._write_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M, 0x0C);
        // time.sleep(0.01);
        // Check ID registers.
        if (
           this._read_u8(_XGTYPE, _LSM9DS1_REGISTER_WHO_AM_I_XG) !== _LSM9DS1_XG_ID
           || this._read_u8(_MAGTYPE, _LSM9DS1_REGISTER_WHO_AM_I_M) !== _LSM9DS1_MAG_ID
        ) {
            throw new Error('Could not find LSM9DS1, check wiring!');
        }
        // enable gyro continuous
        this._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G, 0xC0); // on XYZ
        // Enable the accelerometer continous
        this._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG5_XL, 0x38);
        this._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL, 0xC0);
        // enable mag continuous
        this._write_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG3_M, 0x00);
        // Set default ranges for the various sensors
        this._accel_mg_lsb = null;
        this._mag_mgauss_lsb = null;
        this._gyro_dps_digit = null;
        this.set_accel_range(ACCELRANGE_2G);
        this.set_mag_gain(MAGGAIN_4GAUSS);
        this.set_gyro_scale(GYROSCALE_245DPS);
    }

    /**
     * The accelerometer range.  Must be a value of:
     - ACCELRANGE_2G
     - ACCELRANGE_4G
     - ACCELRANGE_8G
     - ACCELRANGE_16G
     * @returns {number}
     */
    accel_range() {
        const reg = this._read_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL);
        return (reg & 0b00011000) & 0xFF;
    }

    set_accel_range(val) {
        if ([
            ACCELRANGE_2G,
            ACCELRANGE_4G,
            ACCELRANGE_8G,
            ACCELRANGE_16G
        ].indexOf(val) < 0) {
            throw new Error('Unexpected val for set_accel_range');
        }
        let reg = this._read_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL);
        reg = (reg & ~(0b00011000)) & 0xFF;
        reg |= val;
        this._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL, reg);
        if (val === ACCELRANGE_2G) {
            this._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_2G;
        } else if (val === ACCELRANGE_4G) {
            this._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_4G;
        } else if (val === ACCELRANGE_8G) {
            this._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_8G;
        } else if (val === ACCELRANGE_16G) {
            this._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_16G;
        }
    }

    /**
     * The magnetometer gain.  Must be a value of:
     - MAGGAIN_4GAUSS
     - MAGGAIN_8GAUSS
     - MAGGAIN_12GAUSS
     - MAGGAIN_16GAUSS
     */
    mag_gain() {
        const reg = this._read_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M);
        return (reg & 0b01100000) & 0xFF;
    }

    set_mag_gain(val) {
        if ([
            MAGGAIN_4GAUSS,
            MAGGAIN_8GAUSS,
            MAGGAIN_12GAUSS,
            MAGGAIN_16GAUSS
        ].indexOf(val) < 0) {
            throw new Error('unexpected val for set_mag_gain');
        }
        let reg = this._read_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M);
        reg = (reg & ~(0b01100000)) & 0xFF;
        reg |= val;
        this._write_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M, reg);
        if (val === MAGGAIN_4GAUSS) {
            this._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_4GAUSS;
        } else if (val === MAGGAIN_8GAUSS) {
            this._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_8GAUSS;
        } else if (val === MAGGAIN_12GAUSS) {
            this._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_12GAUSS;
        } else if (val === MAGGAIN_16GAUSS) {
            this._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_16GAUSS;
        }
    }

    /**
     * The gyroscope scale.  Must be a value of:
     - GYROSCALE_245DPS
     - GYROSCALE_500DPS
     - GYROSCALE_2000DPS
     */
    gyro_scale() {
        const reg = this._read_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G);
        return (reg & 0b00011000) & 0xFF;
    }

    set_gyro_scale(val) {
        if ([
            GYROSCALE_245DPS,
            GYROSCALE_500DPS,
            GYROSCALE_2000DPS
        ].indexOf(val) < 0) {
            throw new Error('unexpected val for set_mag_gain');
        }

        let reg = this._read_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G);
        reg = (reg & ~(0b00011000)) & 0xFF;
        reg |= val;
        this._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G, reg);
        if (val === GYROSCALE_245DPS) {
            this._gyro_dps_digit = _LSM9DS1_GYRO_DPS_DIGIT_245DPS;
        } else if (val === GYROSCALE_500DPS) {
            this._gyro_dps_digit = _LSM9DS1_GYRO_DPS_DIGIT_500DPS;
        } else if (val === GYROSCALE_2000DPS) {
            this._gyro_dps_digit = _LSM9DS1_GYRO_DPS_DIGIT_2000DPS;
        }
    }

    /**
     * Read the raw accelerometer sensor values and return it as a
     3-tuple of X, Y, Z axis values that are 16-bit unsigned values.  If you
     want the acceleration in nice units you probably want to use the
     accelerometer property!
     */
    read_accel_raw() {
        // Read the accelerometer
        const buff = this._read_bytes(
            _XGTYPE,
            0x80 | _LSM9DS1_REGISTER_OUT_X_L_XL,
            6
        );
        return struct.unpack_from('<hhh', buff.slice(0, 6));
    }

    /**
     * The accelerometer X, Y, Z axis values as a 3-tuple of m/s^2 values.
     */
    acceleration() {
        const raw = this.read_accel_raw();
        return raw.map((x) => {
            return x * this._accel_mg_lsb / 1000.0 * _SENSORS_GRAVITY_STANDARD;
        });
    }

    /**
     * Read the raw magnetometer sensor values and return it as a
     3-tuple of X, Y, Z axis values that are 16-bit unsigned values.  If you
     want the magnetometer in nice units you probably want to use the
     magnetometer property!
     */
    read_mag_raw() {
        // Read the magnetometer
        const buff = this._read_bytes(_MAGTYPE, 0x80 | _LSM9DS1_REGISTER_OUT_X_L_M, 6);
        return struct.unpack_from('<hhh', buff.slice(0, 6));
    }

    /**
     * The magnetometer X, Y, Z axis values as a 3-tuple of
     gauss values.
     */
    magnetic() {
        const raw = this.read_mag_raw();
        return raw.map((x) => {
            return x * this._mag_mgauss_lsb / 1000.0;
        });
    }

    /**
     * Read the raw gyroscope sensor values and return it as a
     3-tuple of X, Y, Z axis values that are 16-bit unsigned values.  If you
     want the gyroscope in nice units you probably want to use the
     gyroscope property!
     */
    read_gyro_raw() {
        // Read the gyroscope
        const buff = this._read_bytes(_XGTYPE, 0x80 | _LSM9DS1_REGISTER_OUT_X_L_G, 6);
        return struct.unpack_from('<hhh', buff.slice(0, 6));
    }

    /**
     * The gyroscope X, Y, Z axis values as a 3-tuple of degrees/second values.
     */
    gyro() {
        const raw = this.read_gyro_raw();
        return raw.map((x) => { return x * this._gyro_dps_digit; });
    }

    /**
     * Read the raw temperature sensor value and return it as a 12-bit
     signed value.  If you want the temperature in nice units you probably
     want to use the temperature property!
     */
    read_temp_raw() {
        // Read temp sensor
        const buff = this._read_bytes(_XGTYPE, 0x80 | _LSM9DS1_REGISTER_TEMP_OUT_L, 2);
        const temp = ((buff[1] << 8) | buff[0]) >> 4;
        return twos_comp(temp, 12);
    }

    /**
     * The temperature of the sensor in degrees Celsius.
     *
     * This is just a guess since the starting point (21C here) isn't documented :(
     * See discussion from:
     * https://github.com/kriswiner/LSM9DS1/issues/3
     */
    temperature() {
        let temp = this.read_temp_raw();
        temp = 27.5 + temp / 16;
        return temp;
    }

    /**
     * Read an 8-bit unsigned value from the specified 8-bit address.
     * The sensor_type boolean should be _MAGTYPE when talking to the
     * magnetometer, or _XGTYPE when talking to the accel or gyro.
     * MUST be implemented by subclasses!
     *
     * @param sensor_type
     * @param address
     * @private
     */
    _read_u8(sensor_type, address) {
        throw new Error('Not implemented');
    }

    /**
     * Read a count number of bytes into buffer from the provided 8-bit
     * register address.  The sensor_type boolean should be _MAGTYPE when
     * talking to the magnetometer, or _XGTYPE when talking to the accel or
     * gyro.  MUST be implemented by subclasses!
     *
     * @param sensor_type
     * @param address
     * @param count
     * @param buf
     * @private
     */
    _read_bytes(sensor_type, address, count, buf) {
        throw new Error('Not implemented');
    }

    /**
     * Write an 8-bit unsigned value to the specified 8-bit address.
     * The sensor_type boolean should be _MAGTYPE when talking to the
     * magnetometer, or _XGTYPE when talking to the accel or gyro.
     * MUST be implemented by subclasses!
     *
     * @param sensor_type
     * @param address
     * @param val
     * @private
     */
    _write_u8(sensor_type, address, val) {
        throw new Error('Not implemented');
    }
}

module.exports = LSM9DS1;