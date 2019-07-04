const LSM9DS1 = require('./lsm9ds1');

const _MAGTYPE                         = true;
const _LSM9DS1_ADDRESS_ACCELGYRO       = 0x6B;
const _LSM9DS1_ADDRESS_MAG             = 0x1E;

/**
 * Driver for the LSM9DS1 connect over I2C.
 */
class Lsm9ds1I2c extends LSM9DS1 {
    _read_u8(sensor_type, address) {
        const buff = this._read_bytes(sensor_type, address, 1);
        return buff[0];
    }

    _read_bytes(sensor_type, address, count) {
        const deviceAddress = (sensor_type === _MAGTYPE) ? _LSM9DS1_ADDRESS_MAG : _LSM9DS1_ADDRESS_ACCELGYRO;

        //readSync(address, register, length)
        return this.i2c.readSync(
            deviceAddress,
            address,
            count
        );
    }

    _write_u8(sensor_type, address, val) {
        const deviceAddress = sensor_type === _MAGTYPE ? _LSM9DS1_ADDRESS_MAG : _LSM9DS1_ADDRESS_ACCELGYRO;
        this.i2c.writeSync(
            deviceAddress,
            address,
            Buffer.from([val])
        );
    }
}

module.exports = Lsm9ds1I2c;