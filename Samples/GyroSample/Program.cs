using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using SecretLabs.NETMF.Hardware;
using SecretLabs.NETMF.Hardware.NetduinoPlus;

using JollySamurai.I2C;
using JollySamurai.I2C.Devices;
using JollySamurai.I2C.Math;

namespace GyroSample
{
    public class Program
    {
        private static I2CBus _i2cBus;
        private static MPU6050 _mpu;

        public static void Main()
        {
            _i2cBus = new I2CBus();
            _mpu = new MPU6050(_i2cBus, MPU6050.AddressLow, 400);

            Debug.Print("Testing device connections...");

            if (_mpu.TestConnection() == false) {
                Debug.Print("Connection failed");
            } else {
                Debug.Print("Initializing device...");

                _mpu.Initialize();

                short accelX, accelY, accelZ;
                short gyroX, gyroY, gyroZ;

                while (true) {
                    _mpu.GetMotion6(out accelX, out accelY, out accelZ, out gyroX, out gyroY, out gyroZ);

                    accelX /= 16384;
                    accelY /= 16384;
                    accelZ /= 16384;

                    gyroX /= 131;
                    gyroY /= 131;
                    gyroZ /= 131;

                    Debug.Print("Accel[" + accelX.ToString() + "," + accelY.ToString() + "," + accelZ.ToString() + "] "
                            + "Gyro[" + gyroX.ToString() + "," + gyroY.ToString() + "," + gyroZ.ToString() + "]");
                }
            }
        }
    }
}
