using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using SecretLabs.NETMF.Hardware;
using SecretLabs.NETMF.Hardware.NetduinoPlus;
using JollySamurai.I2C.Devices.InvenSense;

namespace JollySamurai.I2C
{
    public class Program
    {
        private static I2CBus _i2cBus;
        private static Devices.InvenSense.MPU6050 _mpu;
        private static Devices.InvenSense.DigitalMotionProcessor _dmp;

        private static bool _newDataAvailable = false;
        private static InterruptPort _interruptPort;

        public static void Main()
        {
            Debug.EnableGCMessages(true);

            _i2cBus = new I2CBus();
            _mpu = new Devices.InvenSense.MPU6050(_i2cBus, 0x68, 100);
            _dmp = _mpu.DigitalMotionProcessor;

            if (_mpu.Initialize() == false) {
                Debug.Print("Failed to initialize device");
                return;
            }
            
            // get/set hardware configuration. start gyro
            // wake up all sensors
            _mpu.Sensors = Sensors.Gyro | Sensors.Accelerometer;

            // push both gyro and accel data into the FIFO
            _mpu.ConfigureFifo(Sensors.Gyro | Sensors.Accelerometer);
            _mpu.SampleRate = DigitalMotionProcessor.DefaultFifoRate;

            /* Read back configuration in case it was set improperly. */
            Debug.Print("TODO: mpu_get_sample_rate(&gyro_rate);");
            Debug.Print("TODO: mpu_get_gyro_fsr(&gyro_fsr);");
            Debug.Print("TODO: mpu_get_accel_fsr(&accel_fsr);");
            
            if (_dmp.Initialize() == false) {
                Debug.Print("DMP Initialization failed");
                return;
            }

            /*
             * Known Bug -
             * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
             * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
             * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
             * there will be a 25Hz interrupt from the MPU device.
             *
             * There is a known issue in which if you do not enable DMP_FEATURE_TAP
             * then the interrupts will be at 200Hz even if fifo rate
             * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
             *
             * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
             */
            _dmp.Enable(Feature.Quaternion | Feature.SendRawAccelerometerData | Feature.SendCalibratedGyroData | Feature.CalibrateGyro | Feature.Tap, 50);

            _interruptPort = new InterruptPort(Pins.GPIO_PIN_A0, false, Port.ResistorMode.Disabled, Port.InterruptMode.InterruptEdgeHigh);
            _interruptPort.OnInterrupt += new NativeEventHandler(_interruptPort_OnInterrupt);
            _interruptPort.EnableInterrupt();
            
            short[] gyro = new short[3];
            short[] accelerometer = new short[3];
            float[] gravity = new float[3];
            float[] ypr = new float[3];
            float[] gravity2 = new float[3];
            float[] ypr2 = new float[3];
            int more = 0;
            long[] quaternion = new long[4];
            Quaternion q = new Quaternion();
            Quaternion q2 = new Quaternion();

            while (true) {
                if (_newDataAvailable == true) {
                    /* This function gets new data from the FIFO when the DMP is in
                     * use. The FIFO can contain any combination of gyro, accel,
                     * quaternion, and gesture data. The sensors parameter tells the
                     * caller which data fields were actually populated with new data.
                     * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
                     * the FIFO isn't being filled with accel data.
                     * The driver parses the gesture data to determine if a gesture
                     * event has occurred; on an event, the application will be notified
                     * via a callback (assuming that a callback function was properly
                     * registered). The more parameter is non-zero if there are
                     * leftover packets in the FIFO.
                     */
                    if (_dmp.ReadFifo(gyro, accelerometer, quaternion, ref more) != 0) {
                        continue;
                    }

                    if (more == 0) {
                        _newDataAvailable = false;
                    }

                    q.From(
                        (float) quaternion[0] / 16384.0f,
                        (float) quaternion[1] / 16384.0f,
                        (float) quaternion[2] / 16384.0f,
                        (float) quaternion[3] / 16384.0f
                    );

                    q2.From(
                        (float) quaternion[0],
                        (float) quaternion[1],
                        (float) quaternion[2],
                        (float) quaternion[3]
                    );
                    
                    _dmp.GetGravity(gravity, q);
                    _dmp.GetYawPitchRoll(ypr, q, gravity);

                    _dmp.GetGravity(gravity2, q2);
                    _dmp.GetYawPitchRoll(ypr2, q2, gravity2);

                    ypr[0] *= 180.0f / (float) System.Math.PI;
                    ypr[1] *= 180.0f / (float) System.Math.PI;
                    ypr[2] *= 180.0f / (float) System.Math.PI;

                    ypr2[0] *= 180.0f / (float) System.Math.PI;
                    ypr2[1] *= 180.0f / (float) System.Math.PI;
                    ypr2[2] *= 180.0f / (float) System.Math.PI;

                    //Debug.Print("GRA: " + (gravity[0]).ToString() + "\t\t" + (gravity[1]).ToString() + "\t\t" + (gravity[2]).ToString());
                    Debug.Print("YPR: " + (ypr[0]).ToString() + "\t" + (ypr[1]).ToString() + "\t" + (ypr[2]).ToString() + "\t" + "YPR2: " + (ypr2[0]).ToString() + "\t" + (ypr2[1]).ToString() + "\t" + (ypr2[2]).ToString());
                    //Debug.Print("GX: " + (gyro[0]).ToString() + " GY: " + (gyro[1]).ToString() + " GZ: " + (gyro[2]).ToString() + " more: " + more.ToString());
                    //Debug.Print("AX: " + (accel_short[0]).ToString() + " AY: " + (accel_short[1]).ToString() + " AZ: " + (accel_short[2]).ToString() + " more: " + more.ToString());
                    //Debug.Print("QX: " + (quat[0]).ToString() + " QY: " + (quat[1]).ToString() + " QZ: " + (quat[2]).ToString() + " QW: " + quat[3].ToString());
                }

                
                Thread.Sleep(0);
            }
        }

        private static void _interruptPort_OnInterrupt(uint data1, uint data2, DateTime time)
        {
            _newDataAvailable = true;
        }

    }
}
