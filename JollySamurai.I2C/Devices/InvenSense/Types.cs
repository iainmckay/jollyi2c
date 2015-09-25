using System;
using Microsoft.SPOT;

namespace JollySamurai.I2C.Devices.InvenSense
{
    [Flags]
    public enum Feature
    {
        None = 0,

        /// <summary>
        /// Detect taps along the X, Y, and Z axes.
        /// </summary>
        Tap = 0x001,

        /// <summary>
        ///  Google's screen rotation algorithm. Triggers an event at the four orientations where the screen should rotate.
        /// </summary>
        AndroidOrientation = 0x002,

        /// <summary>
        /// Generate a gyro-only quaternion on the DMP at 200Hz. Integrating the gyro data at higher rates reduces numerical
        /// errors (compared to integration on the MCU at a lower sampling rate).
        /// </summary>
        Quaternion = 0x004,
        Pedometer = 0x008,

        /// <summary>
        /// Generate a gyro/accel quaternion on the DMP at 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
        /// </summary>
        Quaternion6x = 0x010,

        /// <summary>
        /// Calibrates the gyro data after eight seconds of no motion.
        /// </summary>
        CalibrateGyro = 0x020,

        /// <summary>
        /// Add raw accelerometer data to the FIFO.
        /// </summary>
        SendRawAccelerometerData = 0x040,

        /// <summary>
        /// Add raw gyro data to the FIFO.
        /// </summary>
        SendRawGyroData = 0x080,

        /// <summary>
        /// Add calibrated gyro data to the FIFO. Cannot be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
        /// </summary>
        SendCalibratedGyroData = 0x100,

        SendAnyGyroData = SendRawGyroData | SendCalibratedGyroData
    }

    public enum ClockSource : byte
    {
        Internal,
        Pll
    } 

    public enum Register : byte
    {
        AccelerometerConfig = 0x1C,
        AccelerometerOffset = 0x06,
        DigitalLowPassFilterBandwidth = 0x1A,
        DmpInterruptStatus = 0x39,
        EnableFifo = 0x23,
        EnableInterrupt = 0x38,
        FifoCount = 0x72,
        FifoReadWrite = 0x74,
        GyroConfig = 0x1B,
        I2cMaster = 0x24,
        InterruptPinConfig = 0x37,
        InterruptStatus = 0x3A,
        MemoryBankSelect = 0x6D,
        MemoryReadWrite = 0x6F,
        MemoryStartAddress = 0x6E,
        MotionDuration = 0x20,
        MotionThreshold = 0x1F,
        PowerManagement1 = 0x6B,
        PowerManagement2 = 0x6C,
        ProductId = 0x0C,
        ProgramStartAddress = 0x70,
        RateDivisor = 0x19,
        RawAccelerometer = 0x3B,
        RawGyro = 0x43,
        Temperature = 0x41,
        UserControl = 0x6A,
        WhoAmI = 0x75
    }

    public enum GyroRange : byte
    {
        R250 = 0,
        R500,
        R1000,
        R2000,

        Invalid = 0xFF
    }

    public enum AccelerometerRange : byte
    {
        R2G = 0,
        R4G,
        R8G,
        R16G,

        Invalid = 0xFF
    }

    public enum DigitalLowPassFilterBandwidth : byte
    {
        F256HzNoLpf2 = 0,
        F188Hz,
        F98Hz,
        F42Hz,
        F20Hz,
        F10Hz,
        F5Hz,
        F2100HzNoLpf,

        Invalid = 0xFF
    }

    [Flags]
    public enum Sensors : byte
    {
        None = 0,

        GyroX = 0x40,
        GyroY = 0x20,
        GyroZ = 0x10,

        Gyro = GyroX | GyroY | GyroZ,
        Accelerometer = 0x08,
        Compass = 0x01,

        Invalid = 0xFF
    }
}
