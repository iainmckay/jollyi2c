using System;
using System.Threading;
using Microsoft.SPOT;

namespace JollySamurai.I2C.Devices.InvenSense
{
    public abstract class BaseUnit : I2CDevice
    {
        #region Constants
        public const int MaxFifoCount = 1024;

        private const byte Bit_Reset = 0x80;
        private const byte Bit_Sleep = 0x40;

        private const byte Bit_Fifo_Enable = 0x40;
        private const byte Bit_Fifo_Reset = 0x04;
        private const byte Bit_Fifo_Overflow = 0x10;

        private const byte Bit_DigitalMotionProcessor_Enable = 0x80;
        private const byte Bit_DigitalMotionProcessor_InterruptEnable = 0x02;
        private const byte Bit_DigitalMotionProcessor_Reset = 0x08;

        private const byte Bit_Standby_AccelerometerX = 0x20;
        private const byte Bit_Standby_AccelerometerY = 0x10;
        private const byte Bit_Standby_AccelerometerZ = 0x08;
        private const byte Bit_Standby_GyroX = 0x04;
        private const byte Bit_Standby_GyroY = 0x02;
        private const byte Bit_Standby_GyroZ = 0x01;
        private const byte Bit_Standby_Accelerometer = Bit_Standby_AccelerometerX | Bit_Standby_AccelerometerY | Bit_Standby_AccelerometerZ;
        private const byte Bit_Standby_Gyro = Bit_Standby_GyroX | Bit_Standby_GyroY | Bit_Standby_GyroZ;

        private const byte Bit_LowInterrupt_Active = 0x80;
        private const byte Bit_LatchedInterrupt_Enable = 0x20;
        private const byte Bit_Any_Read_Clear = 0x10;
        private const byte Bit_Bypass_Enable = 0x02;
        private const byte Bit_DataReady_Enable = 0x01;
        private static readonly byte Bit_AuxiliaryIf_Enable = 0x20;
        #endregion

        #region Properties
        public DigitalMotionProcessor DigitalMotionProcessor
        {
            get
            {
                return _dmp;
            }
        }

        public Sensors Sensors
        {
            get
            {
                return _sensors;
            }
            set
            {
                byte[] buffer1 = SharedBuffers.B1;
                buffer1[0] = Bit_Sleep;

                if ((value & Sensors.Gyro) != 0) {
                    buffer1[0] = (byte) ClockSource.Pll;
                } else if (value != Sensors.None) {
                    buffer1[0] = 0;
                }

                if (WriteBytes(Register.PowerManagement1, buffer1) == false) {
                    _sensors = Sensors.None;
                } else {
                    _clockSource = (ClockSource) (buffer1[0] & ~Bit_Sleep);
                    buffer1[0] = 0;

                    if ((value & Sensors.GyroX) == 0) {
                        buffer1[0] |= Bit_Standby_GyroX;
                    }

                    if ((value & Sensors.GyroY) == 0) {
                        buffer1[0] |= Bit_Standby_GyroY;
                    }

                    if ((value & Sensors.GyroZ) == 0) {
                        buffer1[0] |= Bit_Standby_GyroZ;
                    }

                    if ((value & Sensors.Accelerometer) == 0) {
                        buffer1[0] |= Bit_Standby_Accelerometer;
                    }

                    if (WriteBytes(Register.PowerManagement2, buffer1) == false) {
                        _sensors = Sensors.None;
                    } else {
                        if ((value != Sensors.None) && (value != Sensors.Accelerometer)) {
                            // latched interrupts only used in LP accel mode.
                            this.UseLatchedInterrupts = false;
                        }

                        _sensors = value;
                        _lowPowerAccelerometerModeEnabled = false;

                        Thread.Sleep(50);
                    }
                }
            }
        }

        public bool UseLatchedInterrupts
        {
            get
            {
                return _useLatchedInterrupts;
            }
            set
            {
                if (_useLatchedInterrupts == value) {
                    return;
                }

                byte[] buffer1 = SharedBuffers.B1;
                buffer1[0] = 0;

                if (value == true) {
                    buffer1[0] = Bit_LatchedInterrupt_Enable | Bit_Any_Read_Clear;
                }

                if (_bypassModeEnabled == true) {
                    buffer1[0] |= Bit_Bypass_Enable;
                }

                if (_lowInterruptsActive == true) {
                    buffer1[0] |= Bit_LowInterrupt_Active;
                }

                if (WriteBytes(Register.InterruptPinConfig, buffer1) == false) {
                    // FIXME:
                    throw new Exception();
                }

                _useLatchedInterrupts = value;
            }
        }

        public bool InterruptEnabled
        {
            get
            {
                return _interruptEnabled;
            }
            set
            {
                byte[] buffer1 = SharedBuffers.B1;

                if (_dmp.IsEnabled == true) {
                    if (value == true) {
                        buffer1[0] = Bit_DigitalMotionProcessor_InterruptEnable;
                    } else {
                        buffer1[0] = 0x00;
                    }

                    if (WriteBytes(Register.EnableInterrupt, buffer1) == false) {
                        // FIXME:
                        throw new Exception();
                    }

                    _interruptEnabled = value;
                } else {
                    if (_sensors == Sensors.None) {
                        return;
                    }

                    if ((value == true) && (_interruptEnabled == true)) {
                        return;
                    }

                    if (value == true) {
                        buffer1[0] = Bit_DataReady_Enable;
                    } else {
                        buffer1[0] = 0;
                    }

                    if (WriteBytes(Register.EnableInterrupt, buffer1) == false) {
                        // FIXME:
                        throw new Exception();
                    }

                    _interruptEnabled = value;
                }
            }
        }

        public ushort SampleRate
        {
            get
            {
                return _sampleRate;
            }
            set
            {
                if ((_sensors == Sensors.None) ||
                    (_dmp.IsEnabled == true)) {
                    return;
                }

                if (_lowPowerAccelerometerModeEnabled == true) {
                    if ((value > 0) && (value <= 40)) {
                        // just stay in low-power accel mode.
                        SetLowPowerAccelerometerMode(value);
                        return;
                    }

                    // requested rate exceeds the allowed frequencies in LP accel mode, switch back to full-power mode.
                    SetLowPowerAccelerometerMode(0);
                }

                if (value < 4) {
                    value = 4;
                } else if (value > 1000) {
                    value = 1000;
                }

                byte[] buffer1 = SharedBuffers.B1;
                buffer1[0] = (byte) (1000 / value - 1);

                if (WriteBytes(Register.RateDivisor, buffer1) == false) {
                    // FIXME:
                    throw new Exception();
                }

                _sampleRate = (ushort) (1000 / (1 + buffer1[0]));

                // automatically set LPF to 1/2 sampling rate.
                this.DigitalLowPassFilterBandwidth = FilterFromSampleRate(_sampleRate >> 1);
            }
        }


        public GyroRange GyroFullScaleRange
        {
            get
            {
                return _gyroFullScaleRange;
            }
            set
            {
                if ((_sensors == Sensors.None)
                    || (_gyroFullScaleRange == value)) {
                    return;
                }

                byte[] buffer1 = SharedBuffers.B1;
                buffer1[0] = (byte) ((byte) value << 3);

                if (WriteBytes(Register.GyroConfig, buffer1) == false) {
                    // FIXME:
                    throw new Exception();
                }

                _gyroFullScaleRange = value;
            }
        }

        public AccelerometerRange AccelerometerFullScaleRange
        {
            get
            {
                return _accelerometerFullScaleRange;
            }
            set
            {
                if ((_sensors == Sensors.None)
                    || (_accelerometerFullScaleRange == value)) {
                    return;
                }

                byte[] buffer1 = SharedBuffers.B1;
                buffer1[0] = (byte) ((byte) value << 3);

                if (WriteBytes(Register.AccelerometerConfig, buffer1) == false) {
                    // FIXME:
                    throw new Exception();
                }

                _accelerometerFullScaleRange = value;
            }
        }

        public DigitalLowPassFilterBandwidth DigitalLowPassFilterBandwidth
        {
            get
            {
                return _digitalLowPassFilterBandwidth;
            }
            set
            {
                if ((_sensors == Sensors.None)
                    || (_digitalLowPassFilterBandwidth == value)) {
                    return;
                }

                byte[] buffer1 = SharedBuffers.B1;
                buffer1[0] = (byte) value;

                if (WriteBytes(Register.DigitalLowPassFilterBandwidth, buffer1) == false) {
                    // FIXME:
                    throw new Exception();
                }

                _digitalLowPassFilterBandwidth = value;
            }
        }        
        #endregion

        #region Fields
        private DigitalMotionProcessor _dmp;

        // shared buffer for common small read/writes. 12 bytes is the largest so far
        private byte[] _sharedBuffer = new byte[12];

        // state
        private Sensors _sensors = Sensors.Invalid;
        private GyroRange _gyroFullScaleRange = GyroRange.Invalid;
        private AccelerometerRange _accelerometerFullScaleRange = AccelerometerRange.Invalid;
        private DigitalLowPassFilterBandwidth _digitalLowPassFilterBandwidth = DigitalLowPassFilterBandwidth.Invalid;

        /* Matches fifo_en register. */
        private Sensors _fifoEnabledSensorsMask = Sensors.Invalid;

        // enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2
        private ushort _sampleRate = 0xFFFF;
        private bool _lowPowerAccelerometerModeEnabled;
        private ClockSource _clockSource = ClockSource.Pll;
        private bool _useLatchedInterrupts;
        private bool _lowInterruptsActive = true;

        // true if devices on auxiliary I2C bus appear on the primary
        private bool _bypassModeEnabled;
        private bool _interruptEnabled;
        #endregion

        #region Constructor
        public BaseUnit(I2CBus bus, ushort address, int clockRate)
            : base(bus, address, clockRate)
        {
            _dmp = new DigitalMotionProcessor(this);
        }
        #endregion

        #region Methods
        public bool Initialize()
        {
            // reset device
            byte[] buffer1 = SharedBuffers.B1;
            buffer1[0] = Bit_Reset;

            if (WriteBytes(Register.PowerManagement1, buffer1) == false) {
                return false;
            }

            Thread.Sleep(100);

            // wake up chip
            buffer1[0] = 0x00;

            if (WriteBytes(Register.PowerManagement1, buffer1) == false) {
                return false;
            }

            this.GyroFullScaleRange = GyroRange.R2000;
            this.AccelerometerFullScaleRange = AccelerometerRange.R2G;
            this.DigitalLowPassFilterBandwidth = DigitalLowPassFilterBandwidth.F42Hz;
            this.SampleRate = 50;

            ConfigureFifo(Sensors.None);
            
            // already disabled by setup_compass
            if (SetBypass(false) != 0) {
                return false;
            }

            this.Sensors = Sensors.None;

            return true;
        }

        public int ConfigureFifo(Sensors sensors)
        {
            Sensors prev;
            int result = 0;

            if (_dmp.IsEnabled == true) {
                return 0;
            } else {
                if (_sensors == Sensors.None) {
                    return -1;
                }

                prev = _fifoEnabledSensorsMask;
                _fifoEnabledSensorsMask = sensors & _sensors;

                if (_fifoEnabledSensorsMask != sensors) {
                    // you're not getting what you asked for. Some sensors are asleep.
                    result = -1;
                } else {
                    result = 0;
                }

                if ((sensors != Sensors.None) || (_lowPowerAccelerometerModeEnabled == true)) {
                    this.InterruptEnabled = true;
                } else {
                    this.InterruptEnabled = false;
                }

                if (sensors != Sensors.None) {
                    if (ResetFifo() != 0) {
                        _fifoEnabledSensorsMask = prev;
                        return -1;
                    }
                }
            }

            return result;
        }

        public int ReadFifoStream(byte[] data, ref int more)
        {
            if (_dmp.IsEnabled == false) {
                return -1;
            }

            if (_sensors == Sensors.None) {
                return -1;
            }

            byte[] buffer1 = SharedBuffers.B1;
            byte[] buffer2 = SharedBuffers.B2;
            int length = data.Length;

            if (ReadBytes(Register.FifoCount, buffer2) != 2) {
                return -1;
            }

            ushort fifoCount = (ushort) ((buffer2[0] << 8) | buffer2[1]);

            if (fifoCount < length) {
                more = 0;
                return -1;
            }

            if (fifoCount > (MaxFifoCount >> 1)) {
                // FIFO is 50% full, better check overflow bit
                if (ReadBytes(Register.InterruptStatus, buffer1) != 1) {
                    return -1;
                }

                if ((buffer1[0] & Bit_Fifo_Overflow) == Bit_Fifo_Overflow) {
                    ResetFifo();
                    return -2;
                }
            }

            if (ReadBytes(Register.FifoReadWrite, data) != length) {
                return -1;
            }

            more = (fifoCount / length - 1);

            return 0;
        }

        public int RestoreFifo()
        {
            byte[] buffer1 = SharedBuffers.B1;
            buffer1[0] = (byte) _fifoEnabledSensorsMask;

            if (WriteBytes(0x23, buffer1, 0, 1) == false) {
                return -1;
            }

            return 0;
        }

        public int ResetFifo()
        {
            if (_sensors == Sensors.None) {
                return -1;
            }

            byte[] buffer1 = SharedBuffers.B1;
            buffer1[0] = 0;

            if (WriteBytes(Register.EnableInterrupt, buffer1) == false) {
                return -1;
            }

            if (WriteBytes(Register.EnableFifo, buffer1) == false) {
                return -1;
            }

            if (WriteBytes(Register.UserControl, buffer1) == false) {
                return -1;
            }

            if (_dmp.IsEnabled == true) {
                buffer1[0] = Bit_Fifo_Reset | Bit_DigitalMotionProcessor_Reset;

                if (WriteBytes(Register.UserControl, buffer1) == false) {
                    return -1;
                }

                Thread.Sleep(100);

                buffer1[0] = Bit_Fifo_Enable | Bit_DigitalMotionProcessor_Enable;

                if ((_sensors & Sensors.Compass) == Sensors.Compass) {
                    buffer1[0] |= Bit_AuxiliaryIf_Enable;
                }

                if (WriteBytes(Register.UserControl, buffer1) == false) {
                    return -1;
                }

                if (_interruptEnabled == true) {
                    buffer1[0] = Bit_DigitalMotionProcessor_InterruptEnable;
                } else {
                    buffer1[0] = 0;
                }

                if (WriteBytes(Register.EnableInterrupt, buffer1) == false) {
                    return -1;
                }

                buffer1[0] = 0;

                if (WriteBytes(Register.EnableFifo, buffer1) == false) {
                    return -1;
                }
            } else {
                buffer1[0] = Bit_Fifo_Reset;

                if (WriteBytes(Register.UserControl, buffer1) == false) {
                    return -1;
                }

                if ((_bypassModeEnabled == true) || ((_sensors & Sensors.Compass) != Sensors.Compass)) {
                    buffer1[0] = Bit_Fifo_Enable;
                } else {
                    buffer1[0] = (byte) (Bit_Fifo_Enable | Bit_AuxiliaryIf_Enable);
                }

                if (WriteBytes(Register.UserControl, buffer1) == false) {
                    return -1;
                }

                Thread.Sleep(100);

                if (_interruptEnabled == true) {
                    buffer1[0] = Bit_DataReady_Enable;
                } else {
                    buffer1[0] = 0;
                }

                if (WriteBytes(Register.EnableInterrupt, buffer1) == false) {
                    return -1;
                }

                buffer1[0] = (byte) _fifoEnabledSensorsMask;

                if (WriteBytes(Register.EnableFifo, buffer1) == false) {
                    return -1;
                }
            }

            return 0;
        }

        public int SetBypass(bool enable)
        {
            if (_bypassModeEnabled == enable) {
                return 0;
            }

            byte[] buffer1 = SharedBuffers.B1;

            if (enable == true) {
                if (ReadBytes(Register.UserControl, buffer1) != 1) {
                    return -1;
                }

                buffer1[0] &= (byte) ~Bit_AuxiliaryIf_Enable;

                if (WriteBytes(Register.UserControl, buffer1) == false) {
                    return -1;
                }

                Thread.Sleep(3);

                buffer1[0] = Bit_Bypass_Enable;

                if (_lowInterruptsActive == true) {
                    buffer1[0] |= Bit_LowInterrupt_Active;
                }

                if (_useLatchedInterrupts == true) {
                    buffer1[0] |= Bit_LatchedInterrupt_Enable | Bit_Any_Read_Clear;
                }

                if (WriteBytes(Register.InterruptPinConfig, buffer1) == false) {
                    return -1;
                }
            } else {
                // enable I2C master mode if compass is being used
                if (ReadBytes(Register.UserControl, buffer1) != 1) {
                    return -1;
                }

                if ((_sensors & Sensors.Compass) == Sensors.Compass) {
                    buffer1[0] |= Bit_AuxiliaryIf_Enable;
                } else {
                    buffer1[0] &= (byte) ~Bit_AuxiliaryIf_Enable;
                }

                if (WriteBytes(Register.UserControl, buffer1) == false) {
                    return -1;
                }

                Thread.Sleep(3);

                if (_lowInterruptsActive == true) {
                    buffer1[0] = Bit_LowInterrupt_Active;
                } else {
                    buffer1[0] = 0;
                }

                if (_useLatchedInterrupts == true) {
                    buffer1[0] |= Bit_LatchedInterrupt_Enable | Bit_Any_Read_Clear;
                }

                if (WriteBytes(Register.InterruptPinConfig, buffer1) == false) {
                    return -1;
                }
            }

            _bypassModeEnabled = enable;

            return 0;
        }

        private int SetLowPowerAccelerometerMode(ushort rate)
        {
            Debug.Print("TODO: mpu_lp_accel_mode");

            return 0;
        }

        /// <summary>
        /// Write multiple bytes to an 8-bit device register.
        /// </summary>
        /// <param name="register">First register address to write to.</param>
        /// <param name="data">Data to write.</param>
        /// <returns>Status of operation (true = success).</returns>
        public bool WriteBytes(Register register, byte[] data)
        {
            return WriteBytes((byte) register, data);
        }

        /// <summary>
        /// Read multiple bytes from an 8-bit device register.
        /// </summary>
        /// <param name="register">First register address to read from.</param>
        /// <param name="buffer">Buffer to place the data in.</param>
        /// <returns>Number of bytes read (-1 indicates failure).</returns>
        public int ReadBytes(Register register, byte[] buffer)
        {
            return ReadBytes((byte) register, buffer);
        }

        private DigitalLowPassFilterBandwidth FilterFromSampleRate(int rate)
        {
            if (rate >= 188) {
                return DigitalLowPassFilterBandwidth.F188Hz;
            } else if (rate >= 98) {
                return DigitalLowPassFilterBandwidth.F98Hz;
            } else if (rate >= 42) {
                return DigitalLowPassFilterBandwidth.F42Hz;
            } else if (rate >= 20) {
                return DigitalLowPassFilterBandwidth.F20Hz;
            } else if (rate >= 10) {
                return DigitalLowPassFilterBandwidth.F10Hz;
            }

            return DigitalLowPassFilterBandwidth.F5Hz;
        }
        #endregion
    }
}
