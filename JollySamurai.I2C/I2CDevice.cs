using System;

using XI2CDevice = Microsoft.SPOT.Hardware.I2CDevice;

namespace JollySamurai.I2C
{
    public abstract class I2CDevice
    {
        #region Members
        private XI2CDevice.Configuration _config;

        private I2CBus _bus;
        private byte[] _singleByteReadBuffer = new byte[1];
        private byte[] _singleByteWriteBuffer = new byte[1];
        #endregion

        #region Constructor
        public I2CDevice(I2CBus bus, ushort address, int clockRate)
        {
            _bus = bus;
            _config = new XI2CDevice.Configuration(address, clockRate);
        }
        #endregion

        #region Methods
        /// <summary>
        /// Read multiple bytes from an 8-bit device register.
        /// </summary>
        /// <param name="register">First register address to read from.</param>
        /// <param name="buffer">Buffer to place the data in.</param>
        /// <param name="length">Number of bytes to read</param>
        /// <returns>Number of bytes read (-1 indicates failure).</returns>
        public int ReadBytes(byte register, byte[] buffer, int length)
        {
            int transferred = _bus.ReadRegister(_config, register, buffer, length);

            if (transferred != length) {
                return -1;
            }

            return transferred;
        }

        public bool WriteByte(byte register, byte data)
        {
            _singleByteWriteBuffer[0] = data;

            return WriteBytes(register, _singleByteWriteBuffer, 0, 1);
        }

        /// <summary>
        /// Write multiple bytes to an 8-bit device register.
        /// </summary>
        /// <param name="register">First register address to write to.</param>
        /// <param name="data">Data to write.</param>
        /// <param name="start">Where to start writing from data.</param>
        /// <param name="length">Number of bytes from data to write.</param>
        /// <returns>Status of operation (true = success).</returns>
        public bool WriteBytes(byte register, byte[] data, int start, int length)
        {
            byte[] tmp = new byte[length + 1];
            tmp[0] = register;

            Array.Copy(data, start, tmp, 1, length);

            return _bus.Write(_config, tmp, tmp.Length);
        }

        /// <summary>
        /// Write multiple bytes to an 8-bit device register.
        /// </summary>
        /// <param name="register">First register address to write to.</param>
        /// <param name="data">Data to write.</param>
        /// <returns>Status of operation (true = success).</returns>
        public bool WriteBytes(byte register, byte[] data)
        {
            return _bus.Write(_config, register, data);
        }

        /// <summary>
        /// Read multiple bytes from an 8-bit device register.
        /// </summary>
        /// <param name="register">First register address to read from.</param>
        /// <param name="buffer">Buffer to place the data in.</param>
        /// <returns>Number of bytes read (-1 indicates failure).</returns>
        public int ReadBytes(byte register, byte[] buffer)
        {
            int transferred = _bus.ReadRegister(_config, register, buffer);

            if (transferred != buffer.Length) {
                return -1;
            }

            return transferred;
        }
        #endregion
    }
}
