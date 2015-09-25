using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

using XI2CDevice = Microsoft.SPOT.Hardware.I2CDevice;
using I2CTransaction = Microsoft.SPOT.Hardware.I2CDevice.I2CTransaction;
using I2CWriteTransaction = Microsoft.SPOT.Hardware.I2CDevice.I2CWriteTransaction;
using I2CReadTransaction = Microsoft.SPOT.Hardware.I2CDevice.I2CReadTransaction;

namespace JollySamurai.I2C
{
    public class I2CBus
    {
        #region Members
        private XI2CDevice _device;
        private XI2CDevice.Configuration _defaultConfiguration;

        private int _readTimeout;
        private int _writeTimeout;

        private byte[] _singleByteWriteBuffer = new byte[1];
        #endregion

        #region Constructor
        public I2CBus()
            : this(1000, 1000)
        {

        }

        public I2CBus(int readTimeout, int writeTimeout)
        {
            _readTimeout = readTimeout;
            _writeTimeout = writeTimeout;

            _defaultConfiguration = new XI2CDevice.Configuration(0, 0);
            _device = new XI2CDevice(_defaultConfiguration);
        }
        #endregion

        #region Methods
        public int ReadRegister(XI2CDevice.Configuration configuration, byte register, byte[] data, int length)
        {
            _device.Config = configuration;
            _singleByteWriteBuffer[0] = register;

            byte[] tmp = new byte[length];

            XI2CDevice.I2CWriteTransaction write = XI2CDevice.CreateWriteTransaction(_singleByteWriteBuffer);
            XI2CDevice.I2CReadTransaction read = XI2CDevice.CreateReadTransaction(tmp);

            int transferred = _device.Execute(new XI2CDevice.I2CTransaction[] { write, read }, _readTimeout);

            if (transferred == (length + 1)) {
                Array.Copy(tmp, data, transferred - 1);

                return (transferred - 1);
            }

            return -1;
        }

        public bool Write(XI2CDevice.Configuration configuration, byte[] data, int length)
        {
            _device.Config = configuration;

            byte[] tmp;

            if (data.Length == length) {
                tmp = data;
            } else {
                tmp = new byte[length];
                Array.Copy(data, 0, tmp, 0, length);
            }

            XI2CDevice.I2CWriteTransaction transaction = XI2CDevice.CreateWriteTransaction(tmp);
            int transferred = _device.Execute(new XI2CDevice.I2CTransaction[] { transaction }, _writeTimeout);

            return (transferred == data.Length);
        }

        public bool Write(XI2CDevice.Configuration configuration, byte register, byte[] data)
        {
            _device.Config = configuration;
            _singleByteWriteBuffer[0] = register;

            // TODO: investigate why we can't do two separate transactions and save on the alloc+copy
            byte[] tmp = new byte[data.Length + 1];
            tmp[0] = register;

            Array.Copy(data, 0, tmp, 1, data.Length);

            I2CTransaction[] transactions = new I2CTransaction[] {
                //XI2CDevice.CreateWriteTransaction(_singleByteWriteBuffer),
                XI2CDevice.CreateWriteTransaction(tmp)
            };

            int transferred = _device.Execute(transactions, _writeTimeout);

            return (transferred == tmp.Length);
        }

        public int ReadRegister(XI2CDevice.Configuration configuration, byte register, byte[] data)
        {
            _device.Config = configuration;
            _singleByteWriteBuffer[0] = register;

            I2CTransaction[] transactions = new I2CTransaction[] {
                XI2CDevice.CreateWriteTransaction(_singleByteWriteBuffer),
                XI2CDevice.CreateReadTransaction(data)
            };

            int transferred = _device.Execute(transactions, _readTimeout);

            if (transferred == (data.Length + 1)) {
                return (transferred - 1);
            }

            return -1;
        }
        #endregion
    }
}
