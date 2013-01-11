using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

using XI2CDevice = Microsoft.SPOT.Hardware.I2CDevice;

namespace JollySamurai.I2C
{
	public class I2CBus
	{
		#region Members
		private XI2CDevice _device;
		private XI2CDevice.Configuration _defaultConfiguration;

		private int _readTimeout;
		private int _writeTimeout;
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
		public int Read(XI2CDevice.Configuration configuration, byte[] data)
		{
			_device.Config = configuration;

			XI2CDevice.I2CReadTransaction read = XI2CDevice.CreateReadTransaction(data);

			return _device.Execute(new XI2CDevice.I2CTransaction[] { read }, _readTimeout);
		}

		public int ReadRegister(XI2CDevice.Configuration configuration, byte register, byte[] data, int length)
		{
			_device.Config = configuration;

			byte[] tmp = new byte[length];
			
			XI2CDevice.I2CWriteTransaction write = XI2CDevice.CreateWriteTransaction(new byte[] { register });
			XI2CDevice.I2CReadTransaction read = XI2CDevice.CreateReadTransaction(tmp);

			int transferred = _device.Execute(new XI2CDevice.I2CTransaction[] { write, read }, _readTimeout);
			
			if(transferred == (length + 1))
			{
				Array.Copy(tmp, data, transferred - 1);

				return (transferred - 1);
			}

			return -1;
		}

		public bool Write(XI2CDevice.Configuration configuration, byte[] data)
		{
			_device.Config = configuration;

			XI2CDevice.I2CWriteTransaction transaction = XI2CDevice.CreateWriteTransaction(data);
			int transferred = _device.Execute(new XI2CDevice.I2CTransaction[] { transaction }, _writeTimeout);

			return (transferred == data.Length);
		}
		#endregion
	}
}