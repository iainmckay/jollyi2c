using System;

using XI2CDevice = Microsoft.SPOT.Hardware.I2CDevice;

namespace JollySamurai.I2C
{
	public abstract class I2CDevice
	{
		#region Members
		private XI2CDevice.Configuration _config;

		private I2CBus _bus;
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
		/// Read a single bit from an 8-bit device register.
		/// </summary>
		/// <param name="register">Register to read from.</param>
		/// <param name="bit">Bit position to read (0-7).</param>
		/// <param name="data">Container for single bit value.</param>
		/// <returns>Status of read operation (true = success).</returns>
		protected bool ReadBit(byte register, byte bit, out byte data)
		{
			byte b;
			bool status = ReadByte(register, out b);

			data = (byte) (b & (1 << bit));

			return status;
		}

		/// <summary>
		/// Read multiple bits from an 8-bit device register.
		/// </summary>
		/// <param name="register">Register address to read from.</param>
		/// <param name="bitStart">First bit position to read (0-7).</param>
		/// <param name="length">Number of bits to read (not more than 8).</param>
		/// <param name="buffer">Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05).</param>
		/// <returns>Status of read operation (true = success).</returns>
		protected bool ReadBits(byte register, byte bitStart, byte length, out byte data)
		{
			bool status;
			byte b;

			data = 0;

			if((status = ReadByte(register, out b)) == true)
			{
				byte mask = (byte) (((1 << length) - 1) << (bitStart - length + 1));
				b &= mask;
				b >>= (bitStart - length + 1);

				data = b;
			}

			return status;
		}

		/// <summary>
		/// Read single byte from an 8-bit device register.
		/// </summary>
		/// <param name="register">Register address to read from.</param>
		/// <param name="data">Where to put the value.</param>
		/// <returns>Status of read operation (true = success).</returns>
		protected bool ReadByte(byte register, out byte data)
		{
			byte[] buffer = new byte[1];

			int count = ReadBytes(register, buffer);
			data = buffer[0];

			return (count == 1);
		}

		/// <summary>
		/// Read multiple bytes from an 8-bit device register.
		/// </summary>
		/// <param name="register">First register address to read from.</param>
		/// <param name="buffer">Buffer to place the data in.</param>
		/// <returns>Number of bytes read (-1 indicates failure).</returns>
		protected int ReadBytes(byte register, byte[] buffer)
		{
			return ReadBytes(register, buffer, buffer.Length);
		}

		/// <summary>
		/// Read multiple bytes from an 8-bit device register.
		/// </summary>
		/// <param name="register">First register address to read from.</param>
		/// <param name="buffer">Buffer to place the data in.</param>
		/// <param name="length">Number of bytes to read.</param>
		/// <returns>Number of bytes read (-1 indicates failure).</returns>
		protected int ReadBytes(byte register, byte[] buffer, int length)
		{
			int transferred = _bus.ReadRegister(_config, register, buffer, length);

			if(transferred != length)
			{
				return -1;
			}

			return transferred;
		}

		/// <summary>
		/// Write a single bit in an 8-bit device register.
		/// </summary>
		/// <param name="register">Register address to write to.</param>
		/// <param name="bit">Bit position to write (0-7).</param>
		/// <param name="data">New bit value to write.</param>
		/// <returns>Status of operation (true = success).</returns>
		protected bool WriteBit(byte register, byte bit, byte value)
		{
			byte b;

			ReadByte(register, out b);

			b = (byte) ((value != 0) ? (b | (1 << bit)) : (b & ~(1 << bit)));

			return WriteByte(register, b);
		}

		/// <summary>
		/// Write multiple bits in an 8-bit device register.
		/// </summary>
		/// <param name="register">Register address to write to</param>
		/// <param name="bitStart">First bit position to write (0-7).</param>
		/// <param name="length">Number of bits to write (not more than 8).</param>
		/// <param name="data">Right-aligned value to write.</param>
		/// <returns>Status of operation (true = success).</returns>
		protected bool WriteBits(byte register, byte bitStart, byte length, byte data)
		{
			byte b;

			if(ReadByte(register, out b) == true)
			{
				byte mask = (byte) (((1 << length) - 1) << (bitStart - length + 1));
				data <<= (bitStart - length + 1); // shift data into correct position
				data &= mask; // zero all non-important bits in data
				b &= (byte) ~(mask); // zero all important bits in existing byte
				b |= data; // combine data with existing byte

				return WriteByte(register, b);
			}

			return false;
		}

		/// <summary>
		/// Write single byte to an 8-bit device register.
		/// </summary>
		/// <param name="register">Register address to write to.</param>
		/// <param name="data">New byte value to write.</param>
		/// <returns>Status of operation (true = success).</returns>
		protected bool WriteByte(byte register, byte data)
		{
			return WriteBytes(register, new byte[] { data });
		}

		/// <summary>
		/// Write multiple bytes to an 8-bit device register.
		/// </summary>
		/// <param name="register">First register address to write to.</param>
		/// <param name="data">Data to write.</param>
		/// <returns>Status of operation (true = success).</returns>
		protected bool WriteBytes(byte register, byte[] data)
		{
			return WriteBytes(register, data, 0, data.Length);
		}

		/// <summary>
		/// Write multiple bytes to an 8-bit device register.
		/// </summary>
		/// <param name="register">First register address to write to.</param>
		/// <param name="data">Data to write.</param>
		/// <param name="start">Where to start writing from data.</param>
		/// <param name="length">Number of bytes from data to write.</param>
		/// <returns>Status of operation (true = success).</returns>
		protected bool WriteBytes(byte register, byte[] data, int start, int length)
		{
			byte[] tmp = new byte[length + 1];
			tmp[0] = register;

			Array.Copy(data, start, tmp, 1, length);

			return _bus.Write(_config, tmp);
		}

		/// <summary>
		/// Write a uint16 to the device register.
		/// </summary>
		/// <param name="register">Register address to write to.</param>
		/// <param name="data">Value to write.</param>
		/// <returns>Status of operation (true = success)</returns>
		public bool Write(byte register, ushort data)
		{
			return Write(register, new ushort[] { data });
		}

		/// <summary>
		/// Write multiple uint16's to the device register.
		/// </summary>
		/// <param name="register">First register address to write to.</param>
		/// <param name="data">Data to write.</param>
		/// <returns>Status of operation (true = success).</returns>
		public bool Write(byte register, ushort[] data)
		{
			byte[] byteData = new byte[data.Length * 2];

			for(int i = 0; i < data.Length; i++)
			{
				byteData[i * 2 + 0] = (byte) (data[i] >> 8);
				byteData[i * 2 + 1] = (byte) data[i];
			}

			return _bus.Write(_config, byteData);
		}
		#endregion
	}
}