using System;

namespace JollySamurai.I2C.Exceptions
{
	public class UnexpectedDeviceException : Exception
	{
		#region Properties
		/// <summary>
		/// Gets the ID that was actually received.
		/// </summary>
		public byte ActualDeviceID
		{
			get
			{
				return _actualDeviceID;
			}
		}

		/// <summary>
		/// Gets the ID that was expected.
		/// </summary>
		public byte ExpectedDeviceID
		{
			get
			{
				return _expectedDeviceID;
			}
		}
		#endregion

		#region Members
		private byte _actualDeviceID;
		private byte _expectedDeviceID;		
		#endregion

		#region Constructor
		public UnexpectedDeviceException(byte expectedDeviceID, byte actualDeviceID)
			: base("Expected 0x" + expectedDeviceID.ToString("X") + " but got 0x" + actualDeviceID.ToString("X") + ".")
		{
			_actualDeviceID = actualDeviceID;
			_expectedDeviceID = expectedDeviceID;
		}
		#endregion
	}
}