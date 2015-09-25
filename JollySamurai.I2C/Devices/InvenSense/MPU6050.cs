using System;
using Microsoft.SPOT;

namespace JollySamurai.I2C.Devices.InvenSense
{
    public class MPU6050 : BaseUnit
    {

        #region Constructor
        public MPU6050(I2CBus bus, ushort address, int clockRate)
            : base(bus, address, clockRate)
        {
        }
        #endregion
    }
}
