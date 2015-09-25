using System;
using Microsoft.SPOT;

namespace JollySamurai.I2C
{
    public struct Quaternion
    {
        #region Properties
        public float W
        {
            get
            {
                return _w;
            }
        }

        public float X
        {
            get
            {
                return _x;
            }
        }

        public float Y
        {
            get
            {
                return _y;
            }
        }

        public float Z
        {
            get
            {
                return _z;
            }
        }
        #endregion

        #region Members
        private float _w;
        private float _x;
        private float _y;
        private float _z;
        #endregion

        #region Methods
        public float LengthSquared()
        {
            return ((((this.X * this.X) + (this.Y * this.Y)) + (this.Z * this.Z)) + (this.W * this.W));
        }

        public void From(float w, float x, float y, float z)
        {
            _w = w;
            _x = x;
            _y = y;
            _z = z;
        }
        #endregion
    }
}
