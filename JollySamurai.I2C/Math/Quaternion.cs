using System;
using Microsoft.SPOT;

namespace JollySamurai.I2C.Math
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

		public Quaternion Conjugate
		{
			get
			{
				return new Quaternion(_w, -_x, -_y, -_z);
			}
		}

		public float Magnitude
		{
			get
			{
				return (float) System.Math.Sqrt(_w * _w + _x * _x + _y * _y + _z * _z);
			}
		}

		public Quaternion Normalized
		{
			get
			{
				Quaternion r = new Quaternion(_w, _x, _y, _z);
				r.Normalize();

				return r;
			}
		}
		#endregion

		#region Members
		private float _w;
		private float _x;
		private float _y;
		private float _z;
		#endregion

		#region Constructor
		public Quaternion(float w, float x, float y, float z)
		{
			_w = w;
			_x = x;
			_y = y;
			_z = z;
		}
		#endregion

		#region Methods
		public Quaternion GetProduct(Quaternion q)
		{
			// Quaternion multiplication is defined by:
			//     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
			//     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
			//     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
			//     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
			return new Quaternion(
				(_w * q.W) - (_x * q.X) - (_y * q.Y) - (_z * q.Z),  // new w
				(_w * q.X) + (_x * q.W) + (_y * q.Z) - (_z * q.Y),  // new x
				(_w * q.Y) - (_x * q.Z) + (_y * q.W) + (_z * q.X),  // new y
				(_w * q.Z) + (_x * q.Y) - (_y * q.X) + (_z * q.W)); // new z
		}

		public void Normalize()
		{
			float m = this.Magnitude;

			_w /= m;
			_x /= m;
			_y /= m;
			_z /= m;
		}
		#endregion
	}
}
