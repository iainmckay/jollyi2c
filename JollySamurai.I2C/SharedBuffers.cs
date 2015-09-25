using System;
using System.Collections;
using Microsoft.SPOT;

namespace JollySamurai.I2C
{
    internal class SharedBuffers
    {
        // FIXME: these buffers are never freed regardless of the frequency they are used
        // there are a few that are only used during initialization and could save some memory

        public static byte[] B1 = new byte[1];
        public static byte[] B2 = new byte[2];
        public static byte[] B4 = new byte[4];
        public static byte[] B8 = new byte[8];
        public static byte[] B9 = new byte[9];
        public static byte[] B10 = new byte[10];
        public static byte[] B12 = new byte[12];
    }
}
