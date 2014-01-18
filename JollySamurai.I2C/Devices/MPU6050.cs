using System;
using System.Threading;
using Microsoft.SPOT;

using JollySamurai.I2C.Exceptions;
using JollySamurai.I2C.Math;

namespace JollySamurai.I2C.Devices
{
    public class MPU6050 : I2CDevice
    {
        #region Constants
        public const ushort AddressLow = 0x68; // address pin low (GND), default for InvenSense evaluation board
        public const ushort AddressHigh = 0x69; // address pin high (VCC)
        public const ushort AddressDefault = AddressLow;

        private const byte ExpectedDeviceID = 0x34;


        private const byte RegisterAddress_AccelerationXOut = 0x3B;
        private const byte RegisterAddress_AccelerometerConfig = 0x1C;
        private const byte RegisterAddress_Config = 0x1A;
        private const byte RegisterAddress_DMPConfig1 = 0x70;
        private const byte RegisterAddress_DMPConfig2 = 0x71;
        private const byte RegisterAddress_FIFO_Count = 0x72;
        private const byte RegisterAddress_GyroConfig = 0x1B;
        private const byte RegisterAddress_Motion_Duration = 0x20;
        private const byte RegisterAddress_Motion_Threshold = 0x1F;
        private const byte RegisterAddress_PowerManagement1 = 0x6B;
        private const byte RegisterAddress_SampleRateDivider = 0x19;
        private const byte RegisterAddress_WhoAmI = 0x75;
        private const byte RegisterAddress_ZeroMotion_Duration = 0x22;
        private const byte RegisterAddress_ZeroMotion_Threshold = 0x21;

        private const byte RegisterAddress_GyroConfig_OffsetX = 0x00; // [7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
        private const byte RegisterAddress_GyroConfig_OffsetY = 0x01; // [7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
        private const byte RegisterAddress_GyroConfig_OffsetZ = 0x02; // [7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD

        private const byte RegisterAddress_GyroConfig_UserOffsetX = 0x13; // [15:0] XG_OFFS_USR
        private const byte RegisterAddress_GyroConfig_UserOffsetY = 0x15; // [15:0] YG_OFFS_USR
        private const byte RegisterAddress_GyroConfig_UserOffsetZ = 0x17; // [15:0] ZG_OFFS_USR

        private const byte RegisterAddress_Interrupt_Enable = 0x38;
        private const byte RegisterAddress_Interrupt_Status = 0x3A;

        private const byte RegisterAddress_Memory_BankSelect = 0x6D;
        private const byte RegisterAddress_Memory_ReadWrite = 0x6F;
        private const byte RegisterAddress_Memory_StartAddress = 0x6E;

        private const byte RegisterAddress_UserControl = 0x6A;

        private const byte RegisterAddress_Slave0_Address = 0x25;
        private const byte RegisterAddress_Slave0_Register = 0x26;
        private const byte RegisterAddress_Slave0_Control = 0x27;

        private const byte Bit_Power1_ClockSelect = 2;
        private const byte Bit_Power1_ClockSelectLength = 3;
        private const byte Bit_Power1_CycleBit = 5;
        private const byte Bit_Power1_Sleep = 6;
        private const byte Bit_Power1_Reset = 7;

        private const byte Bit_AccelerometerConfig_FullScaleRange = 4;
        private const byte Bit_AccelerometerConfig_FullScaleRangeLength = 2;

        private const byte Bit_Config_DigitalLowPassFilter_Mode = 2;
        private const byte Bit_Config_DigitalLowPassFilter_ModeLength = 3;
        private const byte Bit_Config_ExternalFrameSync = 5;
        private const byte Bit_Config_ExternalFrameSyncLength = 3;

        private const byte Bit_GyroConfig_FullScaleRange = 4;
        private const byte Bit_GyroConfig_FullScaleRangeLength = 2;
        private const byte Bit_GyroConfig_Offset = 6;
        private const byte Bit_GyroConfig_OffsetLength = 6;

        private const byte Bit_WhoAmI = 6;
        private const byte Bit_WhoAmILength = 6;

        private const byte Bit_UserControl_DMP_Enabled = 7;
        private const byte Bit_UserControl_DMP_Reset = 3;
        private const byte Bit_UserControl_FIFO_Enabled = 6;
        private const byte Bit_UserControl_FIFO_Reset = 2;
        private const byte Bit_UserControl_I2C_MasterEnabled = 5;
        private const byte Bit_UserControl_I2C_MasterReset = 1;

        private const byte Bit_Undocumented_OTPBankValid = 0;

        private const byte DMPMemoryBankCount = 8;
        private const byte DMPMemoryBankSize = 255;
        private const byte DMPMemoryChunkSize = 16;
        #endregion

        #region DMP Programs
        /// <summary>
        /// This block of memory gets written to the MPU on start-up, and it seems
        /// to be volatile memory, so it has to be done each time (it only takes ~1
        /// second though).
        /// </summary>
        private static readonly byte[] DMPProgram = new byte[] {
			// bank 0, 256 bytes
			0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
			0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
			0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
			0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
			0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
			0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
			0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
			0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
			0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
			0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
			0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,

			// bank 1, 256 bytes
			0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
			0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
			0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
			0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
			0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,

			// bank 2, 256 bytes
			0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
			0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

			// bank 3, 256 bytes
			0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F,
			0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2,
			0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF,
			0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C,
			0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1,
			0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01,
			0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80,
			0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C,
			0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80,
			0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E,
			0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9,
			0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24,
			0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0,
			0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86,
			0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
			0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,

			// bank 4, 256 bytes
			0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
			0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
			0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
			0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
			0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
			0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
			0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
			0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
			0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
			0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
			0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
			0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
			0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
			0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
			0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
			0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,

			// bank 5, 256 bytes
			0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
			0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
			0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
			0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
			0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
			0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
			0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
			0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
			0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A,
			0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60,
			0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97,
			0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04,
			0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78,
			0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79,
			0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68,
			0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68,

			// bank 6, 256 bytes
			0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04,
			0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66,
			0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31,
			0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60,
			0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76,
			0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56,
			0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD,
			0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91,
			0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8,
			0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE,
			0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9,
			0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD,
			0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E,
			0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8,
			0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89,
			0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79,

			// bank 7, 138 bytes (remainder)
			0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8,
			0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA,
			0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB,
			0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3,
			0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3,
			0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
			0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3, 0xA3,
			0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC,
			0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF
		};

        // thanks to Noah Zerkin for piecing this stuff together!
        private static readonly byte[] DMPConfigProgram = new byte[] {
			//  BANK    OFFSET  LENGTH  [DATA]
			0x03,   0x7B,   0x03,   0x4C, 0xCD, 0x6C,					// FCFG_1 inv_set_gyro_calibration
			0x03,   0xAB,   0x03,   0x36, 0x56, 0x76,					// FCFG_3 inv_set_gyro_calibration
			0x00,   0x68,   0x04,   0x02, 0xCB, 0x47, 0xA2,				// D_0_104 inv_set_gyro_calibration
			0x02,   0x18,   0x04,   0x00, 0x05, 0x8B, 0xC1,				// D_0_24 inv_set_gyro_calibration
			0x01,   0x0C,   0x04,   0x00, 0x00, 0x00, 0x00,				// D_1_152 inv_set_accel_calibration
			0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_accel_calibration
			0x03,   0x89,   0x03,   0x26, 0x46, 0x66,					// FCFG_7 inv_set_accel_calibration
			0x00,   0x6C,   0x02,   0x20, 0x00,							// D_0_108 inv_set_accel_calibration
			0x02,   0x40,   0x04,   0x00, 0x00, 0x00, 0x00,				// CPASS_MTX_00 inv_set_compass_calibration
			0x02,   0x44,   0x04,   0x00, 0x00, 0x00, 0x00,				// CPASS_MTX_01
			0x02,   0x48,   0x04,   0x00, 0x00, 0x00, 0x00,				// CPASS_MTX_02
			0x02,   0x4C,   0x04,   0x00, 0x00, 0x00, 0x00,				// CPASS_MTX_10
			0x02,   0x50,   0x04,   0x00, 0x00, 0x00, 0x00,				// CPASS_MTX_11
			0x02,   0x54,   0x04,   0x00, 0x00, 0x00, 0x00,				// CPASS_MTX_12
			0x02,   0x58,   0x04,   0x00, 0x00, 0x00, 0x00,				// CPASS_MTX_20
			0x02,   0x5C,   0x04,   0x00, 0x00, 0x00, 0x00,				// CPASS_MTX_21
			0x02,   0xBC,   0x04,   0x00, 0x00, 0x00, 0x00,				// CPASS_MTX_22
			0x01,   0xEC,   0x04,   0x00, 0x00, 0x40, 0x00,				// D_1_236 inv_apply_endian_accel
			0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_mpu_sensors
			0x04,   0x02,   0x03,   0x0D, 0x35, 0x5D,					// CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
			0x04,   0x09,   0x04,   0x87, 0x2D, 0x35, 0x3D,				// FCFG_5 inv_set_bias_update
			0x00,   0xA3,   0x01,   0x00,								// D_0_163 inv_set_dead_zone
			
			// SPECIAL 0x01 = enable interrupts
			0x00,   0x00,   0x00,   0x01,								// SET INT_ENABLE at i=22, SPECIAL INSTRUCTION
			0x07,   0x86,   0x01,   0xFE,								// CFG_6 inv_set_fifo_interupt
			0x07,   0x41,   0x05,   0xF1, 0x20, 0x28, 0x30, 0x38,		// CFG_8 inv_send_quaternion
			0x07,   0x7E,   0x01,   0x30,								// CFG_16 inv_set_footer
			0x07,   0x46,   0x01,   0x9A,								// CFG_GYRO_SOURCE inv_send_gyro
			0x07,   0x47,   0x04,   0xF1, 0x28, 0x30, 0x38,				// CFG_9 inv_send_gyro -> inv_construct3_fifo
			0x07,   0x6C,   0x04,   0xF1, 0x28, 0x30, 0x38,				// CFG_12 inv_send_accel -> inv_construct3_fifo
			0x02,   0x16,   0x02,   0x00, 0x01							// D_0_22 inv_set_fifo_rate

			// This very last 0x01 WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
			// 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
			// DMP output frequency is calculated easily using this equation: (200Hz / (1 + value))

			// It is important to make sure the host processor can keep up with reading and processing
			// the FIFO output at the desired rate. Handling FIFO overflow cleanly is also a good idea.
		};

        private static readonly byte[] DMPUpdateProgram = new byte[] {
			0x01,   0xB2,   0x02,   0xFF, 0xFF,
			0x01,   0x90,   0x04,   0x09, 0x23, 0xA1, 0x35,
			0x01,   0x6A,   0x02,   0x06, 0x00,
			0x01,   0x60,   0x08,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00,   0x60,   0x04,   0x40, 0x00, 0x00, 0x00,
			0x01,   0x62,   0x02,   0x00, 0x00,
			0x00,   0x60,   0x04,   0x00, 0x40, 0x00, 0x00
		};
        #endregion

        #region Properties
        #region Accelerometer
        /// <summary>
        /// Get/set full-scale accelerometer range.
        /// </summary>
        public AccelerometerRange AccelerometerFullScaleRange
        {
            get
            {
                byte b;
                ReadBits(RegisterAddress_AccelerometerConfig, Bit_AccelerometerConfig_FullScaleRange, Bit_AccelerometerConfig_FullScaleRangeLength, out b);

                return (AccelerometerRange) b;
            }
            set
            {
                WriteBits(RegisterAddress_AccelerometerConfig, Bit_AccelerometerConfig_FullScaleRange, Bit_AccelerometerConfig_FullScaleRangeLength, (byte) value);
            }
        }
        #endregion

        #region Config
        /// <summary>
        /// Get/set digital low-pass filter configuration.
        /// </summary>
        /// <remarks>
        /// The DLPF_CFG parameter sets the digital low pass filter configuration. It
        /// also detrmines the internal sampling rate used by the device as shown in
        /// the table below.
        /// <para/>
        /// Note: The accelerometer output rate is 1kHz. This means that for a Sample
        /// Rate greater than 1kHz, the same accelerometer sample may be output to the
        /// FIFO, DMP, and sensor registers more than once.
        /// <para/>
        ///          |   ACCELEROMETER    |           GYROSCOPE
        /// DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
        /// ---------+-----------+--------+-----------+--------+-------------
        /// 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
        /// 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
        /// 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
        /// 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
        /// 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
        /// 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
        /// 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
        /// 7        |   -- Reserved --   |   -- Reserved --   | Reserved
        /// </summary>
        public DigitalLowPassFilterBandwidth DigitalLowPassFilterBandwidth
        {
            get
            {
                byte b;
                ReadBits(RegisterAddress_Config, Bit_Config_DigitalLowPassFilter_Mode, Bit_Config_DigitalLowPassFilter_ModeLength, out b);

                return (DigitalLowPassFilterBandwidth) b;
            }
            set
            {
                WriteBits(RegisterAddress_Config, Bit_Config_DigitalLowPassFilter_Mode, Bit_Config_DigitalLowPassFilter_ModeLength, (byte) value);
            }
        }

        /// <summary>
        /// Get/set external FSYNC configuration.
        /// </summary>
        /// <remarks>
        /// Configures the external Frame Synchronization (FSYNC) pin sampling. An
        /// external signal connected to the FSYNC pin can be sampled by configuring
        /// EXT_SYNC_SET. Signal changes to the FSYNC pin are latched so that short
        /// strobes may be captured. The latched FSYNC signal will be sampled at the
        /// Sampling Rate, as defined in register 25. After sampling, the latch will
        /// reset to the current FSYNC signal state.
        /// <para/>
        /// The sampled value will be reported in place of the least significant bit in
        /// a sensor data register determined by the value of EXT_SYNC_SET according to
        /// the following table.
        /// <para/>
        /// EXT_SYNC_SET | FSYNC Bit Location
        /// -------------+-------------------
        /// 0            | Input disabled
        /// 1            | TEMP_OUT_L[0]
        /// 2            | GYRO_XOUT_L[0]
        /// 3            | GYRO_YOUT_L[0]
        /// 4            | GYRO_ZOUT_L[0]
        /// 5            | ACCEL_XOUT_L[0]
        /// 6            | ACCEL_YOUT_L[0]
        /// 7            | ACCEL_ZOUT_L[0]
        /// </remarks>
        public ExternalFrameSyncSource ExternalFrameSync
        {
            get
            {
                byte b;
                ReadBits(RegisterAddress_Config, Bit_Config_ExternalFrameSync, Bit_Config_ExternalFrameSyncLength, out b);

                return (ExternalFrameSyncSource) b;
            }
            set
            {
                WriteBits(RegisterAddress_Config, Bit_Config_ExternalFrameSync, Bit_Config_ExternalFrameSyncLength, (byte) value);
            }
        }
        #endregion

        #region DMP
        public byte DMPConfig1
        {
            get
            {
                byte b;
                ReadByte(RegisterAddress_DMPConfig1, out b);

                return b;
            }
            set
            {
                WriteByte(RegisterAddress_DMPConfig1, value);
            }
        }

        public byte DMPConfig2
        {
            get
            {
                byte b;
                ReadByte(RegisterAddress_DMPConfig2, out b);

                return b;
            }
            set
            {
                WriteByte(RegisterAddress_DMPConfig2, value);
            }
        }

        public bool DMPEnabled
        {
            get
            {
                byte b;
                ReadBit(RegisterAddress_UserControl, Bit_UserControl_DMP_Enabled, out b);

                return (b == 1);
            }
            set
            {
                WriteBit(RegisterAddress_UserControl, Bit_UserControl_DMP_Enabled, (byte) ((value == true) ? 1 : 0));
            }
        }
        #endregion

        #region FIFO
        /// <summary>
        /// Get the current FIFO buffer size.
        /// </summary>
        /// <remarks>
        /// This value indicates the number of bytes stored in the FIFO buffer. This
        /// number is in turn the number of bytes that can be read from the FIFO buffer
        /// and it is directly proportional to the number of samples available given the
        /// set of sensor data bound to be stored in the FIFO (register 35 and 36).
        /// </remarks>
        public ushort FIFOBufferLength
        {
            get
            {
                byte[] buffer = new byte[2];
                ReadBytes(RegisterAddress_FIFO_Count, buffer);

                ushort ret = (ushort) (((ushort) buffer[0]) << 8);
                ret |= (ushort) buffer[1];

                return ret;
            }
        }

        public ushort FIFOPacketSize
        {
            get
            {
                return _dmpPacketSize;
            }
        }
        #endregion

        #region General
        /// <summary>
        /// Gets the ID of the device.  Should be 0x34.
        /// </summary>
        public byte DeviceID
        {
            get
            {
                byte b;

                ReadBits(RegisterAddress_WhoAmI, Bit_WhoAmI, Bit_WhoAmILength, out b);

                return b;
            }
        }
        #endregion

        #region Gyro
        /// <summary>
        /// Get full-scale gyroscope range.
        /// </summary>
        /// <remarks>
        /// The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
        /// as described in the table below.
        /// <para/>
        /// 0 = +/- 250 degrees/sec
        /// 1 = +/- 500 degrees/sec
        /// 2 = +/- 1000 degrees/sec
        /// 3 = +/- 2000 degrees/sec
        /// </remarks>
        public GyroRange GyroFullScaleRange
        {
            get
            {
                byte b;
                ReadBits(RegisterAddress_GyroConfig, Bit_GyroConfig_FullScaleRange, Bit_GyroConfig_FullScaleRangeLength, out b);

                return (GyroRange) b;
            }
            set
            {
                WriteBits(RegisterAddress_GyroConfig, Bit_GyroConfig_FullScaleRange, Bit_GyroConfig_FullScaleRangeLength, (byte) value);
            }
        }

        public int GyroOffsetX
        {
            get
            {
                byte b;
                ReadBits(RegisterAddress_GyroConfig_OffsetX, Bit_GyroConfig_Offset, Bit_GyroConfig_OffsetLength, out b);

                return (int) b;
            }
            set
            {
                WriteBits(RegisterAddress_GyroConfig_OffsetX, Bit_GyroConfig_Offset, Bit_GyroConfig_OffsetLength, (byte) value);
            }
        }

        public int GyroOffsetY
        {
            get
            {
                byte b;
                ReadBits(RegisterAddress_GyroConfig_OffsetY, Bit_GyroConfig_Offset, Bit_GyroConfig_OffsetLength, out b);

                return (int) b;
            }
            set
            {
                WriteBits(RegisterAddress_GyroConfig_OffsetY, Bit_GyroConfig_Offset, Bit_GyroConfig_OffsetLength, (byte) value);
            }
        }

        public int GyroOffsetZ
        {
            get
            {
                byte b;
                ReadBits(RegisterAddress_GyroConfig_OffsetZ, Bit_GyroConfig_Offset, Bit_GyroConfig_OffsetLength, out b);

                return (int) b;
            }
            set
            {
                WriteBits(RegisterAddress_GyroConfig_OffsetZ, Bit_GyroConfig_Offset, Bit_GyroConfig_OffsetLength, (byte) value);
            }
        }

        /// <summary>
        /// Get/set gyroscope output rate divider.
        /// </summary>
        /// <remarks> 
        /// The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
        /// Motion detection, and Free Fall detection are all based on the Sample Rate.
        /// The Sample Rate is generated by dividing the gyroscope output rate by
        /// SMPLRT_DIV:
        /// <para/>
        /// Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
        /// <para/>
        /// where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
        /// * 7), and 1kHz when the DLPF is enabled (see Register 26).
        /// <para/>
        /// Note: The accelerometer output rate is 1kHz. This means that for a Sample
        /// Rate greater than 1kHz, the same accelerometer sample may be output to the
        /// FIFO, DMP, and sensor registers more than once.
        /// <para/>
        /// For a diagram of the gyroscope and accelerometer signal paths, see Section 8
        /// of the MPU-6000/MPU-6050 Product Specification document.
        /// </remarks>
        public int GyroSampleRate
        {
            get
            {
                byte b;
                ReadByte(RegisterAddress_SampleRateDivider, out b);

                return (int) b;
            }
            set
            {
                WriteByte(RegisterAddress_SampleRateDivider, (byte) value);
            }
        }

        public ushort GyroUserOffsetX
        {
            get
            {
                byte[] buffer = new byte[2];
                ReadBytes(RegisterAddress_GyroConfig_UserOffsetX, buffer);

                ushort ret = (ushort) (((ushort) buffer[0]) << 8);
                ret |= (ushort) buffer[1];

                return ret;
            }
            set
            {
                Write(RegisterAddress_GyroConfig_UserOffsetX, value);
            }
        }

        public ushort GyroUserOffsetY
        {
            get
            {
                byte[] buffer = new byte[2];
                ReadBytes(RegisterAddress_GyroConfig_UserOffsetY, buffer);

                ushort ret = (ushort) (((ushort) buffer[0]) << 8);
                ret |= (ushort) buffer[1];

                return ret;
            }
            set
            {
                Write(RegisterAddress_GyroConfig_UserOffsetY, value);
            }
        }

        public ushort GyroUserOffsetZ
        {
            get
            {
                byte[] buffer = new byte[2];
                ReadBytes(RegisterAddress_GyroConfig_UserOffsetZ, buffer);

                ushort ret = (ushort) (((ushort) buffer[0]) << 8);
                ret |= (ushort) buffer[1];

                return ret;
            }
            set
            {
                Write(RegisterAddress_GyroConfig_UserOffsetZ, value);
            }
        }
        #endregion

        #region Interrupts
        /// <summary>
        /// Get/set full interrupt enabled status.
        /// </summary>
        /// <remarks>
        /// Full register byte for all interrupts, for quick reading. Each bit will be
        /// set 0 for disabled, 1 for enabled.
        /// </remarks>
        public byte InterruptEnabled
        {
            get
            {
                byte b;
                ReadByte(RegisterAddress_Interrupt_Enable, out b);

                return b;
            }
            set
            {
                WriteByte(RegisterAddress_Interrupt_Enable, value);
            }
        }

        /// <summary>
        /// Get full set of interrupt status bits.
        /// </summary>
        /// <remarks>
        /// These bits clear to 0 after the register has been read. Very useful
        /// for getting multiple INT statuses, since each single bit read clears
        ///  all of them because it has to read the whole byte.
        /// </summary> @return Current interrupt status
        public byte InterruptStatus
        {
            get
            {
                byte b;
                ReadByte(RegisterAddress_Interrupt_Status, out b);

                return b;
            }
        }
        #endregion

        #region Motion
        /// <summary>
        /// Get/set motion detection event duration threshold.
        /// </summary>
        /// <remarks>
        /// This register configures the duration counter threshold for Motion interrupt
        /// generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit
        /// of 1LSB = 1ms. The Motion detection duration counter increments when the
        /// absolute value of any of the accelerometer measurements exceeds the Motion
        /// detection threshold (Register 31). The Motion detection interrupt is
        /// triggered when the Motion detection counter reaches the time count specified
        /// in this register.
        /// <para/>
        /// For more details on the Motion detection interrupt, see Section 8.3 of the
        /// MPU-6000/MPU-6050 Product Specification document.
        /// </remarks>
        public int MotionDetectionDuration
        {
            get
            {
                byte b;
                ReadByte(RegisterAddress_Motion_Duration, out b);

                return (int) b;
            }
            set
            {
                WriteByte(RegisterAddress_Motion_Duration, (byte) value);
            }
        }

        /// <summary>
        /// Get/set motion detection event acceleration threshold.
        /// </summary>
        /// <remarks>
        /// This register configures the detection threshold for Motion interrupt
        /// generation. The unit of MOT_THR is 1LSB = 2mg. Motion is detected when the
        /// absolute value of any of the accelerometer measurements exceeds this Motion
        /// detection threshold. This condition increments the Motion detection duration
        /// counter (Register 32). The Motion detection interrupt is triggered when the
        /// Motion Detection counter reaches the time count specified in MOT_DUR
        /// (Register 32).
        /// <para/>
        /// The Motion interrupt will indicate the axis and polarity of detected motion
        /// in MOT_DETECT_STATUS (Register 97).
        /// <para/>
        /// For more details on the Motion detection interrupt, see Section 8.3 of the
        /// MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and
        /// 58 of this document.
        /// </remarks>
        public int MotionDetectionThreshold
        {
            get
            {
                byte b;
                ReadByte(RegisterAddress_Motion_Threshold, out b);

                return (int) b;
            }
            set
            {
                WriteByte(RegisterAddress_Motion_Threshold, (byte) value);
            }
        }

        /// <summary>
        /// Get zero motion detection event duration threshold.
        /// </summary>
        /// <remarks>
        /// This register configures the duration counter threshold for Zero Motion
        /// interrupt generation. The duration counter ticks at 16 Hz, therefore
        /// ZRMOT_DUR has a unit of 1 LSB = 64 ms. The Zero Motion duration counter
        /// increments while the absolute value of the accelerometer measurements are
        /// each less than the detection threshold (Register 33). The Zero Motion
        /// interrupt is triggered when the Zero Motion duration counter reaches the time
        /// count specified in this register.
        /// <para/>
        /// For more details on the Zero Motion detection interrupt, see Section 8.4 of
        /// the MPU-6000/MPU-6050 Product Specification document, as well as Registers 56
        /// and 58 of this document.
        /// </remarks>
        public int ZeroMotionDetectionDuration
        {
            get
            {
                byte b;
                ReadByte(RegisterAddress_ZeroMotion_Duration, out b);

                return (int) b;
            }
            set
            {
                WriteByte(RegisterAddress_ZeroMotion_Duration, (byte) value);
            }
        }

        /// <summary>
        /// Get/set zero motion detection event acceleration threshold.
        /// </summary>
        /// <remarks>
        /// This register configures the detection threshold for Zero Motion interrupt
        /// generation. The unit of ZRMOT_THR is 1LSB = 2mg. Zero Motion is detected when
        /// the absolute value of the accelerometer measurements for the 3 axes are each
        /// less than the detection threshold. This condition increments the Zero Motion
        /// duration counter (Register 34). The Zero Motion interrupt is triggered when
        /// the Zero Motion duration counter reaches the time count specified in
        /// ZRMOT_DUR (Register 34).
        /// <para/>
        /// Unlike Free Fall or Motion detection, Zero Motion detection triggers an
        /// interrupt both when Zero Motion is first detected and when Zero Motion is no
        /// longer detected.
        /// <para/>
        /// When a zero motion event is detected, a Zero Motion Status will be indicated
        /// in the MOT_DETECT_STATUS register (Register 97). When a motion-to-zero-motion
        /// condition is detected, the status bit is set to 1. When a zero-motion-to-
        /// motion condition is detected, the status bit is set to 0.
        /// <para/>
        /// For more details on the Zero Motion detection interrupt, see Section 8.4 of
        /// the MPU-6000/MPU-6050 Product Specification document as well as Registers 56
        /// and 58 of this document.
        /// </remarks>
        public int ZeroMotionDetectionThreshold
        {
            get
            {
                byte b;
                ReadByte(RegisterAddress_ZeroMotion_Threshold, out b);

                return (int) b;
            }
            set
            {
                WriteByte(RegisterAddress_ZeroMotion_Threshold, (byte) value);
            }
        }
        #endregion

        #region User Control
        /// <summary>
        /// Get/set FIFO enabled status.
        /// </summary>
        /// <remarks>
        /// When this bit is set to 0, the FIFO buffer is disabled. The FIFO buffer
        /// cannot be written to or read from while disabled. The FIFO buffer's state
        /// does not change unless the MPU-60X0 is power cycled.
        /// </remarks>
        public bool FIFOEnabled
        {
            get
            {
                byte b;
                ReadBit(RegisterAddress_UserControl, Bit_UserControl_FIFO_Enabled, out b);

                return (b == 1);
            }
            set
            {
                WriteBit(RegisterAddress_UserControl, Bit_UserControl_FIFO_Enabled, (byte) ((value == true) ? 1 : 0));
            }
        }

        /// <summary>
        /// Get/set I2C Master Mode enabled status.
        /// </summary>
        /// <remarks>
        /// When this mode is enabled, the MPU-60X0 acts as the I2C Master to the
        /// external sensor slave devices on the auxiliary I2C bus. When this bit is
        /// cleared to 0, the auxiliary I2C bus lines (AUX_DA and AUX_CL) are logically
        /// driven by the primary I2C bus (SDA and SCL). This is a precondition to
        /// enabling Bypass Mode. For further information regarding Bypass Mode, please
        /// refer to Register 55.
        /// </remarks>
        public bool I2CMasterModeEnabled
        {
            get
            {
                byte b;
                ReadBit(RegisterAddress_UserControl, Bit_UserControl_I2C_MasterEnabled, out b);

                return (b == 1);
            }
            set
            {
                WriteBit(RegisterAddress_UserControl, Bit_UserControl_I2C_MasterEnabled, (byte) ((value == true) ? 1 : 0));
            }
        }
        #endregion

        #region Power Management
        /// <summary>
        /// Get/set clock source.
        /// </summary>
        public ClockSource ClockSource
        {
            get
            {
                byte b;

                ReadBits(RegisterAddress_PowerManagement1, Bit_Power1_ClockSelect, Bit_Power1_ClockSelectLength, out b);

                return (ClockSource) b;
            }
            set
            {
                WriteBits(RegisterAddress_PowerManagement1, Bit_Power1_ClockSelect, Bit_Power1_ClockSelectLength, (byte) value);
            }
        }

        /// <summary>
        /// Get/set sleep mode status.
        /// </summary>
        /// <remarks>
        /// Setting the SLEEP bit in the register puts the device into very low power
        /// sleep mode. 
        /// <p/>
        /// In this mode, only the serial interface and internal registers
        /// remain active, allowing for a very low standby current. Clearing this bit
        /// puts the device back into normal mode. 
        /// <p/>
        /// To save power, the individual standby selections for each of the gyros should 
        /// be used if any gyro axis is not used by the application.
        /// </remarks>
        public bool SleepEnabled
        {
            get
            {
                byte b;

                ReadBit(RegisterAddress_PowerManagement1, Bit_Power1_Sleep, out b);

                return (b == 1);
            }
            set
            {
                WriteBit(RegisterAddress_PowerManagement1, Bit_Power1_Sleep, (byte) ((value == true) ? 1 : 0));
            }
        }

        /// <summary>
        /// Get/set wake cycle enabled status.
        /// </summary>
        /// <remarks>
        /// When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle
        /// between sleep mode and waking up to take a single sample of data from active
        /// sensors at a rate determined by LP_WAKE_CTRL (register 108).
        /// </remarks>
        public bool WakeCycleEnabled
        {
            get
            {
                byte b;
                ReadBit(RegisterAddress_PowerManagement1, Bit_Power1_CycleBit, out b);

                return (b == 1);
            }
            set
            {
                WriteBit(RegisterAddress_PowerManagement1, Bit_Power1_CycleBit, (byte) ((value == true) ? 1 : 0));
            }
        }
        #endregion

        #region Undocumented
        public bool OTPBankValid
        {
            get
            {
                byte b;
                ReadBit(RegisterAddress_GyroConfig_OffsetX, Bit_Undocumented_OTPBankValid, out b);

                return (b == 1);
            }
            set
            {
                WriteBit(RegisterAddress_GyroConfig_OffsetX, Bit_Undocumented_OTPBankValid, (byte) ((value == true) ? 1 : 0));
            }
        }
        #endregion
        #endregion

        #region Members
        private ushort _dmpPacketSize;
        #endregion

        #region Constructor
        public MPU6050(I2CBus bus, ushort address, int clockRate)
            : base(bus, address, clockRate)
        {

        }
        #endregion

        #region Methods
        #region Initialization
        public void Initialize()
        {
            this.ClockSource = ClockSource.GyroX;
            this.GyroFullScaleRange = GyroRange.R250;
            this.AccelerometerFullScaleRange = AccelerometerRange.R2;
            this.SleepEnabled = false;
        }

        /// <summary>
        /// Verifies the I2C connection.
        /// </summary>
        /// <returns>True if we're connected to an MPU6050, false if not.</returns>
        public bool TestConnection()
        {
            return (this.DeviceID == ExpectedDeviceID);
        }
        #endregion

        #region DMP
        public int DMPGetEuler(float[] data, Quaternion q)
        {
            data[0] = (float) System.Math.Atan2(2 * q.X * q.Y - 2 * q.W * q.Z, 2 * q.W * q.W + 2 * q.X * q.X - 1);	// psi
            data[1] = (float) -System.Math.Asin(2 * q.X * q.Z + 2 * q.W * q.Y);										// theta
            data[2] = (float) System.Math.Atan2(2 * q.Y * q.Z - 2 * q.W * q.X, 2 * q.W * q.W + 2 * q.Z * q.Z - 1);	// phi

            return 0;
        }

        public int DMPGetGravity(float[] v, Quaternion q)
        {
            v[0] = 2 * (q.X * q.Z - q.W * q.Y);
            v[1] = 2 * (q.W * q.X + q.Y * q.Z);
            v[2] = 2 * (q.W * q.W - q.X * q.X - q.Y * q.Y + q.Z * q.Z);

            return 0;
        }

        public int DMPGetYawPitchRoll(float[] data, Quaternion q, float[] gravity)
        {
            // yaw: (about Z axis)
            data[0] = (float) System.Math.Atan2(2 * q.X * q.Y - 2 * q.W * q.Z, 2 * q.W * q.W + 2 * q.X * q.X - 1);

            // pitch: (nose up/down, about Y axis)
            data[1] = (float) System.Math.Atan(gravity[0] / (float) System.Math.Sqrt(gravity[1] * gravity[1] + gravity[2] * gravity[2]));

            // roll: (tilt left/right, about X axis)
            data[2] = (float) System.Math.Atan(gravity[1] / (float) System.Math.Sqrt(gravity[0] * gravity[0] + gravity[2] * gravity[2]));

            return 0;
        }

        public int DMPGetQuaternion(ref Quaternion q, byte[] buffer)
        {
            // TODO: accommodate different arrangements of sent data (ONLY default supported now)
            short[] tmp = new short[4];
            int status = DMPGetQuaternion(tmp, buffer);

            if (status == 0) {
                q = new Quaternion(
                    (float) (tmp[0] / 16384.0f),
                    (float) (tmp[1] / 16384.0f),
                    (float) (tmp[2] / 16384.0f),
                    (float) (tmp[3] / 16384.0f)
                );

                return 0;
            }

            return status; // int16 return value, indicates error if this line is reached
        }

        public int DMPGetQuaternion(short[] data, byte[] buffer)
        {
            data[0] = (short) ((buffer[0] << 8) + buffer[1]);
            data[1] = (short) ((buffer[4] << 8) + buffer[5]);
            data[2] = (short) ((buffer[8] << 8) + buffer[9]);
            data[3] = (short) ((buffer[12] << 8) + buffer[13]);

            return 0;
        }

        public int DMPInitialize()
        {
            // reset device
            Debug.Print("Resetting MPU6050...");
            Reset();

            Thread.Sleep(50); // wait after reset

            // enable sleep mode and wake cycle
            Debug.Print("Enabling sleep mode...");
            this.SleepEnabled = true;

            Debug.Print("Enabling wake cycle...");
            this.WakeCycleEnabled = true;

            // disable sleep mode
            Debug.Print("Disabling sleep mode...");
            this.SleepEnabled = false;

            // get MPU hardware revision
            Debug.Print("Selecting user bank 16...");
            SetMemoryBank(0x10, true, true);

            Debug.Print("Selecting memory byte 6...");
            SetMemoryStartAddress(0x06);

            Debug.Print("Checking hardware revision...");
            byte hwRevision = ReadMemoryByte();

            Debug.Print("Revision @ user[16][6] = 0x" + hwRevision.ToString("X2"));
            Debug.Print("Resetting memory bank selection to 0...");
            SetMemoryBank(0, false, false);

            // check OTP bank valid
            Debug.Print("Reading OTP bank valid flag...");
            Debug.Print("OTP bank is " + ((this.OTPBankValid == true) ? "valid" : "invalid!"));

            // get X/Y/Z gyro offsets
            Debug.Print("Reading gyro offset values...");

            int gyroOffsetX = this.GyroOffsetX;
            int gyroOffsetY = this.GyroOffsetY;
            int gyroOffsetZ = this.GyroOffsetZ;

            Debug.Print("...X offset: " + gyroOffsetX.ToString());
            Debug.Print("...Y offset: " + gyroOffsetY.ToString());
            Debug.Print("...Z offset: " + gyroOffsetZ.ToString());

            // setup weird slave stuff (?)
            Debug.Print("Setting slave 0 address to 0x7F...");
            SetSlaveAddress(0, 0x7F);

            Debug.Print("Disabling I2C Master mode...");
            this.I2CMasterModeEnabled = false;

            Debug.Print("Setting slave 0 address to 0x68 (self)...");
            SetSlaveAddress(0, 0x68);

            Debug.Print("Resetting I2C Master control...");
            ResetI2CMaster();

            Thread.Sleep(50);

            // load DMP code into memory banks
            Debug.Print("Writing DMP code to MPU memory banks (" + DMPProgram.Length.ToString() + " bytes)");

            if (WriteProgramMemoryBlock(DMPProgram) == true) {
                Debug.Print("Success! DMP code written and verified.");

                // write DMP configuration
                Debug.Print("Writing DMP configuration to MPU memory banks (" + DMPConfigProgram.Length.ToString() + " bytes in config def)");

                if (WriteProgramDMPConfigurationSet(DMPConfigProgram) == true) {
                    Debug.Print("Success! DMP configuration written and verified.");

                    Debug.Print("Setting clock source to Z Gyro...");
                    this.ClockSource = ClockSource.GyroZ;

                    Debug.Print("Setting DMP and FIFO_OFLOW interrupts enabled...");
                    this.InterruptEnabled = 0x12;

                    Debug.Print("Setting sample rate to 200Hz...");
                    this.GyroSampleRate = 4; // 1khz / (1 + 4) = 200 Hz

                    Debug.Print("Setting external frame sync to TEMP_OUT_L[0]...");
                    this.ExternalFrameSync = ExternalFrameSyncSource.Temperature;

                    Debug.Print("Setting DLPF bandwidth to 42Hz...");
                    this.DigitalLowPassFilterBandwidth = DigitalLowPassFilterBandwidth.BW42;

                    Debug.Print("Setting gyro sensitivity to +/- 2000 deg/sec...");
                    this.GyroFullScaleRange = GyroRange.R2000;

                    Debug.Print("Setting DMP configuration bytes (function unknown)...");
                    this.DMPConfig1 = 0x03;
                    this.DMPConfig2 = 0x00;

                    Debug.Print("Clearing OTP Bank flag...");
                    this.OTPBankValid = false;

                    Debug.Print("Setting X/Y/Z gyro offsets to previous values...");
                    this.GyroOffsetX = gyroOffsetX;
                    this.GyroOffsetY = gyroOffsetY;
                    this.GyroOffsetZ = gyroOffsetZ;

                    Debug.Print("Setting X/Y/Z gyro user offsets to zero...");
                    this.GyroUserOffsetX = 0;
                    this.GyroUserOffsetY = 0;
                    this.GyroUserOffsetZ = 0;

                    Debug.Print("Writing final memory update 1/7 (function unknown)...");
                    byte[] dmpUpdate = new byte[16];
                    int pos = 0;

                    for (int j = 0; (j < 4) || (j < dmpUpdate[2] + 3); j++, pos++) {
                        dmpUpdate[j] = DMPUpdateProgram[pos];
                    }

                    WriteMemoryBlock(dmpUpdate, 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

                    Debug.Print("Writing final memory update 2/7 (function unknown)...");

                    for (int j = 0; (j < 4) || (j < dmpUpdate[2] + 3); j++, pos++) {
                        dmpUpdate[j] = DMPUpdateProgram[pos];
                    }

                    WriteMemoryBlock(dmpUpdate, 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

                    Debug.Print("Resetting FIFO...");
                    ResetFIFO();

                    Debug.Print("Reading FIFO count...");
                    ushort fifoLength = this.FIFOBufferLength;
                    byte[] fifoBuffer = new byte[fifoLength];

                    Debug.Print("Current FIFO count=" + fifoLength.ToString());
                    ReadFIFO(fifoBuffer);

                    Debug.Print("Setting motion detection threshold to 2...");
                    this.MotionDetectionThreshold = 2;

                    Debug.Print("Setting zero-motion detection threshold to 156...");
                    this.ZeroMotionDetectionThreshold = 156;

                    Debug.Print("Setting motion detection duration to 80...");
                    this.MotionDetectionDuration = 80;

                    Debug.Print("Setting zero-motion detection duration to 0...");
                    this.ZeroMotionDetectionDuration = 0;

                    Debug.Print("Resetting FIFO...");
                    ResetFIFO();

                    Debug.Print("Enabling FIFO...");
                    this.FIFOEnabled = true;

                    Debug.Print("Enabling DMP...");
                    this.DMPEnabled = true;

                    Debug.Print("Resetting DMP...");
                    ResetDMP();

                    Debug.Print("Writing final memory update 3/7 (function unknown)...");

                    for (int j = 0; (j < 4) || (j < dmpUpdate[2] + 3); j++, pos++) {
                        dmpUpdate[j] = DMPUpdateProgram[pos];
                    }

                    WriteMemoryBlock(dmpUpdate, 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

                    Debug.Print("Writing final memory update 4/7 (function unknown)...");

                    for (int j = 0; (j < 4) || (j < dmpUpdate[2] + 3); j++, pos++) {
                        dmpUpdate[j] = DMPUpdateProgram[pos];
                    }

                    WriteMemoryBlock(dmpUpdate, 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

                    Debug.Print("Writing final memory update 5/7 (function unknown)...");

                    for (int j = 0; (j < 4) || (j < dmpUpdate[2] + 3); j++, pos++) {
                        dmpUpdate[j] = DMPUpdateProgram[pos];
                    }

                    WriteMemoryBlock(dmpUpdate, 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

                    Debug.Print("Waiting for FIFO count > 2...");
                    while ((fifoLength = this.FIFOBufferLength) < 3) ;
                    fifoBuffer = new byte[fifoLength];
                    Debug.Print("Current FIFO count=" + fifoLength.ToString());

                    Debug.Print("Reading FIFO data...");
                    ReadFIFO(fifoBuffer);

                    Debug.Print("Reading interrupt status...");
                    byte mpuIntStatus = this.InterruptStatus;
                    Debug.Print("Current interrupt status=0x" + mpuIntStatus.ToString("X2"));

                    Debug.Print("Reading final memory update 6/7 (function unknown)...");
                    for (int j = 0; (j < 4) || (j < dmpUpdate[2] + 3); j++, pos++) {
                        dmpUpdate[j] = DMPUpdateProgram[pos];
                    }

                    WriteMemoryBlock(dmpUpdate, 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

                    Debug.Print("Waiting for FIFO count > 2...");
                    while ((fifoLength = this.FIFOBufferLength) < 3) ;
                    fifoBuffer = new byte[fifoLength];
                    Debug.Print("Current FIFO count=" + fifoLength.ToString());

                    Debug.Print("Reading FIFO data...");
                    ReadFIFO(fifoBuffer);

                    Debug.Print("Reading interrupt status...");
                    mpuIntStatus = this.InterruptStatus;
                    Debug.Print("Current interrupt status=0x" + mpuIntStatus.ToString("X2"));

                    Debug.Print("Writing final memory update 7/7 (function unknown)...");

                    for (int j = 0; (j < 4) || (j < dmpUpdate[2] + 3); j++, pos++) {
                        dmpUpdate[j] = DMPUpdateProgram[pos];
                    }

                    WriteMemoryBlock(dmpUpdate, 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

                    Debug.Print("DMP is good to go! Finally.");

                    Debug.Print("Disabling DMP (you turn it on later)...");
                    this.DMPEnabled = false;

                    Debug.Print("Setting up internal 42-byte (default) DMP packet buffer...");
                    _dmpPacketSize = 42;

                    Debug.Print("Resetting FIFO and clearing INT status one last time...");
                    ResetFIFO();
                    mpuIntStatus = this.InterruptStatus;
                } else {
                    Debug.Print("ERROR! DMP configuration verification failed.");
                    return 2; // configuration block loading failed
                }
            } else {
                Debug.Print("ERROR! DMP code verification failed.");
                return 1; // main binary block loading failed
            }

            return 0; // success
        }

        public void ResetDMP()
        {
            WriteBit(RegisterAddress_UserControl, Bit_UserControl_DMP_Reset, 1);
        }
        #endregion

        #region Memory
        public byte ReadMemoryByte()
        {
            byte b;
            ReadByte(RegisterAddress_Memory_ReadWrite, out b);

            return b;
        }

        public void SetMemoryBank(byte bank, bool prefetchEnabled = false, bool userBank = false)
        {
            bank &= 0x1F;

            if (userBank == true) {
                bank |= 0x20;
            }

            if (prefetchEnabled == true) {
                bank |= 0x40;
            }

            WriteByte(RegisterAddress_Memory_BankSelect, bank);
        }

        public void SetMemoryStartAddress(byte address)
        {
            WriteByte(RegisterAddress_Memory_StartAddress, address);
        }

        public bool WriteDMPConfigurationSet(byte[] data)
        {
            // config set data is a long string of blocks with the following structure:
            // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]

            int dataLength = data.Length;
            bool success;
            byte special;

            for (int i = 0; i < dataLength; ) {
                byte bank = data[i++];
                byte offset = data[i++];
                byte length = data[i++];

                // write data or perform special action
                if (length > 0) {
                    // regular block of data to write
                    success = WriteMemoryBlock(data, i, length, bank, offset, true);
                    i += length;
                } else {
                    // special instruction
                    // NOTE: this kind of behavior (what and when to do certain things)
                    // is totally undocumented. This code is in here based on observed
                    // behavior only, and exactly why (or even whether) it has to be here
                    // is anybody's guess for now.
                    special = data[i++];

                    if (special == 0x01) {
                        // enable DMP-related interrupts

                        //setIntZeroMotionEnabled(true);
                        //setIntFIFOBufferOverflowEnabled(true);
                        //setIntDMPEnabled(true);
                        WriteByte(RegisterAddress_Interrupt_Enable, 0x32); // single operation

                        success = true;
                    } else {
                        // unknown special command
                        success = false;
                    }
                }

                if (success == false) {
                    return false;
                }
            }

            return true;
        }

        public bool WriteMemoryBlock(byte[] data, int dataOffset, int dataLength, byte bank = 0, byte address = 0, bool verify = true)
        {
            SetMemoryBank(bank);
            SetMemoryStartAddress(address);

            byte[] verifyBuffer = null;

            if (verify == true) {
                //verifyBuffer = new byte[DMPMemoryChunkSize];
            }

            for (int i = 0; i < dataLength; ) {
                // determine correct chunk size according to bank position and data size
                byte chunkSize = DMPMemoryChunkSize;
                chunkSize = 255;
                // make sure we don't go past the data size
                if ((i + chunkSize) > dataLength) {
                    chunkSize = (byte) (dataLength - i);
                }

                // make sure this chunk doesn't go past the bank boundary (256 bytes)
                if (chunkSize > (256 - address)) {
                    chunkSize = (byte) (256 - address);
                }

                // write the chunk of data as specified
                WriteBytes(RegisterAddress_Memory_ReadWrite, data, dataOffset + i, chunkSize);

                // verify data if needed
                if (verify == true) {
                    verifyBuffer = new byte[chunkSize];

                    SetMemoryBank(bank);
                    SetMemoryStartAddress(address);

                    ReadBytes(RegisterAddress_Memory_ReadWrite, verifyBuffer, chunkSize);

                    byte[] tmp = new byte[chunkSize];

                    Array.Copy(data, dataOffset + i, tmp, 0, chunkSize);

                    if (tmp.Equals(verifyBuffer) == false) {
                        //return false; // uh oh.
                    }
                }

                // increase byte index by [chunkSize]
                i += chunkSize;

                // uint8_t automatically wraps to 0 at 256
                address += chunkSize;

                // if we aren't done, update bank (if necessary) and address
                if (i < dataLength) {
                    if (address == 0) {
                        bank++;
                    }

                    SetMemoryBank(bank);
                    SetMemoryStartAddress(address);
                }
            }

            return true;
        }

        public bool WriteProgramDMPConfigurationSet(byte[] data)
        {
            return WriteDMPConfigurationSet(data);
        }

        public bool WriteProgramMemoryBlock(byte[] data, byte bank = 0, byte address = 0, bool verify = true)
        {
            return WriteMemoryBlock(data, 0, data.Length, bank, address, verify);
        }
        #endregion

        #region Power management
        /// <summary>
        /// Trigger a full device reset.
        /// </summary>
        /// <remarks>
        /// A small delay of ~50ms may be desirable after triggering a reset.
        /// </remarks>
        public void Reset()
        {
            WriteBit(RegisterAddress_PowerManagement1, Bit_Power1_Reset, (byte) 1);
        }
        #endregion

        #region Slave
        /// <summary>
        /// Get the I2C address of the specified slave (0-3).
        /// </summary>
        /// <remarks>
        /// Note that Bit 7 (MSB) controls read/write mode. If Bit 7 is set, it's a read
        /// operation, and if it is cleared, then it's a write operation. The remaining
        /// bits (6-0) are the 7-bit device address of the slave device.
        /// <para/>
        /// In read mode, the result of the read is placed in the lowest available 
        /// EXT_SENS_DATA register. For further information regarding the allocation of
        /// read results, please refer to the EXT_SENS_DATA register description
        /// (Registers 73 - 96).
        /// <para/>
        /// The MPU-6050 supports a total of five slaves, but Slave 4 has unique
        /// characteristics, and so it has its own functions (getSlave4* and setSlave4*).
        /// <para/>
        /// I2C data transactions are performed at the Sample Rate, as defined in
        /// Register 25. The user is responsible for ensuring that I2C data transactions
        /// to and from each enabled Slave can be completed within a single period of the
        /// Sample Rate.
        /// <para/>
        /// The I2C slave access rate can be reduced relative to the Sample Rate. This
        /// reduced access rate is determined by I2C_MST_DLY (Register 52). Whether a
        /// slave's access rate is reduced relative to the Sample Rate is determined by
        /// I2C_MST_DELAY_CTRL (Register 103).
        /// <para/>
        /// The processing order for the slaves is fixed. The sequence followed for
        /// processing the slaves is Slave 0, Slave 1, Slave 2, Slave 3 and Slave 4. If a
        /// particular Slave is disabled it will be skipped.
        /// <para/>
        /// Each slave can either be accessed at the sample rate or at a reduced sample
        /// rate. In a case where some slaves are accessed at the Sample Rate and some
        /// slaves are accessed at the reduced rate, the sequence of accessing the slaves
        /// (Slave 0 to Slave 4) is still followed. However, the reduced rate slaves will
        /// be skipped if their access rate dictates that they should not be accessed
        /// during that particular cycle. 
        /// <para/>
        /// For further information regarding the reduced
        /// access rate, please refer to Register 52. Whether a slave is accessed at the
        /// Sample Rate or at the reduced rate is determined by the Delay Enable bits in
        /// Register 103.
        /// </remarks>
        /// <param name="slave">Slave number (0-3).</param>
        /// <returns>The current address for the specified slave.</returns>
        public byte GetSlaveAddress(int slave)
        {
            if (slave > 3) {
                return 0;
            }

            byte b;
            ReadByte((byte) (RegisterAddress_Slave0_Address + (slave * 3)), out b);

            return b;
        }

        /// <summary>
        /// Set the I2C address of the specified slave (0-3).
        /// </summary>
        /// <param name="slave">Slave number (0-3).</param>
        /// <param name="address">New address for specified slave.</param>
        public void SetSlaveAddress(int slave, byte address)
        {
            if (slave > 3) {
                return;
            }

            WriteByte((byte) (RegisterAddress_Slave0_Address + (slave * 3)), address);
        }
        #endregion

        #region User Control
        public void ReadFIFO(byte[] buffer)
        {
            if ((buffer != null) && (buffer.Length > 0)) {
                ReadBytes(RegisterAddress_Memory_ReadWrite, buffer);
            }
        }

        /// <summary>
        /// Reset the FIFO.
        /// </summary>
        /// <remarks>
        /// This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
        /// bit automatically clears to 0 after the reset has been triggered.
        /// </remarks>
        public void ResetFIFO()
        {
            WriteBit(RegisterAddress_UserControl, Bit_UserControl_FIFO_Reset, 1);
        }

        /// <summary>
        /// Reset the I2C Master.
        /// </summary>
        /// <remarks>
        /// This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
        /// This bit automatically clears to 0 after the reset has been triggered.
        /// </remarks>
        public void ResetI2CMaster()
        {
            WriteBit(RegisterAddress_UserControl, Bit_UserControl_I2C_MasterReset, 1);
        }
        #endregion

        public void GetMotion6(out short accelX, out short accelY, out short accelZ, out short gyroX, out short gyroY, out short gyroZ)
        {
            byte[] buffer = new byte[14];

            ReadBytes(RegisterAddress_AccelerationXOut, buffer);

            accelX = (short) ((((short) buffer[0]) << 8) | buffer[1]);
            accelY = (short) ((((short) buffer[2]) << 8) | buffer[3]);
            accelZ = (short) ((((short) buffer[4]) << 8) | buffer[5]);

            gyroX = (short) ((((short) buffer[8]) << 8) | buffer[9]);
            gyroY = (short) ((((short) buffer[10]) << 8) | buffer[11]);
            gyroZ = (short) ((((short) buffer[12]) << 8) | buffer[13]);
        }
        #endregion
    }

    public enum AccelerometerRange : byte
    {
        R2 = 0x00,
        R4 = 0x01,
        R8 = 0x02,
        R16 = 0x03
    }

    public enum ClockSource : byte
    {
        Internal = 0x00,

        GyroX = 0x01,
        GyroY = 0x02,
        GyroZ = 0x03
    }

    public enum GyroRange : byte
    {
        R250 = 0x00,
        R500 = 0x01,
        R1000 = 0x02,
        R2000 = 0x03
    }

    public enum ExternalFrameSyncSource : byte
    {
        Disabled = 0x00,
        Temperature = 0x1,
        GyroX = 0x2,
        GyroY = 0x3,
        GyroZ = 0x4,
        AccelerometerX = 0x5,
        AccelerometerY = 0x6,
        AccelerometerZ = 0x7
    }

    public enum DigitalLowPassFilterBandwidth : byte
    {
        BW256 = 0x00,
        BW188 = 0x01,
        BW98 = 0x02,
        BW42 = 0x03,
        BW20 = 0x04,
        BW10 = 0x05,
        BW5 = 0x06
    }
}
