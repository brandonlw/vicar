using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Vicar
{
  public static class Utilities
  {
    public static ushort ToLittleEndianUshort(byte[] buffer, int offset)
    {
      return (ushort)(buffer[offset + 0] | ((buffer[offset + 1] << 8) & 0xFF00));
    }

    public static ushort ToBigEndianUshort(byte[] buffer, int offset)
    {
      return (ushort)(buffer[offset + 1] | ((buffer[offset + 0] << 8) & 0xFF00));
    }

    public static uint ToLittleEndianUint(byte[] buffer, int offset)
    {
      return (uint)(buffer[offset + 0] | (uint)((buffer[offset + 1] << 8) & 0xFF00) |
        ((uint)(buffer[offset + 2] << 16) & 0xFF0000) | (uint)((buffer[offset + 3] << 24) & 0xFF000000));
    }

    public static uint ToBigEndianUint(byte[] buffer, int offset)
    {
      return (uint)(buffer[offset + 3] | (uint)((buffer[offset + 2] << 8) & 0xFF00) |
        ((uint)(buffer[offset + 1] << 16) & 0xFF0000) | (uint)((buffer[offset + 0] << 24) & 0xFF000000));
    }

    public static void SetLittleEndianUshort(byte[] buffer, int offset, ushort value)
    {
      buffer[offset + 0] = (byte)(value & 0xFF);
      buffer[offset + 1] = (byte)((value >> 8) & 0xFF);
    }

    public static void SetBigEndianUshort(byte[] buffer, int offset, ushort value)
    {
      buffer[offset + 0] = (byte)((value >> 8) & 0xFF);
      buffer[offset + 1] = (byte)(value & 0xFF);
    }

    public static void SetLittleEndianUint(byte[] buffer, int offset, uint value)
    {
      buffer[offset + 0] = (byte)(value & 0xFF);
      buffer[offset + 1] = (byte)((value >> 8) & 0xFF);
      buffer[offset + 2] = (byte)((value >> 16) & 0xFF);
      buffer[offset + 3] = (byte)((value >> 24) & 0xFF);
    }

    public static void SetBigEndianUint(byte[] buffer, int offset, uint value)
    {
      buffer[offset + 0] = (byte)((value >> 24) & 0xFF);
      buffer[offset + 1] = (byte)((value >> 16) & 0xFF);
      buffer[offset + 2] = (byte)((value >> 8) & 0xFF);
      buffer[offset + 3] = (byte)(value & 0xFF);
    }
  }
}
