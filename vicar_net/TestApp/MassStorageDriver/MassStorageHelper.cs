using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Vicar.VicarInterface;

namespace Vicar.MassStorageDriver
{
  public class MassStorageHelper
  {
    private VicarDevice _device;
    private IEnumerable<byte> _accumulatedBytes;
    private uint _bytesGoal;
    private bool _isDone;

    public MassStorageHelper(VicarDevice device)
    {
      _device = device;

      _device.IncomingDataReceived += _device_IncomingDataReceived;
    }

    public uint[] GetPartitionTableEntries()
    {
      var bootSector = ReadSector(0, 1);

      var ret = new uint[4];
      for (int i = 0; i < 4; i++)
      {
        ret[i] = Utilities.ToLittleEndianUint(bootSector, 0x1C6 + (i * 0x10));
      }

      return ret;
    }

    public byte[] ReadSector(ulong lba, int blocks)
    {
      _bytesGoal = (uint)(blocks * 512 + 0x0D);
      var d = _StartCBW(0x28, lba, blocks, true);
      _device.SendEndpointData(0x01, Program.OutgoingEndpoint.GetValueOrDefault(), d);
      if (Program.UseUrbs)
      {
        _device.SubmitUrb(0x01, Program.IncomingEndpoint.GetValueOrDefault(), _bytesGoal);
      }
      
      while (!_isDone)
      {
        Thread.Sleep(1);
      }

      return _accumulatedBytes.Take(blocks * 512).ToArray();
    }

    public void WriteSector(ulong lba, byte[] data)
    {
      _bytesGoal = 0x0D;
      var d = _StartCBW(0x2A, lba, data.Length / 512, false);
      _device.SendEndpointData(0x01, Program.OutgoingEndpoint.GetValueOrDefault(), d);
      int i = 0;
      while (i < data.Length)
      {
        _device.SendEndpointData(0x01, Program.OutgoingEndpoint.GetValueOrDefault(),
          data.Skip(i).Take(64).ToArray());
        i += 64;
      }

      if (Program.UseUrbs)
      {
        _device.SubmitUrb(0x01, Program.IncomingEndpoint.GetValueOrDefault(), _bytesGoal);
      }

      while (!_isDone)
      {
        Thread.Sleep(1);
      }
    }

    private byte[] _StartCBW(byte cmd, ulong lba, int blocks, bool incoming)
    {
      _isDone = false;
      _accumulatedBytes = new List<byte>();
      var d = new byte[0x1F];
      d[0] = (byte)'U';
      d[1] = (byte)'S';
      d[2] = (byte)'B';
      d[3] = (byte)'C';
      d[4] = 0xC0;
      d[5] = 0x68;
      d[6] = 0x5D;
      d[7] = 0x15;
      Utilities.SetLittleEndianUint(d, 8, (uint)(blocks * 512));
      d[0x0C] = (byte)(incoming ? 0x80 : 0x00);
      d[0x0E] = 0x0A;
      d[0x0F] = cmd;
      Utilities.SetBigEndianUint(d, 0x11, (uint)lba);
      Utilities.SetBigEndianUshort(d, 0x16, (ushort)blocks);

      return d;
    }

    private void _device_IncomingDataReceived(object sender, IncomingDataEventArgs e)
    {
      if (_accumulatedBytes.Count() < _bytesGoal)
      {
        _accumulatedBytes = _accumulatedBytes.Concat(e.Data);
        if (_accumulatedBytes.Count() >= _bytesGoal)
        {
          _isDone = true;
        }
      }
      else
      {
        _isDone = true;
      }
    }
  }
}
