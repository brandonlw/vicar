using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Vicar.VicarInterface
{
  public class VicarMessage
  {
    public byte Command { get; private set; }
    public uint Sequence { get; private set; }
    public byte[] Data { get; private set; }

    public VicarMessage(byte command, uint sequence, byte[] data)
    {
      Command = command;
      Sequence = sequence;
      Data = data;
    }
  }
}
