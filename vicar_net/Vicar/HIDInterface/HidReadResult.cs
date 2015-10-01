using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Vicar.HIDInterface
{
  public class HidReadResult
  {
    public enum ReadStatus
    {
      Success = 0,
      WaitTimedOut = 1,
      WaitFail = 2,
      NoDataRead = 3,
      ReadError = 4,
      NotConnected = 5
    }

    public HidReadResult(ReadStatus status)
    {
      Status = status;
    }

    public HidReadResult(byte[] data, ReadStatus status)
    {
      Data = data;
      Status = status;
    }

    public byte[] Data { get; private set; }

    public ReadStatus Status { get; private set; }
  }
}
