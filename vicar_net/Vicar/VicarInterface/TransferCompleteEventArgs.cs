using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Vicar.VicarInterface
{
  public class TransferCompleteEventArgs : EventArgs
  {
    public TransferCompleteEventArgs(uint id, byte errorCode, ulong transferred)
    {
      Id = id;
      ErrorCode = (VicarErrorCode)errorCode;
      Transferred = transferred;
    }

    public uint Id { get; private set; }

    public VicarErrorCode ErrorCode { get; private set; }

    public ulong Transferred { get; private set; }
  }
}
