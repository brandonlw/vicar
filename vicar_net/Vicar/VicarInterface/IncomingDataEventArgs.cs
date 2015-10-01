using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Vicar.VicarInterface
{
  public class IncomingDataEventArgs : EventArgs
  {
    public IncomingDataEventArgs(VicarErrorCode errorCode, byte address, byte endpoint, byte[] data)
    {
      ErrorCode = errorCode;
      Address = address;
      Endpoint = endpoint;
      Data = data;
    }

    public VicarErrorCode ErrorCode { get; private set; }

    public byte Address { get; private set; }

    public byte Endpoint { get; private set; }

    public byte[] Data { get; private set; }
  }
}
