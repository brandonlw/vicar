using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Vicar.VicarInterface
{
  public class VicarException : Exception
  {
    public VicarException(string message)
      : base(message)
    {
      ErrorCode = VicarErrorCode.Unknown;
    }

    public VicarException(VicarErrorCode errorCode, string message)
      : base(message)
    {
      ErrorCode = errorCode;
    }

    public VicarErrorCode ErrorCode { get; set; }

    public override string Message
    {
      get
      {
        return string.Format("{0} ({1})", base.Message, ErrorCode.ToString());
      }
    }
  }
}
