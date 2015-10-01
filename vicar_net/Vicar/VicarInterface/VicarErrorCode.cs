using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Vicar.VicarInterface
{
  public enum VicarErrorCode
  {
    Unknown = -1,
    Success = 0x00,
    Busy = 0x01,
    BadRequest = 0x02,
    Undefined = 0x03,
    NAK = 0x04,
    Stall = 0x05,
    ToggleError = 0x06,
    WrongPID = 0x07,
    BadByteCount = 0x08,
    PIDError = 0x09,
    PacketError = 0x0A,
    CRCError = 0x0B,
    KError = 0x0C,
    JError = 0x0D,
    SIETimeout = 0x0E,
    Babble = 0x0F,
    RcvDavIrq = 0xF0,
    UrbMemOutOfSpace = 0xFC,
    EndpointNotConfigured = 0xFD,
    EndpointMemOutOfSpace = 0xFE,
    Timeout = 0xFF
  }
}
