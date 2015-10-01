using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Vicar.VicarInterface
{
  public enum PeripheralStatus : byte
  {
    Unknown = 0x00,
    Detached = 0x10,
    DetachedAndInitializing = 0x11,
    DetachedAndWaiting = 0x12,
    DetachedIllegal = 0x13,
    AttachedAndSettling = 0x20,
    AttachedAndWillReset = 0x30,
    AttachedAndResetting = 0x40,
    AttachedAndWaitingForSOF = 0x50,
    AttachedAndReady = 0x60,
    AttachedAndInError = 0xA0
  }
}
