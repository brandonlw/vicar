using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Vicar.HIDInterface;
using Vicar.WinUSBInterface;

namespace Vicar.VicarInterface
{
  public partial class VicarDevice
  {
    private const int _HID_VENDOR_ID = 0xDEAD;
    private const int _HID_PRODUCT_ID = 0xBEEF;
    private const int _HID_REPORT_ID = 0x01;
    private const int _HID1_REPORT_SIZE = 0x80;
    private const int _CUSTOM_INTERFACE_ID = 0x00;
    private const int _CUSTOM_INCOMING_CMD_ENDPOINT = 0x81;
    private const int _CUSTOM_INCOMING_DATA_ENDPOINT = 0x86;
    private const int _CUSTOM_HID_INCOMING_ENDPOINT = 0x85;
    private const int _CUSTOM_MSD_INCOMING_ENDPOINT = 0x83;
    private const int _CUSTOM_OUTGOING_ENDPOINT = 0x02;
    private const int _CUSTOM_CHUNK_READ_LENGTH = 0x800;
    private const string _CUSTOM_GUID = "32F89CE2-3747-4AF0-93E4-2D165A4E52E8";
    private const int _PACKET_HEADER_LENGTH = 7;
    private HidDevice _hidDevice1 = null;
    private HidDevice _hidDevice2 = null;
    private WinUSBDevice _winUsbDevice;
    private IEnumerable<byte> _bytesReceivedSoFar;

    public OperatingMode Mode { get; private set; }

    public AdditionalCustomInterface Configurations
    {
      get
      {
        if (!_configs.HasValue)
        {
          _GetStatus();
        }

        return _configs.GetValueOrDefault();
      }
    }

    public enum OperatingMode
    {
      HID = 0x01,
      Custom = 0x02
    }

    public enum AdditionalCustomInterface
    {
      None = 0x00,
      HumanInterfaceDevice = 0x80,
      MassStorageDevice = 0x40
    }

    public static Dictionary<OperatingMode, List<string>> GetDevicePaths()
    {
      var ret = new Dictionary<OperatingMode, List<string>>();

      //First get all the HID devices
      var hidPaths = HidDevice.GetDevicePaths(_HID_VENDOR_ID, _HID_PRODUCT_ID);

      //Now get all the WinUSB ones
      var winUsbPaths = NativeInterface.GetAllDevicePaths(new Guid(_CUSTOM_GUID));

      if (winUsbPaths.Any())
      {
        ret.Add(OperatingMode.Custom, new List<string>());
        foreach (var path in winUsbPaths)
        {
          ret[OperatingMode.Custom].Add(path);
        }
      }

      if (hidPaths.Any())
      {
        ret.Add(OperatingMode.HID, new List<string>());
        foreach (var path in hidPaths)
        {
          ret[OperatingMode.HID].Add(path);
        }
      }

      return ret;
    }

    public void Dispose()
    {
      Dispose(true);
      GC.SuppressFinalize(this);
    }

    protected virtual void Dispose(bool disposing)
    {
      if (disposing)
      {
        // Clean up managed resources
        lock (this)
        {
          if (_winUsbDevice != null)
          {
            _winUsbDevice.Dispose();
          }
        }
      }
    }

    private void _Init(string[] devicePaths)
    {
      if (Mode == OperatingMode.HID && devicePaths.Length >= 2)
      {
        _hidDevice1 = new HidDevice(devicePaths.ElementAt(0));
        _hidDevice2 = new HidDevice(devicePaths.ElementAt(1));
      }
      else if (devicePaths.Length > 0)
      {
        _winUsbDevice = new WinUSBDevice(devicePaths.First());
      }
    }

    private void _Open()
    {
      if (Mode == OperatingMode.HID)
      {
        _hidDevice1.Open();
        _hidDevice2.Open();
      }
      else
      {
        _winUsbDevice.Open();
      }
    }

    private void _Close()
    {
      if (_hidDevice1 != null)
      {
        _hidDevice1.Close();
        _hidDevice1 = null;
      }

      if (_hidDevice2 != null)
      {
        _hidDevice2.Close();
        _hidDevice2 = null;
      }

      if (_winUsbDevice != null)
      {
        _winUsbDevice.Dispose();
        _winUsbDevice = null;
      }
    }

    private void _DoRead()
    {
      if (Mode == OperatingMode.HID)
      {
        _DoHidRead();
      }
      else
      {
        _DoWinUSBRead();
      }
    }

    private void _DoHidRead()
    {
      var data = _ReadViaHid();
      if (data != null)
      {
        _bytesReceivedSoFar = _bytesReceivedSoFar.Concat(data);
      }
    }

    private byte[] _ReadViaHid()
    {
      HidReadResult result = null;
      byte[] ret = null;

      try
      {
        _deviceMutex.WaitOne();

        if (_hidDevice1 != null)
        {
          result = _hidDevice1.Read(1);
        }
      }
      finally
      {
        _deviceMutex.ReleaseMutex();
      }

      if (result != null && result.Status == HidReadResult.ReadStatus.Success)
      {
        //Remove the report ID
        ret = result.Data.Skip(1).ToArray();
      }

      return ret;
    }

    private void _DoWinUSBRead()
    {
      byte[] data;

      try
      {
        _deviceMutex.WaitOne();
        data = _ReadViaWinUSB(_CUSTOM_INCOMING_CMD_ENDPOINT);
      }
      finally
      {
        _deviceMutex.ReleaseMutex();
      }

      if (data != null)
      {
        _bytesReceivedSoFar = _bytesReceivedSoFar.Concat(data);
      }
    }

    private byte[] _ReadViaWinUSB(byte pipeId)
    {
      uint bytesRead;
      byte[] ret = null;

      if (_winUsbDevice != null)
      {
        ret = new byte[_CUSTOM_CHUNK_READ_LENGTH];
        if (_winUsbDevice.ReadPipe(_CUSTOM_INTERFACE_ID, pipeId,
          ret, 0, ret.Length, out bytesRead))
        {
          ret = ret.Take((int)bytesRead).ToArray();
        }
      }

      return ret;
    }

    private bool _EnoughForPacketHeader()
    {
      return _bytesReceivedSoFar.Count() >= _PACKET_HEADER_LENGTH;
    }

    private bool _EnoughForWholePacket(int length)
    {
      return _bytesReceivedSoFar.Count() >= (_PACKET_HEADER_LENGTH + length);
    }

    private void _WriteData(byte[] data)
    {
      if (Mode == OperatingMode.HID)
      {
        _hidDevice1.Write(_HID_REPORT_ID, data);
      }
      else
      {
        _winUsbDevice.WritePipe(_CUSTOM_INTERFACE_ID, _CUSTOM_OUTGOING_ENDPOINT, data, 0, data.Length);
      }
    }
  }
}
