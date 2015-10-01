using System;
using Microsoft.Win32.SafeHandles;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using System.Threading;
using System.ComponentModel;

namespace Vicar.WinUSBInterface
{
  partial class WinUSBDevice : IDisposable
  {
    private IntPtr _winUsbHandle = IntPtr.Zero;
    private string _devicePath;
    private SafeFileHandle _deviceHandle;
    private List<IntPtr> _additionalInterfaces;
    private bool _disposed = false;

    public WinUSBDevice(string devicePath)
    {
      _devicePath = devicePath;
      _additionalInterfaces = new List<IntPtr>();
    }

    public void Open()
    {
      try
      {
        _deviceHandle = NativeInterface.CreateFile_SafeFileHandle(_devicePath,
              (NativeInterface.GENERIC_WRITE | NativeInterface.GENERIC_READ),
              NativeInterface.FILE_SHARE_READ | NativeInterface.FILE_SHARE_WRITE,
              IntPtr.Zero,
              NativeInterface.OPEN_EXISTING,
              NativeInterface.FILE_ATTRIBUTE_NORMAL | NativeInterface.FILE_FLAG_OVERLAPPED,
              0);
        if (_deviceHandle.IsInvalid)
        {
          throw new Win32Exception("Failed to open WinUSB device handle.");
        }

        _InitializeDevice();
      }
      catch (Exception)
      {
        if (_deviceHandle != null)
        {
          _deviceHandle.Dispose();
          _deviceHandle = null;
        }

        _FreeWinUSB();
        throw;
      }
    }

    public int InterfaceCount
    {
      get
      {
        return 1 + (_additionalInterfaces == null ? 0 : _additionalInterfaces.Count);
      }
    }

    public void GetInterfaceInformation(int interfaceIndex,
      out NativeInterface.USB_INTERFACE_DESCRIPTOR descriptor, out List<NativeInterface.WINUSB_PIPE_INFORMATION> pipes)
    {
      pipes = new List<NativeInterface.WINUSB_PIPE_INFORMATION>();
      bool success = NativeInterface.WinUsb_QueryInterfaceSettings(_InterfaceHandle(interfaceIndex), 0, out descriptor);
      if (!success)
      {
        throw new Win32Exception("Failed to get WinUSB device interface descriptor.");
      }

      IntPtr interfaceHandle = _InterfaceHandle(interfaceIndex);
      for (byte pipeIdx = 0; pipeIdx < descriptor.bNumEndpoints; pipeIdx++)
      {
        NativeInterface.WINUSB_PIPE_INFORMATION pipeInfo;
        success = NativeInterface.WinUsb_QueryPipe(interfaceHandle, 0, pipeIdx, out pipeInfo);

        pipes.Add(pipeInfo);
        if (!success)
        {
          throw new Win32Exception("Failed to get WinUSB device pipe information.");
        }
      }
    }

    public NativeInterface.USB_DEVICE_DESCRIPTOR GetDeviceDescriptor()
    {
      NativeInterface.USB_DEVICE_DESCRIPTOR ret;
      uint transfered;
      uint size = (uint)Marshal.SizeOf(typeof(NativeInterface.USB_DEVICE_DESCRIPTOR));
      bool success = NativeInterface.WinUsb_GetDescriptor(_winUsbHandle, NativeInterface.USB_DEVICE_DESCRIPTOR_TYPE,
                  0, 0, out ret, size, out transfered);
      if (!success)
      {
        throw new Win32Exception("Failed to get USB device descriptor.");
      }

      if (transfered != size)
      {
        throw new Win32Exception("Incomplete USB device descriptor.");
      }

      return ret;
    }

    public string GetStringDescriptor(byte index)
    {
      byte[] buffer = new byte[256];
      uint transfered;
      bool success = NativeInterface.WinUsb_GetDescriptor(_winUsbHandle, NativeInterface.USB_STRING_DESCRIPTOR_TYPE,
                  index, 0, buffer, (uint)buffer.Length, out transfered);
      if (!success)
      {
        throw new Win32Exception(string.Format("Failed to get USB string descriptor ({0})", index));
      }

      int length = buffer[0] - 2;
      if (length <= 0)
      {
        return null;
      }

      char[] chars = System.Text.Encoding.Unicode.GetChars(buffer, 2, length);
      return new string(chars);
    }

    public bool ReadPipe(int ifaceIndex, byte pipeID, byte[] buffer, int offset,
      int bytesToRead, out uint bytesRead)
    {
      bool success;
      unsafe
      {
        fixed (byte* pBuffer = buffer)
        {
          success = NativeInterface.WinUsb_ReadPipe(_InterfaceHandle(ifaceIndex),
            pipeID, pBuffer + offset, (uint)bytesToRead, out bytesRead, IntPtr.Zero);
        }
      }

      return success;
    }

    public void ReadPipeOverlapped(int ifaceIndex, byte pipeID, byte[] buffer,
      int offset, int bytesToRead, WinUSBAsyncResult result)
    {
      Overlapped overlapped = new Overlapped();
      overlapped.AsyncResult = result;

      unsafe
      {
        NativeOverlapped* pOverlapped = null;
        uint bytesRead;

        pOverlapped = overlapped.Pack(_PipeIOCallback, buffer);
        bool success;

        // Buffer is pinned already by overlapped.Pack
        fixed (byte* pBuffer = buffer)
        {
          success = NativeInterface.WinUsb_ReadPipe(_InterfaceHandle(ifaceIndex), pipeID,
            pBuffer + offset, (uint)bytesToRead, out bytesRead, pOverlapped);
        }

        _HandleOverlappedAPI(success, "Failed to asynchronously read pipe on WinUSB device.",
          pOverlapped, result, (int)bytesRead);
      }
    }

    public bool WritePipe(int ifaceIndex, byte pipeID, byte[] buffer, int offset, int length)
    {
      uint bytesWritten;
      bool success;
      unsafe
      {
        fixed (byte* pBuffer = buffer)
        {
          success = NativeInterface.WinUsb_WritePipe(_InterfaceHandle(ifaceIndex), pipeID, pBuffer + offset, (uint)length,
                  out bytesWritten, IntPtr.Zero);
        }
      }

      return success;
    }

    public void WritePipeOverlapped(int ifaceIndex, byte pipeID, byte[] buffer,
      int offset, int bytesToWrite, WinUSBAsyncResult result)
    {
      Overlapped overlapped = new Overlapped();
      overlapped.AsyncResult = result;

      unsafe
      {
        NativeOverlapped* pOverlapped = null;

        uint bytesWritten;
        pOverlapped = overlapped.Pack(_PipeIOCallback, buffer);

        bool success;
        // Buffer is pinned already by overlapped.Pack
        fixed (byte* pBuffer = buffer)
        {
          success = NativeInterface.WinUsb_WritePipe(_InterfaceHandle(ifaceIndex), pipeID,
            pBuffer + offset, (uint)bytesToWrite, out bytesWritten, pOverlapped);
        }

        _HandleOverlappedAPI(success, "Failed to asynchronously write pipe on WinUSB device.",
          pOverlapped, result, (int)bytesWritten);
      }
    }

    public bool ControlTransfer(byte requestType, byte request, ushort value, ushort index, ushort length, byte[] data)
    {
      uint bytesReturned = 0;
      NativeInterface.WINUSB_SETUP_PACKET setupPacket;

      setupPacket.RequestType = requestType;
      setupPacket.Request = request;
      setupPacket.Value = value;
      setupPacket.Index = index;
      setupPacket.Length = length;

      return NativeInterface.WinUsb_ControlTransfer(_winUsbHandle, setupPacket,
        data, length, ref bytesReturned, IntPtr.Zero);
    }

    public void ControlTransferOverlapped(byte requestType, byte request, ushort value,
      ushort index, ushort length, byte[] data, WinUSBAsyncResult result)
    {
      uint bytesReturned = 0;
      NativeInterface.WINUSB_SETUP_PACKET setupPacket;

      setupPacket.RequestType = requestType;
      setupPacket.Request = request;
      setupPacket.Value = value;
      setupPacket.Index = index;
      setupPacket.Length = length;

      Overlapped overlapped = new Overlapped();
      overlapped.AsyncResult = result;

      unsafe
      {
        NativeOverlapped* pOverlapped = null;
        pOverlapped = overlapped.Pack(_PipeIOCallback, data);
        bool success = NativeInterface.WinUsb_ControlTransfer(_winUsbHandle, setupPacket,
          data, length, ref bytesReturned, pOverlapped);
        _HandleOverlappedAPI(success, "Asynchronous control transfer on WinUSB device failed.",
          pOverlapped, result, (int)bytesReturned);
      }
    }

    public void AbortPipe(int ifaceIndex, byte pipeID)
    {
      bool success = NativeInterface.WinUsb_AbortPipe(_InterfaceHandle(ifaceIndex), pipeID);
      if (!success)
      {
        throw new Win32Exception("Failed to abort pipe on WinUSB device.");
      }
    }

    public void FlushPipe(int ifaceIndex, byte pipeID)
    {
      bool success = NativeInterface.WinUsb_FlushPipe(_InterfaceHandle(ifaceIndex), pipeID);
      if (!success)
      {
        throw new Win32Exception("Failed to flush pipe on WinUSB device.");
      }
    }

    public void SetPipePolicy(int ifaceIndex, byte pipeID, NativeInterface.POLICY_TYPE policyType, bool value)
    {
      byte byteVal = (byte)(value ? 1 : 0);
      bool success = NativeInterface.WinUsb_SetPipePolicy(_InterfaceHandle(ifaceIndex), pipeID, (uint)policyType, 1, ref byteVal);
      if (!success)
      {
        throw new Win32Exception("Failed to set WinUSB pipe policy.");
      }
    }

    public void SetPipePolicy(int ifaceIndex, byte pipeID, NativeInterface.POLICY_TYPE policyType, uint value)
    {
      bool success = NativeInterface.WinUsb_SetPipePolicy(_InterfaceHandle(ifaceIndex), pipeID, (uint)policyType, 4, ref value);

      if (!success)
      {
        throw new Win32Exception("Failed to set WinUSB pipe policy.");
      }
    }

    public bool GetPipePolicyBool(int ifaceIndex, byte pipeID, NativeInterface.POLICY_TYPE policyType)
    {
      byte result;
      uint length = 1;

      bool success = NativeInterface.WinUsb_GetPipePolicy(_InterfaceHandle(ifaceIndex), pipeID,
        (uint)policyType, ref length, out result);
      if (!success || length != 1)
      {
        throw new Win32Exception("Failed to get WinUSB pipe policy.");
      }

      return result != 0;
    }

    public uint GetPipePolicyUInt(int ifaceIndex, byte pipeID, NativeInterface.POLICY_TYPE policyType)
    {
      uint result;
      uint length = 4;
      bool success = NativeInterface.WinUsb_GetPipePolicy(_InterfaceHandle(ifaceIndex), pipeID,
        (uint)policyType, ref length, out result);

      if (!success || length != 4)
      {
        throw new Win32Exception("Failed to get WinUSB pipe policy.");
      }

      return result;
    }

    ~WinUSBDevice()
    {
      Dispose(false);
    }

    public void Dispose()
    {
      Dispose(true);
      GC.SuppressFinalize(this);
    }

    protected virtual void Dispose(bool disposing)
    {
      if (!_disposed)
      {
        if (disposing)
        {
          // Dispose managed resources
          if (_deviceHandle != null && !_deviceHandle.IsInvalid)
          {
            _deviceHandle.Dispose();
          }

          _deviceHandle = null;
        }

        // Dispose unmanaged resources
        _FreeWinUSB();
        _disposed = true;
      }
    }

    private void _FreeWinUSB()
    {
      if (_additionalInterfaces != null)
      {
        for (int i = 0; i < _additionalInterfaces.Count; i++)
        {
          NativeInterface.WinUsb_Free(_additionalInterfaces[i]);
        }

        _additionalInterfaces = null;
      }

      if (_winUsbHandle != IntPtr.Zero)
      {
        NativeInterface.WinUsb_Free(_winUsbHandle);
      }

      _winUsbHandle = IntPtr.Zero;
    }

    private IntPtr _InterfaceHandle(int index)
    {
      if (index == 0)
      {
        return _winUsbHandle;
      }

      return _additionalInterfaces[index - 1];
    }

    private void _InitializeDevice()
    {
      _additionalInterfaces = new List<IntPtr>();
      byte numAddInterfaces = 0;
      byte idx = 0;
      bool success = NativeInterface.WinUsb_Initialize(_deviceHandle, ref _winUsbHandle);
      if (!success)
      {
        throw new Win32Exception("Failed to initialize WinUSB handle. Device might not be connected.");
      }

      try
      {
        while (true)
        {
          IntPtr ifaceHandle = IntPtr.Zero;
          success = NativeInterface.WinUsb_GetAssociatedInterface(_winUsbHandle, idx, out ifaceHandle);
          if (!success)
          {
            if (Marshal.GetLastWin32Error() == NativeInterface.ERROR_NO_MORE_ITEMS)
            {
              break;
            }

            throw new Win32Exception("Failed to enumerate interfaces for WinUSB device.");
          }

          _additionalInterfaces.Add(ifaceHandle);
          idx++;
          numAddInterfaces++;
        }
      }
      finally
      {
        //Do nothing
      }

      // Bind handle (needed for overlapped I/O thread pool)
      ThreadPool.BindHandle(_deviceHandle);
    }

    private unsafe void _HandleOverlappedAPI(bool success, string errorMessage, NativeOverlapped* pOverlapped,
      WinUSBAsyncResult result, int bytesTransferred)
    {
      if (!success)
      {
        if (Marshal.GetLastWin32Error() != NativeInterface.ERROR_IO_PENDING)
        {
          Overlapped.Unpack(pOverlapped);
          Overlapped.Free(pOverlapped);

          throw new Win32Exception(errorMessage);
        }
      }
      else
      {
        // Immediate success!
        Overlapped.Unpack(pOverlapped);
        Overlapped.Free(pOverlapped);

        result.OnCompletion(true, null, bytesTransferred, false);
      }
    }

    private unsafe void _PipeIOCallback(uint errorCode, uint numBytes, NativeOverlapped* pOverlapped)
    {
      try
      {
        Exception error = null;
        if (errorCode != 0)
        {
          error = new Win32Exception((int)errorCode, "Asynchronous operation on WinUSB device failed.");
        }

        Overlapped overlapped = Overlapped.Unpack(pOverlapped);
        WinUSBAsyncResult result = (WinUSBAsyncResult)overlapped.AsyncResult;
        Overlapped.Free(pOverlapped);
        pOverlapped = null;

        result.OnCompletion(false, error, (int)numBytes, true);
      }
      finally
      {
        if (pOverlapped != null)
        {
          Overlapped.Unpack(pOverlapped);
          Overlapped.Free(pOverlapped);
        }
      }
    }
  }
}
