using Microsoft.Win32.SafeHandles;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace Vicar.HIDInterface
{
  public class HidDevice
  {
    private class AsyncState
    {
      internal AsyncState(object callerDelegate, object callbackDelegate)
      {
        CallerDelegate = callerDelegate;
        CallbackDelegate = callbackDelegate;
      }

      internal object CallerDelegate { get; private set; }
      
      internal object CallbackDelegate { get; private set; }
    }

    private string _devicePath;
    private IntPtr _handle;
    private NativeInterface.HIDP_CAPS _capabilities;
    private NativeOverlapped _overlapped;
    private GCHandle _pinnedOverlapped;
    private bool _readPending = false;
    private IntPtr _readBufferPtr = IntPtr.Zero;
    private byte[] _buffer;
    private GCHandle _pinnedBuffer;

    public delegate void ReadCallback(HidReadResult result);
    public delegate void WriteCallback(bool success);

    private delegate HidReadResult _ReadDelegate(int timeout);
    private delegate bool _WriteDelegate(byte reportId, byte[] data, int timeout);

    public HidDevice(string devicePath)
    {
      _devicePath = devicePath;

      _overlapped = new NativeOverlapped();
      _overlapped.EventHandle = NativeInterface.CreateEvent(IntPtr.Zero, false, false, null);
      _pinnedOverlapped = GCHandle.Alloc(_overlapped, GCHandleType.Pinned);
    }

    public static IEnumerable<string> GetDevicePaths(int vendorId, params int[] productIds)
    {
      var ret = new List<string>();

      foreach (var device in _GetAllDevicePaths())
      {
        /* Check this device path for our vendor/product ID(s) */
        try
        {
          int vid = 0;
          int pid = 0;
          var handle = _OpenDevice(device, true);
          _GetDeviceAttributes(handle, out vid, out pid);
          _CloseDevice(handle);

          if (vid == vendorId && productIds.Contains(pid))
          {
            //Found one
            ret.Add(device);
          }
        }
        catch
        {
          //Oh well, we tried...
        }
      }

      return ret;
    }

    public void Open()
    {
      _handle = _OpenDevice(_devicePath, false);
      _capabilities = _GetDeviceCapabilities(_handle);

      _readBufferPtr = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(byte)) *
        Math.Max(_capabilities.FeatureReportByteLength, _capabilities.InputReportByteLength));
    }

    public HidReadResult GetFeature(int reportId)
    {
      var status = HidReadResult.ReadStatus.NoDataRead;
      byte[] buffer = null;

      Marshal.Copy(new byte[] { (byte)reportId }, 0, _readBufferPtr, 1);
      if (NativeInterface.HidD_GetFeature(_handle, _readBufferPtr, _capabilities.FeatureReportByteLength))
      {
        status = HidReadResult.ReadStatus.Success;
        buffer = new byte[_capabilities.FeatureReportByteLength];
        Marshal.Copy(_readBufferPtr, buffer, 0, buffer.Length);
      }

      return new HidReadResult(buffer != null ? buffer.Skip(1).ToArray() : null, status);
    }

    public HidReadResult Read()
    {
      return Read(0);
    }

    public void Read(ReadCallback callback)
    {
      Read(callback, 0);
    }

    public void Read(ReadCallback callback, int timeout)
    {
      var d = new _ReadDelegate(Read);
      var asyncState = new AsyncState(d, callback);

      d.BeginInvoke(timeout, _EndRead, asyncState);
    }

    public HidReadResult Read(int timeout)
    {
      var overlapTimeout = timeout <= 0 ? NativeInterface.WAIT_INFINITE : timeout;
      var status = HidReadResult.ReadStatus.NoDataRead;
      uint bytesRead = 0;

      if (!_readPending)
      {
        _readPending = true;
        _buffer = new byte[_capabilities.InputReportByteLength];
        _pinnedBuffer = GCHandle.Alloc(_buffer, GCHandleType.Pinned);
        NativeInterface.ResetEvent(_overlapped.EventHandle);

        if (!NativeInterface.ReadFile(_handle, _buffer, (uint)_buffer.Length, out bytesRead, ref _overlapped))
        {
          var err = Marshal.GetLastWin32Error();
          if (err != NativeInterface.ERROR_IO_PENDING)
          {
            throw new IOException(string.Format("Unable to read from device ({0})", err.ToString("X04")));
          }
        }
      }

      bool getResult = true;
      if (overlapTimeout != NativeInterface.WAIT_INFINITE)
      {
        getResult = false;
        var result = NativeInterface.WaitForSingleObject(_overlapped.EventHandle, overlapTimeout);
        switch (result)
        {
          case NativeInterface.WAIT_OBJECT_0:
            {
              status = HidReadResult.ReadStatus.Success;
              getResult = true;
              break;
            }

          case NativeInterface.WAIT_TIMEOUT:
            {
              status = HidReadResult.ReadStatus.WaitTimedOut;
              break;
            }

          case NativeInterface.WAIT_FAILED:
            {
              status = HidReadResult.ReadStatus.WaitFail;
              break;
            }

          default:
            {
              status = HidReadResult.ReadStatus.NoDataRead;
              break;
            }
        }
      }

      if (getResult)
      {
        var ret = NativeInterface.GetOverlappedResult(_handle, ref _overlapped, out bytesRead, true);
        if (ret && bytesRead > 0)
        {
          _readPending = false;
          status = HidReadResult.ReadStatus.Success;
          _pinnedBuffer.Free();
        }
        else
        {
          status = HidReadResult.ReadStatus.NoDataRead;
        }
      }

      return new HidReadResult(_buffer != null ? _buffer : null, status);
    }

    public async Task<HidReadResult> ReadAsync(int timeout = 0)
    {
      var d = new _ReadDelegate(Read);

      return await Task<HidReadResult>.Factory.FromAsync(d.BeginInvoke, d.EndInvoke, timeout, null);
    }

    public bool Write(byte reportId, byte[] data)
    {
      return Write(reportId, data, 0);
    }

    public void Write(byte reportId, byte[] data, WriteCallback callback)
    {
      Write(reportId, data, callback, 0);
    }

    public void Write(byte reportId, byte[] data, WriteCallback callback, int timeout)
    {
      var d = new _WriteDelegate(Write);
      var asyncState = new AsyncState(d, callback);

      d.BeginInvoke(reportId, data, timeout, _EndWrite, asyncState);
    }

    public async Task<bool> WriteAsync(byte reportId, byte[] data, int timeout = 0)
    {
      var d = new _WriteDelegate(Write);

      return await Task<bool>.Factory.FromAsync(d.BeginInvoke, d.EndInvoke, reportId, data, timeout, null);
    }

    public bool Write(byte reportId, byte[] data, int timeout)
    {
      bool ret = false;

      if (_capabilities.OutputReportByteLength > 0)
      {
        var overlapTimeout = timeout <= 0 ? NativeInterface.WAIT_INFINITE : timeout;
        var overlapped = new NativeOverlapped();
        var buffer = new byte[_capabilities.OutputReportByteLength];
        uint bytesWritten = 0;

        Array.Copy(data, 0, buffer, 1, Math.Min(data.Length, _capabilities.OutputReportByteLength));
        buffer[0] = reportId;

        try
        {
          NativeInterface.WriteFile(_handle, buffer, (uint)buffer.Length, out bytesWritten, ref overlapped);
          ret = NativeInterface.GetOverlappedResult(_handle, ref overlapped, out bytesWritten, true);
        }
        catch
        {
          ret = false;
        }
      }

      return ret;
    }

    public void Close()
    {
      _CloseDevice(_overlapped.EventHandle);
      _pinnedOverlapped.Free();
      _CloseDevice(_handle);

      Marshal.FreeHGlobal(_readBufferPtr);
    }

    private static void _EndRead(IAsyncResult ar)
    {
      var hidAsyncState = (AsyncState)ar.AsyncState;
      var callerDelegate = (_ReadDelegate)hidAsyncState.CallerDelegate;
      var callbackDelegate = (ReadCallback)hidAsyncState.CallbackDelegate;
      var data = callerDelegate.EndInvoke(ar);

      if ((callbackDelegate != null))
      {
        callbackDelegate.Invoke(data);
      }
    }

    private void _EndWrite(IAsyncResult ar)
    {
      var hidAsyncState = (AsyncState)ar.AsyncState;
      var callerDelegate = (_WriteDelegate)hidAsyncState.CallerDelegate;
      var callbackDelegate = (WriteCallback)hidAsyncState.CallbackDelegate;

      var result = callerDelegate.EndInvoke(ar);
      if (callbackDelegate != null)
      {
        callbackDelegate.Invoke(result);
      }
    }

    private static IntPtr _OpenDevice(string devicePath, bool enumerate)
    {
      return NativeInterface.CreateFile_IntPtr(devicePath, enumerate ? 0 :
        NativeInterface.GENERIC_READ | NativeInterface.GENERIC_WRITE,
        enumerate ? NativeInterface.FILE_SHARE_READ | NativeInterface.FILE_SHARE_WRITE :
        NativeInterface.FILE_SHARE_READ, IntPtr.Zero, NativeInterface.OPEN_EXISTING,
        NativeInterface.FILE_FLAG_OVERLAPPED, 0);
    }

    private static void _CloseDevice(IntPtr handle)
    {
      if (Environment.OSVersion.Version.Major > 5)
      {
        NativeInterface.CancelIoEx(handle, IntPtr.Zero);
      }

      NativeInterface.CloseHandle(handle);
    }

    private static IEnumerable<string> _GetAllDevicePaths()
    {
      var guid = Guid.Empty;
      NativeInterface.HidD_GetHidGuid(ref guid);

      return NativeInterface.GetAllDevicePaths(guid);
    }

    private static void _GetDeviceAttributes(IntPtr hidHandle, out int vendorId, out int productId)
    {
      var deviceAttributes = default(NativeInterface.HIDD_ATTRIBUTES);
      deviceAttributes.Size = Marshal.SizeOf(deviceAttributes);
      NativeInterface.HidD_GetAttributes(hidHandle, ref deviceAttributes);
      vendorId = deviceAttributes.VendorID;
      productId = deviceAttributes.ProductID;
    }

    private static NativeInterface.HIDP_CAPS _GetDeviceCapabilities(IntPtr hidHandle)
    {
      var capabilities = default(NativeInterface.HIDP_CAPS);
      var preparsedDataPointer = default(IntPtr);

      if (NativeInterface.HidD_GetPreparsedData(hidHandle, ref preparsedDataPointer))
      {
        NativeInterface.HidP_GetCaps(preparsedDataPointer, ref capabilities);
        NativeInterface.HidD_FreePreparsedData(preparsedDataPointer);
      }

      return capabilities;
    }
  }
}
