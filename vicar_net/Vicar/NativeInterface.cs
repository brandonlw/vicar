using Microsoft.Win32.SafeHandles;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace Vicar
{
  public static class NativeInterface
  {
    public static readonly IntPtr INVALID_HANDLE_PTR = new IntPtr(-1);

    public const int FILE_ATTRIBUTE_NORMAL = 0x00000080;
    public const int FILE_FLAG_OVERLAPPED = 0x40000000;
    public const short FILE_SHARE_READ = 0x1;
    public const short FILE_SHARE_WRITE = 0x2;
    public const uint GENERIC_READ = 0x80000000;
    public const uint GENERIC_WRITE = 0x40000000;
    public const int ACCESS_NONE = 0;
    public const int INVALID_HANDLE_VALUE = -1;
    public const short OPEN_EXISTING = 3;
    public const int WAIT_TIMEOUT = 0x102;
    public const uint WAIT_OBJECT_0 = 0;
    public const uint WAIT_FAILED = 0xffffffff;
    public const int WAIT_INFINITE = 0xffff;
    public const int DBT_DEVICEARRIVAL = 0x8000;
    public const int DBT_DEVICEREMOVECOMPLETE = 0x8004;
    public const int DBT_DEVTYP_DEVICEINTERFACE = 5;
    public const int DBT_DEVTYP_HANDLE = 6;
    public const int DEVICE_NOTIFY_ALL_INTERFACE_CLASSES = 4;
    public const int DEVICE_NOTIFY_SERVICE_HANDLE = 1;
    public const int DEVICE_NOTIFY_WINDOW_HANDLE = 0;
    public const int WM_DEVICECHANGE = 0x219;
    public const short DIGCF_PRESENT = 0x2;
    public const short DIGCF_DEVICEINTERFACE = 0x10;
    public const int DIGCF_ALLCLASSES = 0x4;
    public const int MAX_DEV_LEN = 1000;
    public const int SPDRP_ADDRESS = 0x1c;
    public const int SPDRP_BUSNUMBER = 0x15;
    public const int SPDRP_BUSTYPEGUID = 0x13;
    public const int SPDRP_CAPABILITIES = 0xf;
    public const int SPDRP_CHARACTERISTICS = 0x1b;
    public const int SPDRP_CLASS = 7;
    public const int SPDRP_CLASSGUID = 8;
    public const int SPDRP_COMPATIBLEIDS = 2;
    public const int SPDRP_CONFIGFLAGS = 0xa;
    public const int SPDRP_DEVICE_POWER_DATA = 0x1e;
    public const int SPDRP_DEVICEDESC = 0;
    public const int SPDRP_DEVTYPE = 0x19;
    public const int SPDRP_DRIVER = 9;
    public const int SPDRP_ENUMERATOR_NAME = 0x16;
    public const int SPDRP_EXCLUSIVE = 0x1a;
    public const int SPDRP_FRIENDLYNAME = 0xc;
    public const int SPDRP_HARDWAREID = 1;
    public const int SPDRP_LEGACYBUSTYPE = 0x14;
    public const int SPDRP_LOCATION_INFORMATION = 0xd;
    public const int SPDRP_LOWERFILTERS = 0x12;
    public const int SPDRP_MFG = 0xb;
    public const int SPDRP_PHYSICAL_DEVICE_OBJECT_NAME = 0xe;
    public const int SPDRP_REMOVAL_POLICY = 0x1f;
    public const int SPDRP_REMOVAL_POLICY_HW_DEFAULT = 0x20;
    public const int SPDRP_REMOVAL_POLICY_OVERRIDE = 0x21;
    public const int SPDRP_SECURITY = 0x17;
    public const int SPDRP_SECURITY_SDS = 0x18;
    public const int SPDRP_SERVICE = 4;
    public const int SPDRP_UI_NUMBER = 0x10;
    public const int SPDRP_UI_NUMBER_DESC_FORMAT = 0x1d;
    public const int SPDRP_UPPERFILTERS = 0x11;
    public const int ERROR_IO_PENDING = 997;
    public const int ERROR_NO_MORE_ITEMS = 259;
    public const int ERROR_INSUFFICIENT_BUFFER = 122;
    public const int USB_DEVICE_DESCRIPTOR_TYPE = 0x01;
    public const int USB_CONFIGURATION_DESCRIPTOR_TYPE = 0x02;
    public const int USB_STRING_DESCRIPTOR_TYPE = 0x03;

    [StructLayout(LayoutKind.Sequential)]
    public struct USB_DEVICE_DESCRIPTOR
    {
      public byte bLength;
      public byte bDescriptorType;
      public ushort bcdUSB;
      public byte bDeviceClass;
      public byte bDeviceSubClass;
      public byte bDeviceProtocol;
      public byte bMaxPacketSize0;
      public ushort idVendor;
      public ushort idProduct;
      public ushort bcdDevice;
      public byte iManufacturer;
      public byte iProduct;
      public byte iSerialNumber;
      public byte bNumConfigurations;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct USB_CONFIGURATION_DESCRIPTOR
    {
      public byte bLength;
      public byte bDescriptorType;
      public ushort wTotalLength;
      public byte bNumInterfaces;
      public byte bConfigurationValue;
      public byte iConfiguration;
      public byte bmAttributes;
      public byte MaxPower;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct USB_INTERFACE_DESCRIPTOR
    {
      public byte bLength;
      public byte bDescriptorType;
      public byte bInterfaceNumber;
      public byte bAlternateSetting;
      public byte bNumEndpoints;
      public byte bInterfaceClass;
      public byte bInterfaceSubClass;
      public byte bInterfaceProtocol;
      public byte iInterface;
    };

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct WINUSB_SETUP_PACKET
    {
      public byte RequestType;
      public byte Request;
      public ushort Value;
      public ushort Index;
      public ushort Length;
    }

    public enum USBD_PIPE_TYPE : int
    {
      UsbdPipeTypeControl,
      UsbdPipeTypeIsochronous,
      UsbdPipeTypeBulk,
      UsbdPipeTypeInterrupt,
    }

    public enum POLICY_TYPE : int
    {
      SHORT_PACKET_TERMINATE = 1,
      AUTO_CLEAR_STALL,
      PIPE_TRANSFER_TIMEOUT,
      IGNORE_SHORT_PACKETS,
      ALLOW_PARTIAL_READS,
      AUTO_FLUSH,
      RAW_IO,
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WINUSB_PIPE_INFORMATION
    {
      public USBD_PIPE_TYPE PipeType;
      public byte PipeId;
      public ushort MaximumPacketSize;
      public byte Interval;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct HIDP_CAPS
    {
      internal short Usage;
      internal short UsagePage;
      internal short InputReportByteLength;
      internal short OutputReportByteLength;
      internal short FeatureReportByteLength;
      [MarshalAs(UnmanagedType.ByValArray, SizeConst = 17)]
      internal short[] Reserved;
      internal short NumberLinkCollectionNodes;
      internal short NumberInputButtonCaps;
      internal short NumberInputValueCaps;
      internal short NumberInputDataIndices;
      internal short NumberOutputButtonCaps;
      internal short NumberOutputValueCaps;
      internal short NumberOutputDataIndices;
      internal short NumberFeatureButtonCaps;
      internal short NumberFeatureValueCaps;
      internal short NumberFeatureDataIndices;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct DeviceInterfaceData
    {
      public int Size;
      public Guid InterfaceClassGuid;
      public int Flags;
      public int Reserved;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct DeviceInterfaceDetailData
    {
      public int Size;
      [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
      public string DevicePath;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SP_DEVINFO_DATA
    {
      internal int cbSize;
      internal Guid ClassGuid;
      internal int DevInst;
      internal IntPtr Reserved;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SP_DEVICE_INTERFACE_DATA
    {
      internal int cbSize;
      internal System.Guid InterfaceClassGuid;
      internal int Flags;
      internal IntPtr Reserved;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SECURITY_ATTRIBUTES
    {
      public int nLength;
      public IntPtr lpSecurityDescriptor;
      public bool bInheritHandle;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct DEVPROPKEY
    {
      public Guid fmtid;
      public ulong pid;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Auto)]
    public struct SP_DEVICE_INTERFACE_DETAIL_DATA
    {
      internal int Size;
      [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
      internal string DevicePath;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct HIDD_ATTRIBUTES
    {
      internal int Size;
      internal ushort VendorID;
      internal ushort ProductID;
      internal short VersionNumber;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct OVERLAPPED
    {
      public int Internal;
      public int InternalHigh;
      public int Offset;
      public int OffsetHigh;
      public int hEvent;
    }

    public static DEVPROPKEY DEVPKEY_Device_BusReportedDeviceDesc =
        new DEVPROPKEY { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc,
          0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid = 4 };

    [DllImport("setupapi.dll", SetLastError = true, CharSet = CharSet.Auto)]
    static extern bool SetupDiGetDeviceInstanceId(
       IntPtr DeviceInfoSet,
       ref SP_DEVINFO_DATA DeviceInfoData,
       StringBuilder DeviceInstanceId,
       int DeviceInstanceIdSize,
       out int RequiredSize
    );

    [DllImport("hid.dll", SetLastError = true)]
    public static extern Boolean HidD_GetInputReport(IntPtr HidDeviceObject,
      IntPtr lpReportBuffer, int ReportBufferLength);

    [DllImport("hid.dll", SetLastError = true)]
    public static extern Boolean HidD_GetFeature(IntPtr HidDeviceObject,
      IntPtr lpReportBuffer, int ReportBufferLength);

    [DllImport("hid.dll", SetLastError = true)]
    public static extern Boolean HidD_SetOutputReport(IntPtr HidDeviceObject,
      byte[] lpReportBuffer, int ReportBufferLength);

    [DllImport("hid.dll", SetLastError = true)]
    public static extern Boolean
    HidD_GetInputReport(IntPtr HidDeviceObject, [In] [Out] ref byte[] lpReportBuffer, Int32 ReportBufferLength);
    
    [DllImport("setupapi.dll", SetLastError = true)]
    public static extern bool SetupDiGetDeviceInterfaceDetail(IntPtr lpDeviceInfoSet,
      ref DeviceInterfaceData oInterfaceData, IntPtr lpDeviceInterfaceDetailData,
      uint nDeviceInterfaceDetailDataSize, ref uint nRequiredSize, IntPtr lpDeviceInfoData);

    [DllImport("setupapi.dll", SetLastError = true)]
    public static extern bool SetupDiGetDeviceInterfaceDetail(IntPtr lpDeviceInfoSet,
      ref DeviceInterfaceData oInterfaceData, ref DeviceInterfaceDetailData oDetailData,
      uint nDeviceInterfaceDetailDataSize, ref uint nRequiredSize, IntPtr lpDeviceInfoData);
    
    [DllImport("hid.dll", SetLastError = true)]
    public static extern void HidD_GetHidGuid(ref Guid gHid);

    [DllImport("setupapi.dll", SetLastError = true)]
    public static extern IntPtr SetupDiGetClassDevs(ref Guid gClass,
      [MarshalAs(UnmanagedType.LPStr)] string strEnumerator, IntPtr hParent, uint nFlags);

    [DllImport("setupapi.dll", SetLastError = true)]
    public static extern int SetupDiDestroyDeviceInfoList(IntPtr lpInfoSet);

    [DllImport("setupapi.dll")]
    public static extern bool SetupDiEnumDeviceInterfaces(IntPtr deviceInfoSet,
      ref SP_DEVINFO_DATA deviceInfoData, ref Guid interfaceClassGuid,
      int memberIndex, ref SP_DEVICE_INTERFACE_DATA deviceInterfaceData);

    [DllImport("setupapi.dll")]
    public static extern bool SetupDiEnumDeviceInterfaces(IntPtr deviceInfoSet,
      IntPtr deviceInfoData, ref Guid interfaceClassGuid,
      int memberIndex, ref SP_DEVICE_INTERFACE_DATA deviceInterfaceData);

    [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    public static extern IntPtr CreateFileW(
         [MarshalAs(UnmanagedType.LPWStr)] string filename,
         [MarshalAs(UnmanagedType.U4)] FileAccess access,
         [MarshalAs(UnmanagedType.U4)] FileShare share,
         IntPtr securityAttributes,
         [MarshalAs(UnmanagedType.U4)] FileMode creationDisposition,
         [MarshalAs(UnmanagedType.U4)] FileAttributes flagsAndAttributes,
         IntPtr templateFile);

    [DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
    public static extern IntPtr CreateFile(string lpFileName, uint dwDesiredAccess,
      int dwShareMode, ref SECURITY_ATTRIBUTES lpSecurityAttributes, int dwCreationDisposition,
      int dwFlagsAndAttributes, int hTemplateFile);

    [DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true, EntryPoint="CreateFile")]
    public static extern SafeFileHandle CreateFile_SafeFileHandle(string lpFileName, uint dwDesiredAccess,
      int dwShareMode, IntPtr lpSecurityAttributes, int dwCreationDisposition,
      int dwFlagsAndAttributes, int hTemplateFile);

    [DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true, EntryPoint="CreateFile")]
    public static extern IntPtr CreateFile_IntPtr(string lpFileName, uint dwDesiredAccess,
      int dwShareMode, IntPtr lpSecurityAttributes, int dwCreationDisposition,
      int dwFlagsAndAttributes, int hTemplateFile);

    [DllImport("kernel32.dll")]
    public static extern bool WriteFile(IntPtr hFile, byte[] lpBuffer,
       uint nNumberOfBytesToWrite, out uint lpNumberOfBytesWritten,
       [In] ref System.Threading.NativeOverlapped lpOverlapped);

    [DllImport("kernel32.dll", SetLastError = true)]
    public static extern bool ReadFile(IntPtr hFile, [Out] byte[] lpBuffer,
      uint nNumberOfBytesToRead, out uint lpNumberOfBytesRead,
      [In] ref System.Threading.NativeOverlapped lpOverlapped);

    [DllImport("kernel32.dll", SetLastError = true)]
    public static extern bool GetOverlappedResult(IntPtr hFile,
       [In] ref System.Threading.NativeOverlapped lpOverlapped,
       out uint lpNumberOfBytesTransferred, bool bWait);
    
    [DllImport("kernel32.dll", CharSet = CharSet.Auto)]
    public static extern IntPtr CreateEvent(ref SECURITY_ATTRIBUTES securityAttributes,
      int bManualReset, int bInitialState, string lpName);

    [DllImport("kernel32.dll")]
    public static extern bool ResetEvent(IntPtr hEvent);
    
    [DllImport("kernel32.dll", CharSet = CharSet.Auto)]
    public static extern IntPtr CreateEvent(IntPtr securityAttributes, bool bManualReset,
      bool bInitialState, string lpName);

    [DllImport("kernel32.dll")]
    public static extern uint WaitForSingleObject(IntPtr hHandle, int dwMilliseconds);

    [DllImport("setupapi.dll")]
    public static extern bool SetupDiEnumDeviceInfo(IntPtr deviceInfoSet, int memberIndex,
      ref SP_DEVINFO_DATA deviceInfoData);

    [DllImport("setupapi.dll", EntryPoint = "SetupDiGetDeviceRegistryProperty")]
    public static extern bool SetupDiGetDeviceRegistryProperty(IntPtr deviceInfoSet,
      ref SP_DEVINFO_DATA deviceInfoData, int propertyVal, ref int propertyRegDataType,
      byte[] propertyBuffer, int propertyBufferSize, ref int requiredSize);

    [DllImport("setupapi.dll", EntryPoint = "SetupDiGetDevicePropertyW", SetLastError = true)]
    public static extern bool SetupDiGetDeviceProperty(IntPtr deviceInfo,
      ref SP_DEVINFO_DATA deviceInfoData, ref DEVPROPKEY propkey, ref ulong propertyDataType,
      byte[] propertyBuffer, int propertyBufferSize, ref int requiredSize, uint flags);

    [DllImport("setupapi.dll", CharSet = CharSet.Auto, EntryPoint = "SetupDiGetDeviceInterfaceDetail")]
    public static extern bool SetupDiGetDeviceInterfaceDetailBuffer(IntPtr deviceInfoSet,
      ref SP_DEVICE_INTERFACE_DATA deviceInterfaceData, IntPtr deviceInterfaceDetailData,
      int deviceInterfaceDetailDataSize, ref int requiredSize, IntPtr deviceInfoData);

    [DllImport("setupapi.dll", CharSet = CharSet.Auto)]
    public static extern bool SetupDiGetDeviceInterfaceDetail(IntPtr deviceInfoSet,
      ref SP_DEVICE_INTERFACE_DATA deviceInterfaceData, ref SP_DEVICE_INTERFACE_DETAIL_DATA deviceInterfaceDetailData,
      int deviceInterfaceDetailDataSize, ref int requiredSize, IntPtr deviceInfoData);

    [DllImport("setupapi.dll", CharSet = CharSet.Auto)]
    public static extern bool SetupDiGetDeviceInterfaceDetail(IntPtr deviceInfoSet,
      ref SP_DEVICE_INTERFACE_DATA deviceInterfaceData, IntPtr deviceInterfaceDetailData,
      int deviceInterfaceDetailDataSize, ref int requiredSize, ref SP_DEVINFO_DATA deviceInfoData);

    [DllImport("setupapi.dll", CharSet = CharSet.Auto)]
    public static extern bool SetupDiGetDeviceInterfaceDetail(IntPtr deviceInfoSet,
      ref SP_DEVICE_INTERFACE_DATA deviceInterfaceData, IntPtr deviceInterfaceDetailData,
      int deviceInterfaceDetailDataSize, ref int requiredSize, IntPtr deviceInfoData);

    [DllImport("hid.dll")]
    public static extern bool HidD_GetAttributes(IntPtr hidDeviceObject, ref HIDD_ATTRIBUTES attributes);

    [DllImport("hid.dll")]
    public static extern bool HidD_GetPreparsedData(IntPtr hidDeviceObject, ref IntPtr preparsedData);

    [DllImport("hid.dll")]
    public static extern int HidP_GetCaps(IntPtr preparsedData, ref HIDP_CAPS capabilities);

    [DllImport("hid.dll")]
    public static extern bool HidD_FreePreparsedData(IntPtr preparsedData);

    [DllImport("kernel32.dll", SetLastError = true, ExactSpelling = true, CharSet = CharSet.Auto)]
    public static extern bool CancelIoEx(IntPtr hFile, IntPtr lpOverlapped);

    [DllImport("kernel32.dll", SetLastError = true, ExactSpelling = true, CharSet = CharSet.Auto)]
    public static extern bool CloseHandle(IntPtr hObject);

    [DllImport("setupapi.dll", CharSet = CharSet.Auto)]
    public static extern IntPtr SetupDiGetClassDevs(ref System.Guid classGuid,
      string enumerator, int hwndParent, int flags);

    [DllImport("winusb.dll", SetLastError = true)]
    public static extern bool WinUsb_ControlTransfer(IntPtr InterfaceHandle, WINUSB_SETUP_PACKET SetupPacket,
      Byte[] Buffer, UInt32 BufferLength, ref UInt32 LengthTransferred, IntPtr Overlapped);

    [DllImport("winusb.dll", SetLastError = true)]
    public static unsafe extern bool WinUsb_ControlTransfer(IntPtr InterfaceHandle, WINUSB_SETUP_PACKET SetupPacket,
      Byte[] Buffer, UInt32 BufferLength, ref UInt32 LengthTransferred, NativeOverlapped* pOverlapped);

    [DllImport("winusb.dll", SetLastError = true)]
    public static extern bool WinUsb_Free(IntPtr InterfaceHandle);

    [DllImport("winusb.dll", SetLastError = true)]
    public static extern bool WinUsb_Initialize(SafeFileHandle DeviceHandle, ref IntPtr InterfaceHandle);

    [DllImport("winusb.dll", SetLastError = true)]
    public static extern bool WinUsb_QueryDeviceInformation(IntPtr InterfaceHandle, UInt32 InformationType,
      ref UInt32 BufferLength, out byte Buffer);

    [DllImport("winusb.dll", SetLastError = true)]
    public static extern bool WinUsb_QueryInterfaceSettings(IntPtr InterfaceHandle, Byte AlternateInterfaceNumber,
      out USB_INTERFACE_DESCRIPTOR UsbAltInterfaceDescriptor);

    [DllImport("winusb.dll", SetLastError = true)]
    public static extern bool WinUsb_QueryPipe(IntPtr InterfaceHandle, Byte AlternateInterfaceNumber, Byte PipeIndex,
      out WINUSB_PIPE_INFORMATION PipeInformation);

    [DllImport("winusb.dll", SetLastError = true)]
    public static unsafe extern bool WinUsb_ReadPipe(IntPtr InterfaceHandle, byte PipeID, byte* pBuffer,
      uint BufferLength, out uint LengthTransferred, IntPtr Overlapped);

    [DllImport("winusb.dll", SetLastError = true)]
    public static unsafe extern bool WinUsb_ReadPipe(IntPtr InterfaceHandle, byte PipeID, byte* pBuffer,
      uint BufferLength, out uint LengthTransferred, NativeOverlapped* pOverlapped);

    [DllImport("winusb.dll", SetLastError = true)]
    public static extern bool WinUsb_AbortPipe(IntPtr InterfaceHandle, byte PipeID);

    [DllImport("winusb.dll", SetLastError = true)]
    public static extern bool WinUsb_SetPipePolicy(IntPtr InterfaceHandle, Byte PipeID, UInt32 PolicyType,
      UInt32 ValueLength, ref byte Value);

    [DllImport("winusb.dll", SetLastError = true)]
    public static extern bool WinUsb_GetPipePolicy(IntPtr InterfaceHandle, Byte PipeID, UInt32 PolicyType,
      ref UInt32 ValueLength, out byte Value);

    [DllImport("winusb.dll", SetLastError = true)]
    public static extern bool WinUsb_SetPipePolicy(IntPtr InterfaceHandle, Byte PipeID, UInt32 PolicyType,
      UInt32 ValueLength, ref UInt32 Value);

    [DllImport("winusb.dll", SetLastError = true)]
    public static extern bool WinUsb_GetPipePolicy(IntPtr InterfaceHandle, Byte PipeID, UInt32 PolicyType,
      ref UInt32 ValueLength, out UInt32 Value);

    [DllImport("winusb.dll", SetLastError = true)]
    public static unsafe extern bool WinUsb_WritePipe(IntPtr InterfaceHandle, byte PipeID, byte* pBuffer,
      uint BufferLength, out uint LengthTransferred, IntPtr Overlapped);

    [DllImport("winusb.dll", SetLastError = true)]
    public static unsafe extern bool WinUsb_WritePipe(IntPtr InterfaceHandle, byte PipeID, byte* pBuffer,
      uint BufferLength, out uint LengthTransferred, NativeOverlapped* pOverlapped);

    [DllImport("winusb.dll", SetLastError = true)]
    public static extern bool WinUsb_FlushPipe(IntPtr InterfaceHandle, byte PipeID);

    [DllImport("winusb.dll", SetLastError = true)]
    public static extern bool WinUsb_GetDescriptor(IntPtr InterfaceHandle, byte DescriptorType,
      byte Index, UInt16 LanguageID, byte[] Buffer, UInt32 BufferLength, out UInt32 LengthTransfered);

    [DllImport("winusb.dll", SetLastError = true)]
    public static extern bool WinUsb_GetDescriptor(IntPtr InterfaceHandle, byte DescriptorType, byte Index,
      UInt16 LanguageID, out USB_DEVICE_DESCRIPTOR deviceDesc, UInt32 BufferLength, out UInt32 LengthTransfered);

    [DllImport("winusb.dll", SetLastError = true)]
    public static extern bool WinUsb_GetDescriptor(IntPtr InterfaceHandle, byte DescriptorType, byte Index,
      UInt16 LanguageID, out USB_CONFIGURATION_DESCRIPTOR deviceDesc, UInt32 BufferLength, out UInt32 LengthTransfered);

    [DllImport("winusb.dll", SetLastError = true)]
    public static extern bool WinUsb_GetAssociatedInterface(IntPtr InterfaceHandle, byte AssociatedInterfaceIndex,
      out IntPtr AssociatedInterfaceHandle);

    public static IEnumerable<string> GetAllDevicePaths(Guid guid)
    {
      var ret = new List<string>();

      var deviceInfoSet = NativeInterface.SetupDiGetClassDevs(ref guid, null, 0,
        NativeInterface.DIGCF_PRESENT | NativeInterface.DIGCF_DEVICEINTERFACE);
      if (deviceInfoSet.ToInt64() != NativeInterface.INVALID_HANDLE_VALUE)
      {
        var deviceInfoData = _CreateDeviceInfoData();
        var deviceIndex = 0;

        while (NativeInterface.SetupDiEnumDeviceInfo(deviceInfoSet, deviceIndex, ref deviceInfoData))
        {
          deviceIndex += 1;

          var deviceInterfaceData = new NativeInterface.SP_DEVICE_INTERFACE_DATA();
          deviceInterfaceData.cbSize = Marshal.SizeOf(deviceInterfaceData);
          var deviceInterfaceIndex = 0;
          while (NativeInterface.SetupDiEnumDeviceInterfaces(deviceInfoSet, ref deviceInfoData,
            ref guid, deviceInterfaceIndex, ref deviceInterfaceData))
          {
            ret.Add(_GetDevicePath(deviceInfoSet, deviceInterfaceData));
            deviceInterfaceIndex++;
          }
        }

        NativeInterface.SetupDiDestroyDeviceInfoList(deviceInfoSet);
      }

      return ret;
    }

    private static string _GetDevicePath(IntPtr deviceInfoSet,
      NativeInterface.SP_DEVICE_INTERFACE_DATA deviceInterfaceData)
    {
      var bufferSize = 0;
      var interfaceDetail = new NativeInterface.SP_DEVICE_INTERFACE_DETAIL_DATA
      {
        Size = IntPtr.Size == 4 ? 4 + Marshal.SystemDefaultCharSize : 8
      };

      NativeInterface.SetupDiGetDeviceInterfaceDetailBuffer(deviceInfoSet, ref deviceInterfaceData,
        IntPtr.Zero, 0, ref bufferSize, IntPtr.Zero);
      return NativeInterface.SetupDiGetDeviceInterfaceDetail(deviceInfoSet, ref deviceInterfaceData,
        ref interfaceDetail, bufferSize, ref bufferSize, IntPtr.Zero) ? interfaceDetail.DevicePath : null;
    }

    private static NativeInterface.SP_DEVINFO_DATA _CreateDeviceInfoData()
    {
      var ret = new NativeInterface.SP_DEVINFO_DATA();

      ret.cbSize = Marshal.SizeOf(ret);
      ret.DevInst = 0;
      ret.ClassGuid = Guid.Empty;
      ret.Reserved = IntPtr.Zero;

      return ret;
    }
  }
}
