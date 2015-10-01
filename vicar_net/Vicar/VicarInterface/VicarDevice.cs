using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Vicar.HIDInterface;

namespace Vicar.VicarInterface
{
  public partial class VicarDevice : IDisposable
  {
    private const uint _FIRST_SEQUENCE = 0x80000001;
    private const int _REQUEST_TIMEOUT_MS = 5000;
    private const int _SLEEP_MS = 1;
    private Thread _readCmdPipeThread;
    private Thread _readDataPipeThread;
    private List<VicarMessage> _receivedMessages;
    private uint _nextSequence = _FIRST_SEQUENCE;
    private AdditionalCustomInterface? _configs;
    private PeripheralStatus? _status;
    private Mutex _deviceMutex = new Mutex();
    private Dictionary<uint, Urb> _urbs;
    private uint? _lastDataPipeUrbId = null;

    public VicarDevice(OperatingMode mode, params string[] devicePaths)
    {
      Mode = mode;
      _receivedMessages = new List<VicarMessage>();
      _urbs = new Dictionary<uint, Urb>();

      _Init(devicePaths);
    }

    public event EventHandler<IncomingDataEventArgs> IncomingDataReceived;

    public event EventHandler<TransferCompleteEventArgs> TransferCompleted;

    public void Open()
    {
      _Open();

      if (_readCmdPipeThread == null)
      {
        _readCmdPipeThread = new Thread(new ThreadStart(_ReadCmdPipeLoop));
        _readCmdPipeThread.IsBackground = true;
        _readCmdPipeThread.Start();
      }

      if (_readDataPipeThread == null)
      {
        _readDataPipeThread = new Thread(new ThreadStart(_ReadDataPipeLoop));
        _readDataPipeThread.IsBackground = true;
        _readDataPipeThread.Start();
      }
    }

    public void ResetBus()
    {
      var response = _SendRequest((byte)'R');
      _VerifyResponseLength(response, 1);
      var error = (VicarErrorCode)response.Data[0];
      if (error != VicarErrorCode.Success)
      {
        throw new VicarException(error, "Error resetting bus");
      }
    }

    public void WaitUntilPeripheralConnected()
    {
      while (_status.GetValueOrDefault() != PeripheralStatus.AttachedAndReady)
      {
        Thread.Sleep(_SLEEP_MS);
      }
    }

    public void ConfigureEndpoint(byte address, byte endpoint, ushort maxPacketSize, bool autoPoll)
    {
      var d = new byte[1 + 1 + 4 + 1];
      d[0] = address;
      d[1] = endpoint;
      Utilities.SetLittleEndianUint(d, 2, maxPacketSize);
      d[6] = (byte)(autoPoll ? 0x01 : 0x00);

      var response = _SendRequest((byte)'E', d);
      _VerifyResponseLength(response, 1);
      var error = (VicarErrorCode)response.Data[0];
      if (error != VicarErrorCode.Success)
      {
        throw new VicarException(error, string.Format(("Error configuring endpoint {0} on address {1}, " +
          "wMaxPacketSize {2}, auto-poll {3}"), endpoint, address, maxPacketSize, autoPoll));
      }
    }

    public void EnableInterfaceForwarding(bool enabled, byte? address, byte? incomingEndpoint, byte? outgoingEndpoint)
    {
      var d = new byte[1 + 1 + 2];
      d[0] = (byte)(enabled ? 1 : 0);
      d[1] = address.GetValueOrDefault();
      d[2] = incomingEndpoint.GetValueOrDefault();
      d[3] = outgoingEndpoint.GetValueOrDefault();

      var response = _SendRequest((byte)'W', d);
      _VerifyResponseLength(response, 1);
      var error = (VicarErrorCode)response.Data[0];
      if (error != VicarErrorCode.Success)
      {
        throw new VicarException(error, string.Format(("Error setting interface forwarding; enabled = " +
          "'{0}', address '{1}', incoming endpoint '{2}', outgoing endpoint '{3}'"), enabled,
          address, incomingEndpoint, outgoingEndpoint));
      }
    }

    public void ForgetAllEndpoints()
    {
      var response = _SendRequest((byte)'F');
      _VerifyResponseLength(response, 1);
      var error = (VicarErrorCode)response.Data[0];
      if (error != VicarErrorCode.Success)
      {
        throw new VicarException(error, "Error forgetting all endpoints");
      }
    }

    public void ForgetEndpoint(byte addr, byte endpoint)
    {
      var response = _SendRequest((byte)'F', new byte[] { addr, endpoint });
      _VerifyResponseLength(response, 1);
      var error = (VicarErrorCode)response.Data[0];
      if (error != VicarErrorCode.Success)
      {
        throw new VicarException(error, "Error forgetting all endpoints");
      }
    }

    public void AllowPeripheralDetection(bool allow)
    {
      var response = _SendRequest((byte)'N', new byte[] { (byte)(allow ? 1 : 0) });
      _VerifyResponseLength(response, 1);
      var error = (VicarErrorCode)response.Data[0];
      if (error != VicarErrorCode.Success)
      {
        throw new VicarException(error, "Error allowing peripheral detection");
      }
    }

    public void SetDeviceConfiguration(VicarDevice.AdditionalCustomInterface configuration)
    {
      _SendRequest((byte)'D', new byte[] { (byte)configuration }, false);
    }

    public VicarMessage SetOperatingMode(OperatingMode mode)
    {
      return _SendRequest((byte)'M', new byte[] { (byte)mode }, false);
    }

    public Urb SubmitUrb(byte address, byte endpoint, uint requestedLength)
    {
      Urb ret = null;

      var d = new byte[1 + 1 + 4];
      d[0] = address;
      d[1] = endpoint;
      Utilities.SetBigEndianUint(d, 2, requestedLength);

      lock (_urbs)
      {
        var response = _SendRequest((byte)'U', d);
        _VerifyResponseLength(response, 5);
        var error = (VicarErrorCode)response.Data[0];
        if (error != VicarErrorCode.Success)
        {
          throw new VicarException(error, string.Format("Error submitting URB for address {0}, endpoint {1}, length {2}",
            address.ToString("X02"), endpoint.ToString("X02"), requestedLength.ToString("X08")));
        }

        ret = new Urb(Utilities.ToLittleEndianUint(response.Data, 1), address, endpoint);
        _urbs.Add(ret.Id, ret);
      }

      return ret;
    }

    public void CancelUrb(uint id)
    {
      var d = new byte[4];
      Utilities.SetLittleEndianUint(d, 0, id);

      _MarkUrbComplete(id, 0);
      var response = _SendRequest((byte)'L', d);
      _VerifyResponseLength(response, 1);
      var error = (VicarErrorCode)response.Data[0];
      if (error != VicarErrorCode.Success)
      {
        throw new VicarException(error, string.Format("Error canceling URB with ID {0}", id.ToString("X08")));
      }
    }

    public byte[] SendControlRequest(byte address, byte bmRequestType, byte bRequest,
      ushort wValue, ushort wIndex, ushort wLength)
    {
      return SendControlRequest(address, bmRequestType, bRequest, wValue, wIndex, wLength, null);
    }

    public byte[] SendControlRequest(byte address, byte bmRequestType, byte bRequest,
      ushort wValue, ushort wIndex, ushort wLength, byte[] data)
    {
      //Address + setup packet + additional data
      var d = new byte[1 + 8 + (data != null ? data.Length : 0)];

      //Specify the destination address
      d[0] = address;

      //Prepare the setup packet
      d[1] = bmRequestType;
      d[2] = bRequest;
      d[3] = (byte)(wValue & 0xFF);
      d[4] = (byte)((wValue >> 8) & 0xFF);
      d[5] = (byte)(wIndex & 0xFF);
      d[6] = (byte)((wIndex >> 8) & 0xFF);
      d[7] = (byte)(wLength & 0xFF);
      d[8] = (byte)((wLength >> 8) & 0xFF);

      //Copy any additional data
      if (data != null)
      {
        Array.Copy(data, 0, d, 1 + 8, data.Length);
      }

      //Send it and check for errors
      var response = _SendRequest((byte)'C', d);
      _VerifyResponseLength(response, 3);
      var error = (VicarErrorCode)response.Data[0];
      if (error != VicarErrorCode.Success)
      {
        throw new VicarException(error, "Error transferring control request " +
          BitConverter.ToString(d.Skip(1).Take(8).ToArray()));
      }

      //Extract the response data, if any, and return it
      var length = Utilities.ToLittleEndianUshort(response.Data, 1);
      var ret = response.Data.Skip(3);
      return ret.Take(Math.Min(length, ret.Count())).ToArray();
    }

    public void SendInputData(VicarDevice.AdditionalCustomInterface iface, byte[] data)
    {
      var d = new byte[1 + 2 + data.Length];
      Utilities.SetLittleEndianUshort(d, 1, (ushort)data.Length);
      Array.Copy(data, 0, d, 3, data.Length);
      switch (iface)
      {
        case AdditionalCustomInterface.HumanInterfaceDevice:
          {
            d[0] = _CUSTOM_HID_INCOMING_ENDPOINT;
            break;
          }

        case AdditionalCustomInterface.MassStorageDevice:
          {
            d[0] = _CUSTOM_MSD_INCOMING_ENDPOINT;
            break;
          }

        default:
          {
            break;
          }
      }

      var response = _SendRequest((byte)'V', d, false);
    }

    public void SendEndpointData(byte address, byte endpoint, byte[] data)
    {
      var d = new byte[1 + 1 + 4 + data.Length];
      d[0] = address;
      d[1] = endpoint;
      Utilities.SetBigEndianUint(d, 2, (uint)data.Length);
      Array.Copy(data, 0, d, 6, data.Length);

      var response = _SendRequest((byte)'O', d);
      _VerifyResponseLength(response, 1);
      var error = (VicarErrorCode)response.Data[0];
      if (error != VicarErrorCode.Success)
      {
        throw new VicarException(error, string.Format("Error sending {0} bytes on address {1}, endpoint {2}",
          data.Length.ToString("X08"), address.ToString("X02"), endpoint.ToString("X02")));
      }
    }

    private void _VerifyResponseLength(VicarMessage message, int length)
    {
      if (message != null)
      {
        var received = message.Data != null ? message.Data.Length : 0;
        if (received < length)
        {
          throw new VicarException(string.Format("Invalid response length; expected {0}, received {1}",
            length, received));
        }
      }
      else
      {
        throw new VicarException("No response specified");
      }
    }

    public void Close()
    {
      _Close();
    }

    private void _GetStatus()
    {
      var response = _SendRequest((byte)'G');
      _VerifyResponseLength(response, 3);

      Mode = (OperatingMode)response.Data[0];
      _configs = (AdditionalCustomInterface)response.Data[1];
      _status = (PeripheralStatus)response.Data[2];
    }

    private void _MarkUrbComplete(uint id, int transferred)
    {
      if (_urbs.ContainsKey(id))
      {
        //Get all the collections of data for this URB
        IEnumerable<byte> data = null;

        var urb = _urbs[id];
        foreach (var entry in urb.ReceivedData)
        {
          if (data == null)
          {
            data = entry;
          }
          else
          {
            data = data.Concat(entry);
          }
        }

        lock (_urbs)
        {
          _urbs.Remove(id);
          if (data != null && IncomingDataReceived != null)
          {
            IncomingDataReceived(this, new IncomingDataEventArgs(VicarErrorCode.Success,
              urb.Address, urb.Endpoint, data.Take(transferred).ToArray()));
          }
        }
      }
    }

    private uint _GetNextSequence()
    {
      var ret = _nextSequence;

      if (_nextSequence == uint.MaxValue)
      {
        _nextSequence = _FIRST_SEQUENCE;
      }
      else
      {
        _nextSequence++;
      }

      return ret;
    }

    private VicarMessage _SendRequest(byte command)
    {
      return _SendRequest(command, null);
    }

    private VicarMessage _SendRequest(byte command, byte[] data)
    {
      return _SendRequest(command, data, true);
    }

    private VicarMessage _SendRequest(byte command, byte[] data, bool waitForResponse)
    {
      VicarMessage ret = null;
      var seq = _GetNextSequence();
      _WriteMessage(command, seq, data);

      if (waitForResponse)
      {
        var timeout = DateTime.Now.AddMilliseconds(_REQUEST_TIMEOUT_MS);

        do
        {
          lock (_receivedMessages)
          {
            ret = _receivedMessages.Where(t => t.Command == command && t.Sequence == seq).FirstOrDefault();
            if (ret != null)
            {
              _receivedMessages.Remove(ret);
            }
          }

          Thread.Sleep(_SLEEP_MS);
        } while (ret == null && DateTime.Now < timeout);
      }

      if (waitForResponse && ret == null)
      {
        throw new VicarException(string.Format("Request '{0}' timed out",
          ASCIIEncoding.ASCII.GetChars(new byte[] { command }).First()));
      }

      return ret;
    }

    private void _ReadCmdPipeLoop()
    {
      while (true)
      {
        try
        {
          var msg = _ReadMessage();
          if (msg != null)
          {
            lock (_receivedMessages)
            {
              _receivedMessages.Add(msg);

              if (Settings.UseVerboseLogging && msg.Command != 0x00)
              {
                Console.WriteLine("Received:" + (msg.Command >= (byte)'A' && msg.Command <= (byte)'z' ?
                  ASCIIEncoding.ASCII.GetString(new byte[] { msg.Command }) : msg.Command.ToString("X02")) +
                  ":" + msg.Sequence.ToString("X08") + ":" + msg.Data.Length.ToString("X04") +
                  ":" + BitConverter.ToString(msg.Data));
              }

              if (_HandleIncomingMessage(msg))
              {
                _receivedMessages.Remove(msg);
              }
            }
          }
        }
        catch
        {
          //Oh well...
        }

        Thread.Sleep(_SLEEP_MS);
      }
    }

    private void _ReadDataPipeLoop()
    {
      while (true)
      {
        if (Mode == OperatingMode.Custom && _winUsbDevice != null)
        {
          try
          {
            var data = _ReadViaWinUSB(_CUSTOM_INCOMING_DATA_ENDPOINT);
            lock (_urbs)
            {
              if (data != null && data.Length > 0)
              {
                int dataOffset = 0;
                if (!_lastDataPipeUrbId.HasValue)
                {
                  _lastDataPipeUrbId = Utilities.ToLittleEndianUint(data, 1);
                  dataOffset = 5;
                }

                if (_urbs.ContainsKey(_lastDataPipeUrbId.Value))
                {
                  var urb = _urbs[_lastDataPipeUrbId.Value];
                  urb.ReceivedData.Add(data.Skip(dataOffset).ToArray());
                }
              }
              else
              {
                //This is a signal that the endpoint data is over
                if (_urbs.ContainsKey(_lastDataPipeUrbId.GetValueOrDefault()))
                {
                  _MarkUrbComplete(_lastDataPipeUrbId.Value,
                    Math.Max(0, _urbs[_lastDataPipeUrbId.Value].ReceivedData.Sum(t => t.Length) - 5));
                }

                _lastDataPipeUrbId = null;
              }
            }
          }
          catch
          {
            //Oh well...
          }
        }

        Thread.Sleep(_SLEEP_MS);
      }
    }

    private bool _HandleIncomingMessage(VicarMessage msg)
    {
      bool ret;

      switch (msg.Command)
      {
        case (byte)'S':
          {
            ret = true;
            _status = (PeripheralStatus)msg.Data.FirstOrDefault();
            break;
          }

        case (byte)'I':
          {
            ret = true;
            if (IncomingDataReceived != null)
            {
              IncomingDataReceived(this, new IncomingDataEventArgs((VicarErrorCode)msg.Data[2],
                msg.Data[0], msg.Data[1], msg.Data.Skip(3).ToArray()));
            }

            break;
          }

        case (byte)'T':
          {
            ret = true;
            var id = Utilities.ToLittleEndianUint(msg.Data, 0);
            if (_urbs.ContainsKey(id))
            {
              _urbs[id].ReceivedData.Add(msg.Data.Skip(6).ToArray());
            }

            break;
          }

        case (byte)'B':
          {
            ret = true;
            var id = Utilities.ToLittleEndianUint(msg.Data, 0);
            var transferred = Utilities.ToLittleEndianUint(msg.Data, 5);
            _MarkUrbComplete(id, (int)transferred);

            if (TransferCompleted != null)
            {
              TransferCompleted(this, new TransferCompleteEventArgs(id,
                msg.Data[4], transferred));
            }

            break;
          }

        default:
          {
            ret = false;
            break;
          }
      }

      return ret;
    }

    private VicarMessage _ReadMessage()
    {
      byte[] ret = null;

      if (_bytesReceivedSoFar == null)
      {
        _bytesReceivedSoFar = new List<byte>();
      }

      //Do we have enough bytes for a packet header right now?
      if (!_EnoughForPacketHeader())
      {
        //No, so do a read
        _DoRead();
      }

      //How about now?
      if (_EnoughForPacketHeader())
      {
        var data = new byte[Utilities.ToBigEndianUshort(_bytesReceivedSoFar.ToArray(), 5)];

        //Do we have enough bytes for the whole packet?
        if (!_EnoughForWholePacket(data.Length))
        {
          //No, so go ahead and do another read -- with a half-packet received, this is likely to give us data
          _DoRead();
        }

        ret = _bytesReceivedSoFar.Take(_PACKET_HEADER_LENGTH + data.Length).ToArray();

        if (Mode == OperatingMode.HID)
        {
          _bytesReceivedSoFar = _bytesReceivedSoFar.Skip(_HID1_REPORT_SIZE).ToList();
        }
        else
        {
          _bytesReceivedSoFar = _bytesReceivedSoFar.Skip(ret.Length).ToList();
        }
      }

      if (ret != null)
      {
        return new VicarMessage(ret[0], Utilities.ToBigEndianUint(ret, 1),
          ret.Skip(_PACKET_HEADER_LENGTH).Take(Utilities.ToBigEndianUshort(ret, 5)).ToArray());
      }
      else
      {
        if (_urbs.Count > 0 && Mode == OperatingMode.HID)
        {
          var result = _hidDevice2.GetFeature(_HID_REPORT_ID);
          if (result.Status == HidReadResult.ReadStatus.Success)
          {
            return new VicarMessage((byte)'T', 0, result.Data);
          }
          else
          {
            return null;
          }
        }
        else
        {
          return null;
        }
      }
    }

    private void _WriteMessage(byte command, uint sequence, byte[] data)
    {
      byte[] message = new byte[(data != null ? data.Length : 0) + _PACKET_HEADER_LENGTH];
      message[0] = command;
      Utilities.SetBigEndianUint(message, 1, sequence);
      if (data != null)
      {
        Utilities.SetBigEndianUshort(message, 5, (ushort)data.Length);
        Array.Copy(data, 0, message, _PACKET_HEADER_LENGTH, data.Length);
      }

      if (Settings.UseVerboseLogging)
      {
        Console.WriteLine(string.Format("Writing:{0}:{1}:", ASCIIEncoding.ASCII.GetString(new byte[] { command }),
          sequence.ToString("X04")) + (data != null ? BitConverter.ToString(data) : string.Empty));
      }

      int index = 0;
      int remaining = message.Length;
      while (remaining > 0)
      {
        int size = Math.Min(remaining, _HID1_REPORT_SIZE);

        _WriteData(message.Skip(index).Take(size).ToArray());
        index += size;
        remaining -= size;
      }
    }

    public class Urb
    {
      public Urb(uint id, byte address, byte endpoint)
      {
        Id = id;
        Address = address;
        Endpoint = endpoint;

        ReceivedData = new List<byte[]>();
      }

      public uint Id { get; set; }

      public byte Address { get; set; }

      public byte Endpoint { get; set; }

      public List<byte[]> ReceivedData { get; private set; }
    }
  }
}
