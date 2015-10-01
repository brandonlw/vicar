using Microsoft.Win32.SafeHandles;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Vicar.MassStorageDriver;
using Vicar.VicarInterface;

namespace Vicar
{
  public static class Program
  {
    public static bool UseUrbs = false;
    public static byte? IncomingEndpoint = null;
    public static byte? OutgoingEndpoint = null;
    public static ushort IncomingEndpointMaxPacketSize;
    public static ushort OutgoingEndpointMaxPacketSize;

    public enum Demo
    {
      MassStorage = 1,
      MassStorageForwarding = 2,
      SimulateKeyboard = 3
    }

    static void Main(string[] args)
    {
      VicarDevice vicar = null;

      try
      {
        /* Parse command-line arguments */

        //Get specified demo
        Demo demo;
        var raw = _GetCommandArgumentValue("demo");
        if (!Enum.TryParse(raw, out demo))
        {
          throw new ArgumentException("No or invalid demo specified: " + raw);
        }

        //Get preferred operating mode
        VicarDevice.OperatingMode mode;
        if (!Enum.TryParse(_GetCommandArgumentValue("mode"), out mode))
        {
          //Custom is default/preferred mode, which can be faster
          mode = VicarDevice.OperatingMode.Custom;
        }

        //Get preferred device configurations
        VicarDevice.AdditionalCustomInterface? configs = null;
        raw = _GetCommandArgumentValue("configs");
        if (!string.IsNullOrEmpty(raw))
        {
          foreach (var r in raw.Split(new char[] { '|' }))
          {
            VicarDevice.AdditionalCustomInterface c;
            if (Enum.TryParse<VicarDevice.AdditionalCustomInterface>(r, out c))
            {
              if (!configs.HasValue)
              {
                configs = VicarDevice.AdditionalCustomInterface.None;
              }

              configs |= c;
            }
          }
        }

        //Get logging verbosity
        Settings.UseVerboseLogging = args.Any(t => t.ToLower() == "/verbose");

        bool sentConfig = false;
        Console.WriteLine(string.Format("Waiting for '{0}' device connection...", mode));
        while (true)
        {
          var devicePaths = VicarDevice.GetDevicePaths();
          if (devicePaths.Any())
          {
            //Go with the first mode found (preferably the one specified)
            var paths = devicePaths.OrderByDescending(t => t.Key == mode).First();

            //Pass no more than 2 interfaces (which HID would use)
            if (vicar != null)
            {
              vicar.Close();
            }

            try
            {
              vicar = new VicarDevice(paths.Key, paths.Value.Take(Math.Min(paths.Value.Count, 2)).ToArray());
              vicar.Open();

              if (vicar.Mode != mode)
              {
                Console.WriteLine(string.Format("'{0}' mode detected, switching to '{1}'...",
                  vicar.Mode, mode));
                vicar.SetOperatingMode(mode);
                vicar.Close();

                //Wait a little bit for things to re-enumerate
                Thread.Sleep(3000);
              }
              else if (!sentConfig && configs.HasValue && vicar.Configurations != configs.Value)
              {
                sentConfig = true;
                Console.WriteLine(string.Format("Setting specified device configuration..."));
                vicar.SetDeviceConfiguration(configs.Value);

                //Wait a little bit for things to re-enumerate
                Thread.Sleep(3000);
              }
              else
              {
                break;
              }
            }
            catch
            {
              //Eat it...
            }
          }

          if (Console.KeyAvailable)
          {
            if (Console.ReadKey(true).Key == ConsoleKey.Escape)
            {
              Console.WriteLine("[ESC] pressed; aborting...");
              if (vicar != null)
              {
                vicar.Close();
                vicar = null;
              }
            }
          }

          //Don't hog the CPU while we wait
          Thread.Sleep(100);
        }

        if (vicar != null)
        {
          Console.WriteLine("Device detected, allowing peripheral detection...");
          vicar.AllowPeripheralDetection(true);
          Console.WriteLine("Waiting for peripheral to become attached...");
          vicar.WaitUntilPeripheralConnected();
          Console.WriteLine("Peripheral is attached. Moving on...");

          //Get the first 8 bytes of the device descriptor
          byte address = 0x00;
          var m = vicar.SendControlRequest(address, 0x80, 0x06, 0x0100, 0x0000, 0x0008, null);

          //Set the address
          vicar.SendControlRequest(address, 0x00, 0x05, ++address, 0x0000, 0x0000);

          //Set the control endpoint's maximum packet size (or just assume 0x08 if we can't)
          //This is necessary to enable large control transfers on this address
          var bMaxPacketSize0 = (byte)(m.Length >= 8 ? m[7] : 0x08); //just stick with 0x08, I guess
          vicar.ConfigureEndpoint(address, 0x00, bMaxPacketSize0, true);

          //Get the configuration descriptor (once to get the size, and again to get the whole thing)
          //  (or just assume 0xFF if we can't)
          var configDescriptor = vicar.SendControlRequest(address, 0x80, 0x06, 0x0200, 0x0000, 4);
          var length = configDescriptor.Length >= 4 ? Utilities.ToLittleEndianUshort(configDescriptor, 2)
            : (ushort)0xFF;
          configDescriptor = vicar.SendControlRequest(address, 0x80, 0x06, 0x0200, 0x0000, length);

          //Get the incoming and outgoing endpoints from any endpoint descriptors
          int i = 0;
          while (i < configDescriptor.Length)
          {
            if (configDescriptor[i + 1] == 0x05)
            {
              //Endpoint descriptor
              var ep = configDescriptor[i + 2];
              var maxPacketSize = Utilities.ToLittleEndianUshort(configDescriptor, i + 4);
              if ((ep & 0x80) > 0)
              {
                //Incoming endpoint
                if (!IncomingEndpoint.HasValue)
                {
                  IncomingEndpoint = ep;
                  IncomingEndpointMaxPacketSize = maxPacketSize;
                }
              }
              else
              {
                //Outgoing endpoint
                if (!OutgoingEndpoint.HasValue)
                {
                  OutgoingEndpoint = ep;
                  OutgoingEndpointMaxPacketSize = maxPacketSize;
                }
              }
            }

            i += configDescriptor[i];
          }

          //Set the device configuration (assuming 1)
          vicar.SendControlRequest(address, 0x00, 0x09, 0x0001, 0x0000, 0x0000);

          //Get the max LUN
          //vicar.SendControlRequest(address, 0xA1, 0xFE, 0x0000, 0x0000, 0x0001);

          //Configure the two endpoints
          //The device will auto-poll them and alert us of any new data (if not using URBs)

          if (IncomingEndpoint.HasValue)
          {
            vicar.ConfigureEndpoint(0x01, IncomingEndpoint.Value, IncomingEndpointMaxPacketSize,
              demo == Demo.MassStorageForwarding || !UseUrbs);
          }

          if (OutgoingEndpoint.HasValue)
          {
            vicar.ConfigureEndpoint(0x01, OutgoingEndpoint.Value, OutgoingEndpointMaxPacketSize, true);
          }

          switch (demo)
          {
            case Demo.MassStorage:
              {
                _DoMassStorageDemo(vicar);

                break;
              }

            case Demo.MassStorageForwarding:
              {
                vicar.EnableInterfaceForwarding(true, 0x01, IncomingEndpoint.GetValueOrDefault(),
                  OutgoingEndpoint.GetValueOrDefault());
                Console.WriteLine("Forwarding is enabled. Press any key to quit.");
                Console.ReadKey(true);

                break;
              }

            case Demo.SimulateKeyboard:
              {
                while (true)
                {
                  Console.WriteLine("Enter a string and press enter. It will be echoed back to you in 5 seconds.");
                  Console.WriteLine("Enter a blank line to quit.");
                  Console.Write("> ");
                  var line = Console.ReadLine();
                  if (string.IsNullOrEmpty(line))
                  {
                    break;
                  }

                  Thread.Sleep(5000);
                  _SendString(vicar, line);
                  _SendKey(vicar, 0x00, 0x28);
                }

                break;
              }

            default:
              {
                break;
              }
          }


          vicar.Close();
        }
        else
        {
          Console.WriteLine("No device detected; exiting.");
        }
      }
      catch (Exception ex)
      {
        Console.WriteLine("FATAL: " + ex.ToString());
      }
      finally
      {
        if (Debugger.IsAttached)
        {
          Console.WriteLine("Press any key to exit the application.");
          Console.ReadKey(true);
        }
      }
    }

    private static void _DoMassStorageDemo(VicarDevice vicar)
    {
      //Create an instance of a FAT32 file system image
      var stream = new DiscUtilsStream(new MassStorageHelper(vicar));
      var fat = new DiscUtils.Fat.FatFileSystem(stream);
      var currentDir = "\\";
      bool running = true;
      Console.WriteLine("Type 'help' for options or 'quit' to quit.");
      while (running)
      {
        Console.Write(currentDir.TrimStart('\\') + "> ");
        var line = Console.ReadLine();
        var parts = line.Split(new char[] { ' ' });

        try
        {
          switch (parts[0].ToLower())
          {
            case "flush":
              {
                stream.Flush();
                break;
              }

            case "ls":
            case "dir":
              {
                try
                {
                  //List all directories
                  foreach (var dir in fat.GetDirectories(currentDir))
                  {
                    Console.WriteLine("\t" + dir);
                  }

                  //Now list all files
                  foreach (var file in fat.GetFiles(currentDir))
                  {
                    Console.WriteLine("\t" + Path.GetFileName(file));
                  }
                }
                catch (Exception ex)
                {
                  Console.WriteLine("ERROR: " + ex.Message);
                }

                break;
              }

            case "pwd":
              {
                Console.WriteLine(currentDir);
                break;
              }

            case "get":
              {
                var file = fat.OpenFile(currentDir + "\\" + parts[1], FileMode.Open);
                var data = new byte[file.Length];
                file.Read(data, 0, data.Length);
                file.Close();

                File.WriteAllBytes(parts[1], data);
                break;
              }

            case "put":
              {
                var data = File.ReadAllBytes(parts[1]);
                var file = fat.OpenFile(currentDir + "\\" + parts[1], FileMode.Create);
                file.Write(data, 0, data.Length);
                file.Close();

                break;
              }

            case "cd":
              {
                if (parts[1] == "..")
                {
                  var idx = currentDir.LastIndexOf("\\");
                  if (idx != -1)
                  {
                    currentDir = currentDir.Substring(0, idx);
                  }
                }
                else
                {
                  currentDir += "\\" + parts[1];
                }

                break;
              }

            case "exit":
            case "quit":
              {
                running = false;
                break;
              }

            case "help":
              {
                Console.WriteLine("Help:");
                Console.WriteLine("cd\tChanges to specified directory.");
                Console.WriteLine("dir\tDisplays current directory's contents.");
                Console.WriteLine("ls");
                Console.WriteLine("pwd\tPrints current directory.");
                Console.WriteLine("get\tGets specified file to disk.");
                break;
              }

            default:
              {
                break;
              }
          }
        }
        catch (Exception ex)
        {
          Console.WriteLine("Error handling command: " + ex.ToString());
        }
      }
    }

    private static void _SendString(VicarDevice vicar, string str)
    {
      foreach (var c in str)
      {
        byte modifier, key;
        _GetKeyCode(c, out modifier, out key);

        if (modifier != 0 || key != 0)
        {
          _SendKey(vicar, modifier, key);
        }
      }
    }

    private static void _SendKey(VicarDevice vicar, byte modifier, byte key)
    {
      vicar.SendInputData(VicarDevice.AdditionalCustomInterface.HumanInterfaceDevice,
        new byte[] { modifier, 0x00, key, 0x00, 0x00, 0x00, 0x00, 0x00 });
      vicar.SendInputData(VicarDevice.AdditionalCustomInterface.HumanInterfaceDevice,
        new byte[] { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
    }

    private static void _GetKeyCode(char c, out byte modifier, out byte key)
    {
      modifier = 0x00;
      key = 0x00;

      if (c >= 'A' && c <= 'Z')
      {
        //Hold down shift
        modifier |= 0x02;

        key = (byte)(((byte)c - (byte)'A') + 0x04);
      }
      else if (c >= 'a' && c <= 'z')
      {
        key = (byte)(((byte)c - (byte)'a') + 0x04);
      }
      else if (c == ' ')
      {
        key = 0x2C;
      }
    }

    private static string _GetCommandArgumentValue(string arg)
    {
      var matches = Environment.GetCommandLineArgs().Where(t =>
      {
        var idx = t.IndexOf("=");
        if (idx != -1)
        {
          return arg.ToLower().TrimStart('/') == t.Substring(0, idx).TrimStart('/').ToLower();
        }
        else
        {
          return false;
        }
      });

      if (matches.Any())
      {
        var match = matches.First();
        return match.Substring(match.IndexOf("=") + 1);
      }
      else
      {
        return string.Empty;
      }
    }
  }
}
