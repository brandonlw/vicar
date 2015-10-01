using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Vicar.MassStorageDriver
{
  public class DiscUtilsStream : Stream
  {
    private const int _MAX_CACHED_SECTORS = 98304; //48MB
    private MassStorageHelper _msHelper;
    private bool _canRead;
    private bool _canWrite;
    private bool _canSeek;
    private long _currentPosition = 0;
    private List<CachedSector> _sectors;
    private uint? _partitionOffset;

    public DiscUtilsStream(MassStorageHelper msHelper)
      : base()
    {
      _msHelper = msHelper;
      _sectors = new List<CachedSector>();

      _canRead = true;
      _canWrite = true;
      _canSeek = true;
    }

    public override bool CanRead
    {
      get
      {
        return _canRead;
      }
    }

    public override bool CanWrite
    {
      get
      {
        return _canWrite;
      }
    }

    public override bool CanSeek
    {
      get
      {
        return _canSeek;
      }
    }

    public override long Position
    {
      get
      {
        return _currentPosition;
      }

      set
      {
        _currentPosition = value;
      }
    }

    public override long Length
    {
      get
      {
        //Uh?!
        return Position;
      }
    }

    public override void SetLength(long value)
    {
      //Do nothing for now
    }

    public override void Flush()
    {
      //Run through all cached sectors, which will write them to disk if necessary
      foreach (var sector in _sectors.Where(t => t.IsDirty))
      {
        _msHelper.WriteSector(sector.LBA, sector.SectorData);
        sector.IsDirty = false;
      }
    }

    public override long Seek(long offset, SeekOrigin origin)
    {
      switch (origin)
      {
        case SeekOrigin.Begin:
          {
            Position = offset;
            break;
          }
        case SeekOrigin.Current:
          {
            Position += offset;
            break;
          }
        case SeekOrigin.End:
          {
            Position = Length - offset;
            break;
          }
        default:
          {
            //Uh...
            break;
          }
      }

      return Position;
    }

    private uint _GetPartitionOffset()
    {
      if (!_partitionOffset.HasValue)
      {
        _partitionOffset = _msHelper.GetPartitionTableEntries().First();
      }

      return _partitionOffset.Value;
    }

    public override int Read(byte[] buffer, int offset, int count)
    {
      if (Settings.UseVerboseLogging)
      {
        Console.WriteLine(string.Format("Beginning sector read at position {0}, offset {1}, count {2}",
          Position.ToString("X08"), offset.ToString("X08"), count.ToString("X08")));
      }

      var lba = (ulong)Math.Floor((double)((Position + offset) / 512));
      lba += _GetPartitionOffset();
      var offsetIntoSector = (Position + offset) % 512;
      var sectorData = _GetSector(lba).SectorData;
      while ((sectorData.Length - offsetIntoSector) < count)
      {
        sectorData = sectorData.Concat(_GetSector(++lba).SectorData).ToArray();
      }

      Array.Copy(sectorData, offsetIntoSector, buffer, offset, count);
      if (Settings.UseVerboseLogging)
      {
        Console.WriteLine("Read done.");
      }

      return count;
    }

    public override void Write(byte[] buffer, int offset, int count)
    {
      if (Settings.UseVerboseLogging)
      {
        Console.WriteLine(string.Format("Beginning sector write at position {0}, offset {1}, count {2}",
          Position.ToString("X08"), offset.ToString("X08"), count.ToString("X08")));
      }

      var lba = (ulong)Math.Floor((double)((Position + offset) / 512));
      lba += _GetPartitionOffset();
      var offsetIntoSector = (Position + offset) % 512;
      var l = lba;
      int remaining = count;
      var chunkSize = 512 - offsetIntoSector;
      long idx = 0;
      while (idx < count)
      {
        var sector = _GetSector(l);
        Array.Copy(buffer, offset + idx, sector.SectorData, offsetIntoSector, Math.Min(remaining, chunkSize));
        sector.IsDirty = true;

        l++;
        idx += chunkSize;
        chunkSize = 512;
        offsetIntoSector = 0;
      }
    }

    private CachedSector _GetSector(ulong lba)
    {
      if (!_sectors.Any(t => t.LBA == lba))
      {
        if (_sectors.Count >= _MAX_CACHED_SECTORS)
        {
          _RemoveSector(_sectors.Aggregate((curMin, x) =>
            (curMin == null || x.CacheTime < curMin.CacheTime ? x : curMin)));
        }

        if (Settings.UseVerboseLogging)
        {
          Console.WriteLine("Retrieving data for LBA " + lba.ToString("X08"));
        }

        //Read it
        _sectors.Add(new CachedSector(lba, _msHelper.ReadSector(lba, 1)));
      }

      return _sectors.FirstOrDefault(t => t.LBA == lba);
    }

    private void _RemoveSector(CachedSector sector)
    {
      if (_sectors.Contains(sector))
      {
        _msHelper.WriteSector(sector.LBA, sector.SectorData);
        _sectors.Remove(sector);
      }
    }

    private class CachedSector
    {
      public CachedSector(ulong lba, byte[] sectorData)
      {
        LBA = lba;
        SectorData = sectorData;
        CacheTime = DateTime.Now;
      }

      public ulong LBA { get; set; }
      public DateTime CacheTime { get; set; }
      public byte[] SectorData { get; set; }
      public bool IsDirty { get; set; }
    }
  }
}
