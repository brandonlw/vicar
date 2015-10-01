using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace DiscUtils
{
    /// <summary>
    /// DelayedByteArray
    /// </summary>
    public class DelayedByteArray
    {
        private Stream _stream;
        private byte[] _cachedBitmap;
        private long _startPosition;
        private byte[] _buffer;

        public DelayedByteArray(byte[] data)
        {
          _buffer = data;

          //No stream to read from, so just assume it's all already there
          _cachedBitmap = new byte[Math.Max(8, data.Length / 8)];
          for (int i = 0; i < _cachedBitmap.Length; i++)
          {
            _cachedBitmap[i] = 0xFF;
          }
        }

        /// <summary>
        /// Creates a DelayedByteArray object.
        /// </summary>
        /// <param name="stream"></param>
        /// <param name="count"></param>
        public DelayedByteArray(Stream stream, int count)
        {
            _stream = stream;
            _startPosition = stream.Position;
            _buffer = new byte[count];
            _cachedBitmap = new byte[Math.Max(1, count / 8)];

            if (stream == null)
            {
                //No stream to read from, so just assume it's all already there
                for (int i = 0; i < _cachedBitmap.Length; i++)
                {
                    _cachedBitmap[i] = 0xFF;
                }
            }
        }

        /// <summary>
        /// Length of the array.
        /// </summary>
        public int Length
        {
            get
            {
                return _buffer.Length;
            }
        }

        /// <summary>
        /// Returns a byte at the specified index, retrieving it from the stream if necessary.
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public byte this[int index]
        {
            get
            {
                if (((_cachedBitmap[index / 8] >> (index % 8)) & 0x01) == 0x00)
                {
                    //Read it
                    var temp = _stream.Position;
                    _stream.Position = _startPosition;
                    var sector = index / 512;
                    _stream.Read(_buffer, sector * 512, 512);
                    _stream.Position = temp;

                    //Mark it as read
                    for (int i = 0; i < 512; i++)
                    {
                      _SetCached(sector * 512 + i);
                    }
                }

                return _buffer[index];
            }

            set
            {
                _buffer[index] = value;
            }
        }

        private void _SetCached(int byteOffset)
        {
          int offsetInBitmapByte = byteOffset % 8;

          _cachedBitmap[(byteOffset / 8)] |= (byte)(0x01 << (offsetInBitmapByte));
        }

        /// <summary>
        /// RawData
        /// </summary>
        public byte[] RawData
        {
            get
            {
                return _buffer;
            }
        }

        /// <summary>
        /// GetBytes
        /// </summary>
        /// <param name="offset"></param>
        /// <param name="count"></param>
        /// <returns></returns>
        public byte[] GetBytes(int offset, int count)
        {
            var ret = new byte[count];

            for (int i = 0; i < ret.Length; i++)
            {
                ret[i] = this[offset + i];
            }

            return ret;
        }
    }
}
