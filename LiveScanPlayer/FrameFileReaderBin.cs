using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace LiveScanPlayer {
    class FrameFileReaderBin : IFrameFileReader {
        private readonly BinaryReader _binaryReader;
        private int _currentFrameIdx;

        public FrameFileReaderBin(string filename) {
            _binaryReader = new BinaryReader(File.Open(filename, FileMode.Open));
        }

        ~FrameFileReaderBin() {
            _binaryReader.Dispose();
        }

        public int FrameIdx {
            get => _currentFrameIdx;
            set => JumpToFrame(value);
        }

        public void ReadFrame(List<float> vertices, List<byte> colors) {
            while (true) {
                if (_binaryReader.BaseStream.Position == _binaryReader.BaseStream.Length) Rewind();

                var lineParts = ReadLine().Split(' ');
                var nPoints = int.Parse(lineParts[1]);
                lineParts = ReadLine().Split(' ');
                var frameTimestamp = int.Parse(lineParts[1]);

                Console.WriteLine((frameTimestamp / 1000).ToString());

                var tempVertices = new short[3 * nPoints];
                var tempColors = new byte[4 * nPoints];

                const int bytesPerVertexPoint = 3 * sizeof(short);
                const int bytesPerColorPoint = 4 * sizeof(byte);
                const int bytesPerPoint = bytesPerVertexPoint + bytesPerColorPoint;

                var frameData = _binaryReader.ReadBytes(bytesPerPoint * nPoints);

                if (frameData.Length < bytesPerPoint * nPoints) {
                    Rewind();
                    continue;
                }

                var vertexDataSize = nPoints * bytesPerVertexPoint;
                var colorDataSize = nPoints * bytesPerColorPoint;
                Buffer.BlockCopy(frameData, 0, tempVertices, 0, vertexDataSize);
                Buffer.BlockCopy(frameData, vertexDataSize, tempColors, 0, colorDataSize);

                for (var i = 0; i < nPoints; i++) {
                    for (var j = 0; j < 3; j++) {
                        vertices.Add(tempVertices[3 * i + j] / 1000.0f);
                        colors.Add(tempColors[4 * i + j]);
                    }
                }

                _binaryReader.ReadByte();

                _currentFrameIdx++;
                break;
            }
        }

        public void JumpToFrame(int frameIdx) {
            Rewind();
            for (var i = 0; i < frameIdx; i++) {
                var vertices = new List<float>();
                var colors = new List<byte>();
                ReadFrame(vertices, colors);
            }
        }

        public void Rewind() {
            _currentFrameIdx = 0;
            _binaryReader.BaseStream.Seek(0, SeekOrigin.Begin);
        }

        private string ReadLine() {
            var builder = new StringBuilder();
            var buffer = _binaryReader.ReadByte();

            while (buffer != '\n') {
                builder.Append((char) buffer);
                buffer = _binaryReader.ReadByte();
            }

            return builder.ToString();
        }
    }
}