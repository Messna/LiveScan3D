using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace LiveScanPlayer {
    internal class FrameFileReaderPly : IFrameFileReader {
        private readonly string[] _filenames;
        private int _currentFrameIdx;

        public FrameFileReaderPly(string[] filenames) {
            _filenames = filenames;
        }

        public int FrameIdx {
            get => _currentFrameIdx;
            set => JumpToFrame(value);
        }

        public void ReadFrame(List<float> vertices, List<byte> colors) {
            var reader = new BinaryReader(new FileStream(_filenames[_currentFrameIdx], FileMode.Open));

            var line = ReadLine(reader);
            while (!line.Contains("element vertex"))
                line = ReadLine(reader);
            var lineElems = line.Split(' ');
            var nPoints = int.Parse(lineElems[2]);
            while (!line.Contains("end_header"))
                line = ReadLine(reader);

            for (var i = 0; i < nPoints; i++) {
                for (var j = 0; j < 3; j++) {
                    vertices.Add(reader.ReadSingle());
                }

                for (var j = 0; j < 3; j++) {
                    colors.Add(reader.ReadByte());
                }
            }

            reader.Dispose();

            _currentFrameIdx++;
            if (_currentFrameIdx >= _filenames.Length)
                _currentFrameIdx = 0;
        }

        public void JumpToFrame(int frameIdx) {
            _currentFrameIdx = frameIdx;
            if (_currentFrameIdx >= _filenames.Length)
                _currentFrameIdx = 0;
        }

        public void Rewind() {
            _currentFrameIdx = 0;
        }

        private static string ReadLine(BinaryReader binaryReader) {
            var builder = new StringBuilder();
            var buffer = binaryReader.ReadByte();

            while (buffer != '\n') {
                builder.Append((char) buffer);
                buffer = binaryReader.ReadByte();
            }

            return builder.ToString();
        }
    }
}