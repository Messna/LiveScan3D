using System.Collections.Generic;

namespace LiveScanPlayer {
    internal interface IFrameFileReader {
        int FrameIdx { get; set; }

        void ReadFrame(List<float> vertices, List<byte> colors);

        void JumpToFrame(int frameIdx);

        void Rewind();
    }
}