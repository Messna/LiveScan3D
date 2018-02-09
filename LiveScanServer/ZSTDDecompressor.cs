using System;
using System.Runtime.InteropServices;
using size_t = System.UIntPtr;

namespace KinectServer {
    internal static class ZstdDecompressor {
        private const string DllName = "libzstd.dll";

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern ulong ZSTD_getDecompressedSize(IntPtr src, size_t srcSize);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern size_t ZSTD_decompress(IntPtr dst, size_t dstSize,
                                                    IntPtr src, size_t srcSize);

        public static byte[] Decompress(byte[] array) {
            var size = array.Length;
            var ptr = Marshal.AllocHGlobal(size);
            Marshal.Copy(array, 0, ptr, size);

            var outSize = (int) ZSTD_getDecompressedSize(ptr, (size_t) size);

            var outPtr = Marshal.AllocHGlobal(outSize);
            ZSTD_decompress(outPtr, (size_t) outSize, ptr, (size_t) size);

            var outArray = new byte[outSize];
            Marshal.Copy(outPtr, outArray, 0, outSize);

            Marshal.FreeHGlobal(ptr);
            Marshal.FreeHGlobal(outPtr);
            return outArray;
        }
    }
}