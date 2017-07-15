﻿//// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF 
//// ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO 
//// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A 
//// PARTICULAR PURPOSE. 
//// 
//// Copyright (c) Microsoft Corporation. All rights reserved.

namespace KinectStudioPlugin
{
    using System;

    public static class HackKStudioEventStreamDataTypeIds
    {
        public static readonly Guid Calibration = new Guid(0x8f78ca9c, 0xc1c0, 0x4673, 0x8e, 0x51, 0x36, 0x48, 0x4f, 0x7e, 0x74, 0x92);
        public static readonly Guid SystemInfo = new Guid(0x0a3914d0, 0x3b16, 0x11e1, 0xaa, 0xc3, 0x00, 0x1e, 0x4f, 0xd5, 0x8c, 0x0f);
        public static readonly Guid ColorSettings = new Guid(0x70b325ba, 0x5819, 0x4c8b, 0x80, 0x37, 0x2e, 0xce, 0x5c, 0xbb, 0x74, 0x67);
        public static readonly Guid SensorTelemetry = new Guid(0x0a3914e9, 0x3b16, 0x11e1, 0xaa, 0xc3, 0x00, 0x1e, 0x4f, 0xd5, 0x8c, 0x0f);
        public static readonly Guid SystemAudio = new Guid(0x9ea644f5, 0x3639, 0x42bc, 0xb3, 0x25, 0xd2, 0xd4, 0x2c, 0xd7, 0x9b, 0x15);
        public static readonly Guid TitleAudio = new Guid(0x787c7abd, 0x9f6e, 0x4a85, 0x8d, 0x67, 0x63, 0x65, 0xff, 0x80, 0xcc, 0x69);
        public static readonly Guid Opaque = new Guid(0xf5280e6e, 0x229c, 0x49ec, 0xa8, 0xd2, 0xb0, 0x6f, 0x7a, 0xd6, 0x43, 0x11);
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "Ir")]
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA1709:IdentifiersShouldBeCasedCorrectly", MessageId = "Ir")]
        public static readonly Guid CommonModeIr = new Guid(0xd6a365d4, 0x46fd, 0x4b4e, 0xb1, 0xfd, 0x5e, 0x28, 0x15, 0xb2, 0x9d, 0x85);
        public static readonly Guid Interaction = new Guid(0x1d162a68, 0xb723, 0x4987, 0xb6, 0x6c, 0x3, 0x5f, 0xd2, 0xb3, 0x2d, 0xa6);
        public static readonly Guid DepthMonitor = new Guid(0x85d9415b, 0x41c9, 0x47f4, 0x8d, 0xd5, 0x88, 0xc3, 0x04, 0x7b, 0xae, 0x6b);
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA1709:IdentifiersShouldBeCasedCorrectly", MessageId = "Ir")]
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "Ir")]
        public static readonly Guid IrMonitor = new Guid(0xbd2018dc, 0xcc6b, 0x4543, 0xac, 0x31, 0x84, 0xd4, 0x00, 0x7b, 0x16, 0x24);
        public static readonly Guid CalibrationMonitor = new Guid(0x682d8a1c, 0xc301, 0x4b73, 0xb3, 0x4d, 0xa9, 0x66, 0x54, 0x95, 0x3d, 0x22);
        public static readonly Guid BodyMonitor = new Guid(0x1c32b14a, 0x1275, 0x496f, 0xb4, 0x35, 0xf6, 0xf2, 0x2d, 0x45, 0x18, 0x85);
        public static readonly Guid BodyIndexMonitor = new Guid(0x37f1135c, 0x9139, 0x444d, 0xb4, 0x0d, 0x82, 0x7f, 0xe4, 0xc6, 0x57, 0x10);
        public static readonly Guid CompressedColorMonitor = new Guid(0x69720fe3, 0x3691, 0x4eab, 0x88, 0xd3, 0xf9, 0xbd, 0x86, 0x6b, 0x85, 0x21);
        public static readonly Guid SystemAudioMonitor = new Guid(0x66d3e38b, 0x8799, 0x477c, 0x8b, 0xc6, 0xc3, 0xec, 0x6e, 0x20, 0x07, 0x37);
        public static readonly Guid TitleAudioMonitor = new Guid(0x0286c131, 0x84fc, 0x4119, 0x90, 0x5e, 0x8d, 0x9b, 0xef, 0x44, 0x42, 0x75);
        public static readonly Guid UncompressedColorMonitor = new Guid(0xeb8f4b9b, 0x48b4, 0x4869, 0xba, 0xc9, 0xb4, 0xa4, 0xa5, 0x1d, 0x89, 0x3d);
    }
}
