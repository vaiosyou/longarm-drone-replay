// FlightReplayData.cs
// JSON schema produced by python/bin2replay.py (format_version 1.0).
// Field names must match the JSON keys exactly (UnityEngine.JsonUtility).
using System;

namespace FlightReplay
{
    [Serializable]
    public class ReplaySegment
    {
        public int seg;
        public float t_start, t_end;            // replay time [s]
        public double t_src_start, t_src_end;   // source time [s] (time since boot for .bin)
    }

    [Serializable]
    public class ReplayMeta
    {
        public string format_version;
        public string source_file;
        public string position_source;
        public float rate_hz;
        public float max_gap_s;
        public float arm_length_m;
        public float[] arm_mount_frd;           // arm root from body centre [m] (forward, right, down); may be absent in old files
        public double t0_boot_s;
        public string frame;
        public string body_frame;
        public string mapping;
        public int n_frames;
        public float duration_s;
        public float[] origin_unity;
        public bool skip_gaps;                  // true: gaps removed, segments joined back-to-back
        // motors (absent in files made without motor data)
        public int motor_count;
        public int[] motor_channels;
        public string motor_channel_source;
        public string motor_source;
        public float pwm_min, pwm_max;
        public float frame_class, frame_type;
        // geodetic position of Unity (0,0,0): lat [deg], lon [deg], alt [m] (absent if unknown)
        public double[] geo_origin;
        public int n_segments;
        public ReplaySegment[] segments;
    }

    [Serializable]
    public class ReplayFrame
    {
        public float t;                 // [s] from first frame
        public float px, py, pz;        // body position, Unity world [m]
        public float qx, qy, qz, qw;    // body rotation, Unity world
        public float tx, ty, tz;        // arm tip position, Unity world [m]
        public float att_err_deg;       // |desired - actual| attitude [deg], -1 if unavailable
        public int valid;               // 0 = inside a log gap (interpolated, low confidence)
        public int seg;                 // segment index, -1 = inside a gap
        public double t_src;            // source time [s] (time since boot for .bin)
        public float[] pwm;             // raw motor output of motor 1..n (PWM us for ArduPilot RCOU)
    }

    [Serializable]
    public class ReplayLog
    {
        public ReplayMeta meta;
        public ReplayFrame[] frames;
    }
}
