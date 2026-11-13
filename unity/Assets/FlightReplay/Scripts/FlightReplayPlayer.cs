// FlightReplayPlayer.cs
// Plays back a replay JSON (python/bin2replay.py) by driving a Transform.
// Coordinate conversion NED -> Unity is already done in Python; values are applied as-is
// in the local space of this GameObject ("ReplayOrigin").
//
// Works in any Unity scene; no Hakoniwa (or other) runtime API is used.
//
// Scene layout:
//   ReplayOrigin  (this script)           <- world anchor of the log origin
//     └ DroneRoot (assign to 'target')    <- any object (model, cube, sphere...); its +Z is the nose
// If 'target' is empty, a placeholder body (box / sphere / capsule) is created automatically.
//
// In a Hakoniwa project, disable the Hakoniwa component that writes the same Transform
// (the WebSocket/PDU avatar script) while replaying.
using System.IO;
using UnityEngine;

namespace FlightReplay
{
    public enum PlaceholderShape { Box, Sphere, Capsule }

    public class FlightReplayPlayer : MonoBehaviour
    {
        [Header("Data")]
        [Tooltip("File name under Assets/StreamingAssets, or an absolute path")]
        public string jsonPath = "flight_replay.json";
        [Tooltip("Optional: use a TextAsset instead of a file (takes priority if set)")]
        public TextAsset jsonAsset;

        [Header("Target")]
        public Transform target;
        [Tooltip("Extra rotation applied to the model if its nose is not +Z (e.g. (0,90,0))")]
        public Vector3 modelEulerOffset = Vector3.zero;
        [Tooltip("Vertical offset so the model does not sink into the ground [m]")]
        public float heightOffset = 0f;

        [Header("Arm (computed in Unity from these values)")]
        [Tooltip("ON: tip = body + rotation * (mount + forward * length) using the values below.\nOFF: use the tip baked into the JSON by Python (--arm-length / --arm-mount).")]
        public bool armFromInspector = true;
        [Tooltip("Arm length [m] (0 = no arm)")]
        public float armLength = 1.5f;
        [Tooltip("Arm root from the body centre [m] in the vehicle's axes: x = right, y = up, z = forward.\nSame order as a child object's Position when the model nose is +Z.")]
        public Vector3 armMountLocal = Vector3.zero;
        [Tooltip("Optional: drag the arm-root object of your model (e.g. ArmBase). Its position is used instead of Arm Mount Local and follows the model.")]
        public Transform armMountReference;

        [Header("Placeholder (used only when Target is empty)")]
        public PlaceholderShape placeholderShape = PlaceholderShape.Box;
        [Tooltip("Body size [m] (x = width, y = height, z = length). Default ~ F450 class")]
        public Vector3 placeholderSize = new Vector3(0.45f, 0.12f, 0.45f);
        public Color placeholderColor = new Color(0.85f, 0.85f, 0.9f);
        [Tooltip("Add a small red marker on the nose so the heading is visible")]
        public bool placeholderNoseMarker = true;

        [Header("Playback")]
        public bool playOnStart = true;
        public bool loop = true;
        [Range(0.05f, 16f)] public float speed = 1f;

        public ReplayLog Log { get; private set; }
        public bool IsLoaded => Log != null && Log.frames != null && Log.frames.Length > 1;
        public bool HasMotors => IsLoaded && Log.frames[0].pwm != null && Log.frames[0].pwm.Length > 0;
        public bool IsPlaying { get; private set; }
        public float CurrentTime { get; private set; }
        public float Duration => IsLoaded ? Log.frames[Log.frames.Length - 1].t : 0f;

        /// <summary>Interpolated state of the current time (read by visualizer/UI).</summary>
        public ReplayFrame Current { get; private set; } = new ReplayFrame();
        public Vector3 CurrentTipLocal { get; private set; }
        /// <summary>Body position in ReplayOrigin space, as logged (x=East, y=Up, z=North) [m], without Height Offset.</summary>
        public Vector3 CurrentPosLocal { get; private set; }
        /// <summary>Body velocity in ReplayOrigin space [m/s] (finite difference of the resampled log).</summary>
        public Vector3 CurrentVelLocal { get; private set; }
        /// <summary>Body rotation in ReplayOrigin space (log value, without the model offset).</summary>
        public Quaternion CurrentRotLocal { get; private set; } = Quaternion.identity;
        /// <summary>Body (log) rotation in world space, without the model offset.</summary>
        public Quaternion CurrentBodyRotationWorld { get; private set; } = Quaternion.identity;

        /// <summary>Arm length actually used [m].</summary>
        public float ArmLength => armFromInspector ? armLength : (IsLoaded ? Log.meta.arm_length_m : 0f);

        /// <summary>Arm root in body axes (x=Right, y=Up, z=Forward) [m].</summary>
        public Vector3 ArmMountUnityBody
        {
            get
            {
                if (armFromInspector)
                {
                    if (armMountReference != null && target != null)
                    {
                        // unscaled offset in the target's axes, then remove the model offset -> body axes
                        Vector3 inTarget = Quaternion.Inverse(target.rotation) * (armMountReference.position - target.position);
                        return Quaternion.Euler(modelEulerOffset) * inTarget;
                    }
                    return armMountLocal;
                }
                var m = IsLoaded ? Log.meta.arm_mount_frd : null;
                return (m != null && m.Length == 3) ? new Vector3(m[1], -m[2], m[0]) : Vector3.zero;
            }
        }

        /// <summary>Arm root in world space.</summary>
        public Vector3 CurrentArmRootWorld => (target != null ? target.position : transform.position)
                                              + CurrentBodyRotationWorld * ArmMountUnityBody;

        Quaternion _offset;

        void Awake()
        {
            _offset = Quaternion.Euler(modelEulerOffset);
            Load();
            if (target == null) target = CreatePlaceholder();
            if (target != null)
            {
                var rb = target.GetComponent<Rigidbody>();
                if (rb != null) rb.isKinematic = true;   // physics must not fight the replay
            }
        }

        void Start()
        {
            if (IsLoaded) { Seek(0f); IsPlaying = playOnStart; }
        }

        Transform CreatePlaceholder()
        {
            var root = new GameObject("ReplayPlaceholder").transform;
            root.SetParent(transform, false);
            PrimitiveType pt = placeholderShape == PlaceholderShape.Sphere ? PrimitiveType.Sphere
                             : placeholderShape == PlaceholderShape.Capsule ? PrimitiveType.Capsule
                             : PrimitiveType.Cube;
            var body = GameObject.CreatePrimitive(pt);
            body.name = "Body";
            Destroy(body.GetComponent<Collider>());
            body.transform.SetParent(root, false);
            Vector3 sz = placeholderSize;
            if (pt == PrimitiveType.Capsule)
            {
                // capsule: height 2 along Y -> lay it along +Z (nose)
                body.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
                sz = new Vector3(placeholderSize.x, placeholderSize.z * 0.5f, placeholderSize.y);
            }
            body.transform.localScale = sz;
            body.GetComponent<Renderer>().material.color = placeholderColor;

            if (placeholderNoseMarker)
            {
                var nose = GameObject.CreatePrimitive(PrimitiveType.Cube);
                nose.name = "NoseMarker";
                Destroy(nose.GetComponent<Collider>());
                nose.transform.SetParent(root, false);
                float m = Mathf.Max(0.03f, placeholderSize.y * 0.5f);
                nose.transform.localScale = Vector3.one * m;
                nose.transform.localPosition = new Vector3(0f, 0f, placeholderSize.z * 0.5f + m * 0.5f);
                nose.GetComponent<Renderer>().material.color = Color.red;
            }
            modelEulerOffset = Vector3.zero;   // placeholder nose is +Z by construction
            _offset = Quaternion.identity;
            return root;
        }

        public void Load()
        {
            string text = null;
            if (jsonAsset != null) text = jsonAsset.text;
            else
            {
                string p = Path.IsPathRooted(jsonPath) ? jsonPath : Path.Combine(Application.streamingAssetsPath, jsonPath);
                if (!File.Exists(p)) { Debug.LogError($"[FlightReplay] JSON not found: {p}"); return; }
                text = File.ReadAllText(p);
            }
            Log = JsonUtility.FromJson<ReplayLog>(text);
            if (!IsLoaded) { Debug.LogError("[FlightReplay] JSON parsed but no frames."); Log = null; return; }
            Debug.Log($"[FlightReplay] loaded {Log.frames.Length} frames, {Duration:F1} s, rate {Log.meta.rate_hz} Hz, src={Log.meta.source_file}");
        }

        void Update()
        {
            if (!IsLoaded || !IsPlaying) return;
            float t = CurrentTime + Time.deltaTime * speed;
            if (t > Duration)
            {
                if (loop) t = 0f;
                else { t = Duration; IsPlaying = false; }
            }
            Seek(t);
        }

        public void Play() { if (IsLoaded) IsPlaying = true; }
        public void Pause() { IsPlaying = false; }
        public void TogglePlay() { if (IsPlaying) Pause(); else Play(); }
        public void Step(float seconds) { Seek(Mathf.Clamp(CurrentTime + seconds, 0f, Duration)); }

        public int SegmentCount => IsLoaded && Log.meta.segments != null ? Log.meta.segments.Length : 0;

        /// <summary>Jump to the start of the next (+1) or previous (-1) segment.</summary>
        public void JumpSegment(int dir)
        {
            if (SegmentCount == 0) return;
            var segs = Log.meta.segments;
            if (dir > 0)
            {
                foreach (var sg in segs) if (sg.t_start > CurrentTime + 1e-3f) { Seek(sg.t_start); return; }
            }
            else
            {
                // previous: start of the current segment if we are >1 s into it, else the one before
                for (int k = segs.Length - 1; k >= 0; k--)
                    if (segs[k].t_start < CurrentTime - 1f) { Seek(segs[k].t_start); return; }
                Seek(segs[0].t_start);
            }
        }

        /// <summary>Move to time t [s] and apply the interpolated pose.</summary>
        public void Seek(float t)
        {
            if (!IsLoaded) return;
            var f = Log.frames;
            CurrentTime = Mathf.Clamp(t, 0f, Duration);

            // frames are uniformly resampled -> O(1) index lookup
            float rate = Log.meta.rate_hz > 0 ? Log.meta.rate_hz : (f.Length - 1) / Mathf.Max(Duration, 1e-6f);
            int i = Mathf.Clamp(Mathf.FloorToInt(CurrentTime * rate), 0, f.Length - 2);
            ReplayFrame a = f[i], b = f[i + 1];
            float u = Mathf.Clamp01((CurrentTime - a.t) / Mathf.Max(b.t - a.t, 1e-6f));
            if (a.seg != b.seg) u = u < 0.5f ? 0f : 1f;   // never blend across a segment boundary (joined gap)

            Vector3 pos = Vector3.Lerp(new Vector3(a.px, a.py, a.pz), new Vector3(b.px, b.py, b.pz), u);
            Quaternion rot = Quaternion.Slerp(new Quaternion(a.qx, a.qy, a.qz, a.qw),
                                              new Quaternion(b.qx, b.qy, b.qz, b.qw), u);
            CurrentTipLocal = Vector3.Lerp(new Vector3(a.tx, a.ty, a.tz), new Vector3(b.tx, b.ty, b.tz), u)
                              + Vector3.up * heightOffset;

            var c = Current;
            c.t = CurrentTime;
            c.att_err_deg = (a.att_err_deg < 0 || b.att_err_deg < 0) ? -1f : Mathf.Lerp(a.att_err_deg, b.att_err_deg, u);
            c.valid = (a.valid == 1 && b.valid == 1) ? 1 : 0;
            ReplayFrame near = u < 0.5f ? a : b;
            if (a.pwm != null && b.pwm != null && a.pwm.Length > 0 && a.pwm.Length == b.pwm.Length)
            {
                if (c.pwm == null || c.pwm.Length != a.pwm.Length) c.pwm = new float[a.pwm.Length];
                for (int k = 0; k < a.pwm.Length; k++) c.pwm[k] = Mathf.Lerp(a.pwm[k], b.pwm[k], u);
            }
            c.seg = near.seg;
            c.t_src = a.t_src + (b.t_src - a.t_src) * u;

            CurrentBodyRotationWorld = transform.rotation * rot;
            CurrentPosLocal = pos;
            CurrentRotLocal = rot;
            // velocity from the neighbouring frames of the same segment
            ReplayFrame va = a, vb = b;
            if (a.seg != b.seg && i > 0) { va = f[i - 1]; vb = a; }
            float vdt = vb.t - va.t;
            CurrentVelLocal = (vdt > 1e-6f && va.seg == vb.seg)
                ? new Vector3(vb.px - va.px, vb.py - va.py, vb.pz - va.pz) / vdt : Vector3.zero;
            if (target != null)
            {
                // Apply in ReplayOrigin's local space
                target.position = transform.TransformPoint(pos + Vector3.up * heightOffset);
                target.rotation = transform.rotation * rot * _offset;
            }
        }

        /// <summary>Tip position in world space (for visualizers).</summary>
        public Vector3 CurrentTipWorld => armFromInspector
            ? CurrentArmRootWorld + CurrentBodyRotationWorld * (Vector3.forward * armLength)
            : transform.TransformPoint(CurrentTipLocal);
    }
}
