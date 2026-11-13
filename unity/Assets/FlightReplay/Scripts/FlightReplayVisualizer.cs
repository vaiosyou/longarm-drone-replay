// FlightReplayVisualizer.cs
// - Builds a simple arm (cylinder) from the arm root (body centre + meta.arm_mount_frd) to the logged tip.
// - Draws the arm-tip trajectory (LineRenderer). With arm_length 0 the arm is hidden and the trail follows the body.
// - Colors the arm and a tip marker by attitude error |desired - actual| (green -> yellow -> red),
//   grey while the frame lies inside a log gap (valid = 0) or attitude error is unavailable.
using System.Collections.Generic;
using UnityEngine;

namespace FlightReplay
{
    [RequireComponent(typeof(FlightReplayPlayer))]
    public class FlightReplayVisualizer : MonoBehaviour
    {
        [Header("Arm")]
        public bool autoBuildArm = true;
        [Tooltip("Assigned or auto-built arm renderer")]
        public Renderer armRenderer;
        public float armDiameter = 0.02f;

        [Tooltip("When the log has no arm (arm_length 0), colour the body by attitude error instead")]
        public bool colorBodyWhenNoArm = true;

        [Header("Tip marker / trail")]
        public float tipMarkerSize = 0.06f;
        public bool drawTrail = true;
        [Tooltip("Trail length [s] (0 = whole history since last seek)")]
        public float trailSeconds = 10f;
        public float trailWidth = 0.01f;

        [Header("Attitude error colour scale [deg]")]
        public float errGood = 2f;
        public float errBad = 10f;
        public Color invalidColor = Color.gray;

        FlightReplayPlayer _p;
        Transform _tip;
        Transform _arm;
        bool _hasArm;
        Renderer _tipR;
        LineRenderer _trail;
        readonly List<Vector3> _pts = new List<Vector3>();
        readonly List<float> _ts = new List<float>();
        MaterialPropertyBlock _mpb;
        float _lastT = -1f;
        int _lastSeg = int.MinValue;

        void Start()
        {
            _p = GetComponent<FlightReplayPlayer>();
            _mpb = new MaterialPropertyBlock();
            // arm length can be changed in the Inspector at run time -> objects are always created, shown when length > 0
            if (armRenderer == null && autoBuildArm && _p.target != null)
            {
                // Free-standing cylinder, re-placed every frame between body centre and tip
                // (not parented to the model, so model scale cannot distort it).
                var arm = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                arm.name = "ReplayArm";
                Destroy(arm.GetComponent<Collider>());
                armRenderer = arm.GetComponent<Renderer>();
                _arm = arm.transform;
            }

            {
                var tip = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                tip.name = "ReplayArmTip";
                Destroy(tip.GetComponent<Collider>());
                tip.transform.localScale = Vector3.one * tipMarkerSize;
                _tip = tip.transform;
                _tipR = tip.GetComponent<Renderer>();
            }

            if (drawTrail)
            {
                var go = new GameObject("ReplayTipTrail");
                _trail = go.AddComponent<LineRenderer>();
                _trail.widthMultiplier = trailWidth;
                _trail.useWorldSpace = true;
                _trail.material = new Material(Shader.Find("Sprites/Default"));
                _trail.startColor = new Color(0.2f, 0.6f, 1f, 0.1f);
                _trail.endColor = new Color(0.2f, 0.6f, 1f, 1f);
            }
        }

        void LateUpdate()
        {
            if (_p == null || !_p.IsLoaded) return;
            var c = _p.Current;
            _hasArm = _p.ArmLength > 0f;
            if (_tip != null) _tip.gameObject.SetActive(_hasArm);
            if (_arm != null) _arm.gameObject.SetActive(_hasArm);
            // without an arm the trail follows the body centre (tip == body in the data)
            Vector3 tipW = _hasArm ? _p.CurrentTipWorld : _p.target.position;
            if (_tip != null) _tip.position = tipW;

            if (_arm != null && _p.target != null)
            {
                // Cylinder primitive: height 2 along local Y -> scale.y = length/2, rotate Y onto the arm direction
                Vector3 root = _p.CurrentArmRootWorld;   // body centre + arm mount offset
                Vector3 d = tipW - root;
                float len = d.magnitude;
                if (len > 1e-4f)
                {
                    _arm.position = root + d * 0.5f;
                    _arm.rotation = Quaternion.FromToRotation(Vector3.up, d / len);
                    _arm.localScale = new Vector3(armDiameter, len * 0.5f, armDiameter);
                }
            }

            Color col = ErrorColor(c);
            SetColor(armRenderer, col);
            SetColor(_tipR, col);
            if (!_hasArm && colorBodyWhenNoArm && _p.target != null)
                foreach (var r in _p.target.GetComponentsInChildren<Renderer>()) if (r.name != "NoseMarker") SetColor(r, col);

            if (_trail != null) UpdateTrail(c.t, c.seg, tipW);
        }

        Color ErrorColor(ReplayFrame c)
        {
            if (c.valid == 0 || c.att_err_deg < 0) return invalidColor;
            float k = Mathf.InverseLerp(errGood, errBad, c.att_err_deg);
            return k < 0.5f ? Color.Lerp(Color.green, Color.yellow, k * 2f)
                            : Color.Lerp(Color.yellow, Color.red, (k - 0.5f) * 2f);
        }

        void SetColor(Renderer r, Color col)
        {
            if (r == null) return;
            r.GetPropertyBlock(_mpb);
            _mpb.SetColor("_Color", col);      // Built-in / Standard
            _mpb.SetColor("_BaseColor", col);  // URP / HDRP Lit
            r.SetPropertyBlock(_mpb);
        }

        void UpdateTrail(float t, int seg, Vector3 p)
        {
            // reset on seek backwards / loop, and when entering another segment (no line across a removed gap)
            if (t < _lastT || seg != _lastSeg) { _pts.Clear(); _ts.Clear(); }
            _lastT = t;
            _lastSeg = seg;
            if (_pts.Count == 0 || (_pts[_pts.Count - 1] - p).sqrMagnitude > 1e-6f)
            {
                _pts.Add(p); _ts.Add(t);
            }
            if (trailSeconds > 0)
            {
                int drop = 0;
                while (drop < _ts.Count && _ts[drop] < t - trailSeconds) drop++;
                if (drop > 0) { _pts.RemoveRange(0, drop); _ts.RemoveRange(0, drop); }
            }
            _trail.positionCount = _pts.Count;
            _trail.SetPositions(_pts.ToArray());
        }

        void OnDrawGizmos()
        {
            var p = GetComponent<FlightReplayPlayer>();
            if (p == null || !p.IsLoaded || p.target == null || p.ArmLength <= 0f) return;
            // arm line + tip in Scene view
            Gizmos.color = ErrorColor(p.Current);
            Gizmos.DrawLine(p.CurrentArmRootWorld, p.CurrentTipWorld);
            Gizmos.DrawWireSphere(p.CurrentTipWorld, tipMarkerSize);
        }
    }
}
