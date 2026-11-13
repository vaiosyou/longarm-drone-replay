// FlightReplayPropellers.cs  (optional)
// Spins propeller objects and shows RPM / thrust computed from the logged motor output (PWM),
// using the user's own calibration entered in the Inspector:
//   RPM    = A * PWM + B                     (0 below 'Spin Min PWM', clamped at 0)
//   Thrust = one of
//      KtRpm2      : T = kT * rpm^2                     [kT in N/rpm^2]
//      KtOmega2    : T = kT * omega^2,  omega = rpm*2pi/60   [kT in N/(rad/s)^2]
//      CtRhoN2D4   : T = Ct * rho * n^2 * D^4, n = rpm/60     [Ct dimensionless, rho kg/m^3, D m]
// The log holds the COMMANDED output (ArduPilot RCOU), not measured RPM; RPM and thrust shown here are
// estimates that are only as good as the calibration.
//
// Real propellers turn far faster than the frame rate can show, so the visual spin is scaled
// (Visual Speed Scale) and capped (Visual Max Rev Per Sec); the numbers on screen are not scaled.
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;

namespace FlightReplay
{
    [RequireComponent(typeof(FlightReplayPlayer))]
    public class FlightReplayPropellers : MonoBehaviour
    {
        public enum ThrustModel { KtRpm2, KtOmega2, CtRhoN2D4 }

        [Serializable]
        public class Prop
        {
            public Transform transform;
            [Tooltip("Motor number in the log (1-based)")]
            public int motor = 1;
            [Tooltip("Counter-clockwise seen from above")]
            public bool ccw = true;
            [Tooltip("Spin axis in the propeller's local space")]
            public Vector3 localAxis = Vector3.up;
        }

        [Header("Propellers")]
        public List<Prop> props = new List<Prop>();
        [Tooltip("Name filter used by 'Auto-assign' (objects under Target whose name contains this)")]
        public string nameFilter = "Pivot";

        [Header("PWM -> RPM   (RPM = A * PWM + B)")]
        public float rpmPerPwm_A = 10f;
        public float rpmOffset_B = -10000f;
        [Tooltip("Below this PWM the motor is treated as stopped")]
        public float spinMinPwm = 1050f;

        [Header("Thrust")]
        public ThrustModel thrustModel = ThrustModel.KtRpm2;
        [Tooltip("Thrust coefficient kT.  KtRpm2: N/rpm^2   KtOmega2: N/(rad/s)^2")]
        [FormerlySerializedAs("kT")] public double kt = 1.0e-7;
        [Tooltip("CtRhoN2D4: thrust coefficient (dimensionless)")]
        public float ct = 0.1f;
        [Tooltip("Air density [kg/m^3]")]
        public float airDensity = 1.225f;
        [Tooltip("Propeller diameter [m] (10 in = 0.254 m)")]
        public float propDiameter = 0.254f;

        [Header("Display")]
        [Tooltip("Visual rev/s = real rev/s * this")]
        public float visualSpeedScale = 0.1f;
        public float visualMaxRevPerSec = 15f;
        public bool showThrustArrows = true;
        [Tooltip("Arrow length per newton [m/N]")]
        public float arrowMetersPerNewton = 0.05f;
        public float arrowWidth = 0.012f;
        public bool showPanel = true;

        FlightReplayPlayer _p;
        readonly List<LineRenderer> _arrows = new List<LineRenderer>();

        /// <summary>Latest values per prop entry (same order as 'props').</summary>
        public float[] Rpm { get; private set; } = new float[0];
        public float[] Thrust { get; private set; } = new float[0];

        void Start()
        {
            _p = GetComponent<FlightReplayPlayer>();
            // empty list, or rows added with "+" in the Inspector (no transform / motor 0) -> assign automatically
            if (props.Count == 0 || props.Exists(pr => pr == null || pr.transform == null || pr.motor < 1))
            {
                if (props.Count > 0) Debug.Log("[FlightReplay] Propellers: list has empty rows -> auto-assigning.");
                AutoAssign();
            }
            BuildArrows();
        }

        public float PwmToRpm(float pwm) => pwm < spinMinPwm ? 0f : Mathf.Max(0f, rpmPerPwm_A * pwm + rpmOffset_B);

        public float RpmToThrust(float rpm)
        {
            switch (thrustModel)
            {
                case ThrustModel.KtOmega2:
                    double w = rpm * 2.0 * Math.PI / 60.0;
                    return (float)(kt * w * w);
                case ThrustModel.CtRhoN2D4:
                    double n = rpm / 60.0, d2 = (double)propDiameter * propDiameter;
                    return (float)(ct * airDensity * n * n * d2 * d2);
                default:
                    return (float)(kt * rpm * rpm);
            }
        }

        void LateUpdate()
        {
            if (_p == null || !_p.HasMotors) return;
            float[] pwm = _p.Current.pwm;
            if (pwm == null) return;
            if (Rpm.Length != props.Count) { Rpm = new float[props.Count]; Thrust = new float[props.Count]; }

            // visual spin follows the replay clock (paused -> stopped, x2 -> twice as fast)
            float dt = _p.IsPlaying ? Time.deltaTime * _p.speed : 0f;
            for (int i = 0; i < props.Count; i++)
            {
                var pr = props[i];
                int m = pr.motor - 1;
                float rpm = (m >= 0 && m < pwm.Length) ? PwmToRpm(pwm[m]) : 0f;
                Rpm[i] = rpm;
                Thrust[i] = RpmToThrust(rpm);
                if (pr.transform == null) continue;

                float rev = Mathf.Min(rpm / 60f * visualSpeedScale, visualMaxRevPerSec);
                // Unity is left-handed: +angle about +Y is clockwise seen from above
                float deg = rev * 360f * dt * (pr.ccw ? -1f : 1f);
                Vector3 axis = pr.localAxis.sqrMagnitude > 1e-6f ? pr.localAxis.normalized : Vector3.up;
                if (deg != 0f) pr.transform.Rotate(axis, deg, Space.Self);

                if (i < _arrows.Count && _arrows[i] != null)
                {
                    var lr = _arrows[i];
                    lr.enabled = showThrustArrows;
                    if (showThrustArrows)
                    {
                        Vector3 o = pr.transform.position;
                        Vector3 up = pr.transform.TransformDirection(axis);
                        lr.SetPosition(0, o);
                        lr.SetPosition(1, o + up * Thrust[i] * arrowMetersPerNewton);
                    }
                }
            }
        }

        void BuildArrows()
        {
            foreach (var a in _arrows) if (a != null) Destroy(a.gameObject);
            _arrows.Clear();
            var mat = new Material(Shader.Find("Sprites/Default"));
            for (int i = 0; i < props.Count; i++)
            {
                var go = new GameObject($"ThrustArrow_M{props[i].motor}");
                var lr = go.AddComponent<LineRenderer>();
                lr.positionCount = 2;
                lr.useWorldSpace = true;
                lr.widthMultiplier = arrowWidth;
                lr.material = mat;
                lr.startColor = new Color(1f, 0.6f, 0.1f, 0.9f);
                lr.endColor = new Color(1f, 0.9f, 0.2f, 0.9f);
                _arrows.Add(lr);
            }
        }

        /// <summary>
        /// Finds objects under Target whose name contains 'nameFilter' and assigns motor numbers by position,
        /// using the ArduPilot Quad-X order [要出典確認: ArduPilot motor order diagram]:
        ///   M1 front-right CCW, M2 rear-left CCW, M3 front-left CW, M4 rear-right CW.
        /// For other frames, edit the list by hand.
        /// </summary>
        [ContextMenu("Auto-assign (ArduPilot Quad X)")]
        public void AutoAssign()
        {
            if (_p == null) _p = GetComponent<FlightReplayPlayer>();
            if (_p == null || _p.target == null) { Debug.LogWarning("[FlightReplay] Propellers: Target is empty."); return; }
            var found = new List<Transform>();
            foreach (var t in _p.target.GetComponentsInChildren<Transform>(true))
                if (t != _p.target && t.name.Contains(nameFilter)) found.Add(t);
            if (found.Count != 4)
            {
                Debug.LogWarning($"[FlightReplay] Propellers: found {found.Count} objects containing '{nameFilter}' (need 4 for Quad X). Assign by hand.");
                return;
            }
            Quaternion toBody = Quaternion.Euler(_p.modelEulerOffset) * Quaternion.Inverse(_p.target.rotation);
            Vector3 centre = Vector3.zero;
            foreach (var t in found) centre += t.position;
            centre /= found.Count;
            props.Clear();
            foreach (var t in found)
            {
                Vector3 b = toBody * (t.position - centre);       // x = right, z = forward
                bool front = b.z > 0f, right = b.x > 0f;
                int motor = front ? (right ? 1 : 3) : (right ? 4 : 2);
                props.Add(new Prop { transform = t, motor = motor, ccw = (motor == 1 || motor == 2) });
            }
            props.Sort((a, c) => a.motor.CompareTo(c.motor));
            string msg = "";
            foreach (var pr in props) msg += $"M{pr.motor}={pr.transform.name}({(pr.ccw ? "CCW" : "CW")}) ";
            Debug.Log("[FlightReplay] Propellers auto-assigned: " + msg);
            if (Application.isPlaying) BuildArrows();
        }

        void OnGUI()
        {
            if (!showPanel || _p == null || !_p.IsLoaded) return;
            float s = FlightReplayUI.CurrentScale;
            Matrix4x4 old = GUI.matrix;
            GUI.matrix = Matrix4x4.Scale(new Vector3(s, s, 1f));
            float w = 330f;
            float h = 44f + 20f * Mathf.Max(1, props.Count);
            GUILayout.BeginArea(new Rect(Screen.width / s - w - 6f, 6f, w, h), GUI.skin.box);
            if (!_p.HasMotors)
            {
                GUILayout.Label("Motors: no PWM in this replay file");
            }
            else
            {
                float pwmPrint(int m) => (_p.Current.pwm != null && m >= 0 && m < _p.Current.pwm.Length) ? _p.Current.pwm[m] : float.NaN;
                GUILayout.Label("motor   PWM     RPM(est)   thrust(est)");
                float total = 0f;
                for (int i = 0; i < props.Count && i < Rpm.Length; i++)
                {
                    total += Thrust[i];
                    GUILayout.Label($"M{props[i].motor} {(props[i].ccw ? "CCW" : "CW ")}  {pwmPrint(props[i].motor - 1),6:F0}  {Rpm[i],8:F0}   {Thrust[i],7:F2} N");
                }
                GUILayout.Label($"total thrust {total:F2} N  ({total / 9.80665f:F3} kgf)");
            }
            GUILayout.EndArea();
            GUI.matrix = old;
        }
    }
}
