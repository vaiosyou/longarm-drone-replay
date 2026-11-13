// FlightReplayTelemetry.cs  (optional)
// On-screen flight data of the current replay time (top-left):
//   position N/E, altitude, ground / vertical speed, course, roll / pitch / yaw, attitude error,
//   arm-tip position and height, log time.
// Values are converted back from Unity (x=East, y=Up, z=North) to the aerospace convention:
//   yaw   = atan2(f.x, f.z)          (0 = North, 90 = East)
//   pitch = asin(f.y)                (+ = nose up)
//   roll  = atan2(-r.y, u.y)         (+ = right side down)
// where f, r, u are the body forward / right / up axes. Positions are relative to the replay origin
// (the first valid sample unless --no-origin-shift was used).
using UnityEngine;

namespace FlightReplay
{
    [RequireComponent(typeof(FlightReplayPlayer))]
    public class FlightReplayTelemetry : MonoBehaviour
    {
        public bool show = true;
        [Tooltip("Also show the altitude relative to the source origin (EKF origin for ArduPilot)")]
        public bool showSourceAltitude = true;

        FlightReplayPlayer _p;
        GUIStyle _mono;

        void Awake() { _p = GetComponent<FlightReplayPlayer>(); }

        /// <summary>Aerospace roll, pitch, yaw [deg] (yaw 0..360) of a rotation in Unity axes.</summary>
        public static Vector3 RollPitchYaw(Quaternion q)
        {
            Vector3 f = q * Vector3.forward, r = q * Vector3.right, u = q * Vector3.up;
            float yaw = Mathf.Atan2(f.x, f.z) * Mathf.Rad2Deg;
            if (yaw < 0) yaw += 360f;
            float pitch = Mathf.Asin(Mathf.Clamp(f.y, -1f, 1f)) * Mathf.Rad2Deg;
            float roll = Mathf.Atan2(-r.y, u.y) * Mathf.Rad2Deg;
            return new Vector3(roll, pitch, yaw);
        }

        void OnGUI()
        {
            if (!show || _p == null || !_p.IsLoaded) return;
            if (_mono == null)
            {
                _mono = new GUIStyle(GUI.skin.label) { richText = true };
                var fnt = Font.CreateDynamicFontFromOSFont(new[] { "Consolas", "Menlo", "DejaVu Sans Mono", "Courier New" }, 13);
                if (fnt != null) _mono.font = fnt;
            }
            float s = FlightReplayUI.CurrentScale;
            Matrix4x4 old = GUI.matrix;
            GUI.matrix = Matrix4x4.Scale(new Vector3(s, s, 1f));

            var c = _p.Current;
            Vector3 p = _p.CurrentPosLocal, v = _p.CurrentVelLocal;
            Vector3 rpy = RollPitchYaw(_p.CurrentRotLocal);
            float gs = new Vector2(v.x, v.z).magnitude;
            float course = gs > 0.2f ? (Mathf.Atan2(v.x, v.z) * Mathf.Rad2Deg + 360f) % 360f : float.NaN;

            // arm tip in replay-origin space (no Height Offset)
            Vector3 tip = transform.InverseTransformPoint(_p.CurrentTipWorld) - Vector3.up * _p.heightOffset;
            bool arm = _p.ArmLength > 0f;

            string srcAlt = "";
            var o = _p.Log.meta.origin_unity;
            if (showSourceAltitude && o != null && o.Length == 3)
                srcAlt = $"\n  (src origin  {p.y + o[1],8:F2} m)";

            string err = c.att_err_deg < 0 ? "   n/a" : $"{c.att_err_deg,6:F1}";
            string txt =
                $"<b>FLIGHT DATA</b>   log t {c.t_src,8:F2} s{(c.valid == 1 ? "" : "  <color=#ff8>[GAP]</color>")}\n" +
                $"North   {p.z,8:F2} m    East  {p.x,8:F2} m\n" +
                $"Alt     {p.y,8:F2} m{srcAlt}\n" +
                $"GndSpd  {gs,8:F2} m/s  Course {(float.IsNaN(course) ? "   -" : course.ToString("F0").PadLeft(4))}°\n" +
                $"VertSpd {v.y,8:F2} m/s\n" +
                $"Roll    {rpy.x,8:F1}°    Pitch {rpy.y,8:F1}°\n" +
                $"Yaw     {rpy.z,8:F1}°    AttErr{err}°" +
                (arm ? $"\n<b>ARM TIP</b>  N {tip.z,7:F2}  E {tip.x,7:F2}  Alt {tip.y,7:F2} m" : "");

            int lines = 8 + (arm ? 1 : 0) + (srcAlt.Length > 0 ? 1 : 0);
            Rect box = new Rect(6f, 6f, 330f, 10f + lines * 17f);
            GUI.Box(box, GUIContent.none);
            GUI.Label(new Rect(box.x + 8, box.y + 4, box.width - 12, box.height - 6), txt, _mono);
            GUI.matrix = old;
        }
    }
}
