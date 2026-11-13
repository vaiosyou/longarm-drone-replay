// FlightReplayFollowCamera.cs  (optional)
// Put on a Camera. Follows the replayed vehicle at a distance; orbit with right-drag, zoom with the wheel.
// Uses IMGUI events, so it works with both the old Input Manager and the Input System package.
using UnityEngine;

namespace FlightReplay
{
    [RequireComponent(typeof(Camera))]
    public class FlightReplayFollowCamera : MonoBehaviour
    {
        public FlightReplayPlayer player;
        [Tooltip("Follow the arm tip instead of the body centre")]
        public bool followTip = false;
        public float distance = 6f;
        [Range(-10f, 89f)] public float elevationDeg = 25f;
        public float azimuthDeg = -30f;      // 0 = looking north (Unity +Z), measured clockwise
        [Tooltip("Rotate the view with the vehicle heading (chase camera)")]
        public bool followHeading = false;
        public float smoothTime = 0.15f;

        Vector3 _vel;

        void Start()
        {
            if (player == null) player = FindObjectOfType<FlightReplayPlayer>();
        }

        void OnGUI()
        {
            var e = Event.current;
            if (e.type == EventType.MouseDrag && e.button == 1)
            {
                azimuthDeg += e.delta.x * 0.3f;
                elevationDeg = Mathf.Clamp(elevationDeg + e.delta.y * 0.3f, -10f, 89f);
                e.Use();
            }
            else if (e.type == EventType.ScrollWheel)
            {
                distance = Mathf.Clamp(distance * (1f + e.delta.y * 0.05f), 0.5f, 500f);
                e.Use();
            }
        }

        void LateUpdate()
        {
            if (player == null || player.target == null) return;
            Vector3 focus = followTip && player.IsLoaded && player.ArmLength > 0f
                ? player.CurrentTipWorld : player.target.position;

            float az = azimuthDeg;
            if (followHeading)
            {
                Vector3 f = player.target.forward; f.y = 0f;
                if (f.sqrMagnitude > 1e-6f) az += Mathf.Atan2(f.x, f.z) * Mathf.Rad2Deg;
            }
            Quaternion rot = Quaternion.Euler(elevationDeg, az, 0f);
            Vector3 want = focus - rot * Vector3.forward * distance;
            transform.position = Vector3.SmoothDamp(transform.position, want, ref _vel, smoothTime);
            transform.LookAt(focus);
        }
    }
}
