// FlightReplayUI.cs
// Minimal IMGUI controller: play/pause, speed, time slider, step, segment jump.
// IMGUI needs no Canvas / UI package and works with both the old Input Manager and the Input System.
// The panel is laid out for a virtual width of 'designWidth' px and scaled down to fit small Game views.
// Keys (Game view focused): Space = play/pause, Left/Right = -/+1 s, Up/Down = speed x2 / x0.5,
//                           PageUp/PageDown = previous/next segment, H = show/hide panel
using UnityEngine;

namespace FlightReplay
{
    [RequireComponent(typeof(FlightReplayPlayer))]
    public class FlightReplayUI : MonoBehaviour
    {
        public enum Anchor { Bottom, Top }

        public bool show = true;
        public Anchor anchor = Anchor.Bottom;
        [Tooltip("Panel is designed for this width [px]; narrower Game views scale it down")]
        public float designWidth = 760f;
        [Tooltip("Extra scale (e.g. 1.5 on high-DPI screens)")]
        [Range(0.5f, 3f)] public float uiScale = 1f;

        static readonly float[] Speeds = { 0.1f, 0.25f, 0.5f, 1f, 2f, 4f, 8f };
        FlightReplayPlayer _p;

        /// <summary>Scale actually applied this frame (shared with other IMGUI panels of this package).</summary>
        public static float CurrentScale { get; private set; } = 1f;

        void Awake() { _p = GetComponent<FlightReplayPlayer>(); }

        public static float ComputeScale(float designWidth, float uiScale)
        {
            float s = uiScale;
            if (Screen.width < designWidth * s) s = Screen.width / designWidth;
            return Mathf.Max(0.3f, s);
        }

        void OnGUI()
        {
            if (_p == null) return;
            HandleKeys();
            if (!show)
            {
                GUI.Label(new Rect(6, anchor == Anchor.Bottom ? Screen.height - 22 : 4, 200, 20), "[H] replay panel");
                return;
            }

            float s = ComputeScale(designWidth, uiScale);
            CurrentScale = s;
            Matrix4x4 old = GUI.matrix;
            GUI.matrix = Matrix4x4.Scale(new Vector3(s, s, 1f));
            float W = Screen.width / s, H = Screen.height / s;   // virtual screen size
            const float h = 92f;
            float y = anchor == Anchor.Bottom ? H - h - 6f : 6f;

            GUILayout.BeginArea(new Rect(6, y, W - 12, h), GUI.skin.box);
            if (!_p.IsLoaded)
            {
                GUILayout.Label("FlightReplay: no data loaded (see Console)");
                GUILayout.EndArea();
                GUI.matrix = old;
                return;
            }

            // row 1: transport + speed
            GUILayout.BeginHorizontal();
            if (GUILayout.Button(_p.IsPlaying ? "Pause" : "Play", GUILayout.Width(60))) _p.TogglePlay();
            if (GUILayout.Button("|<", GUILayout.Width(30))) _p.Seek(0f);
            if (GUILayout.Button("-1s", GUILayout.Width(38))) _p.Step(-1f);
            if (GUILayout.Button("+1s", GUILayout.Width(38))) _p.Step(1f);
            if (_p.SegmentCount > 1)
            {
                if (GUILayout.Button("<seg", GUILayout.Width(44))) _p.JumpSegment(-1);
                if (GUILayout.Button("seg>", GUILayout.Width(44))) _p.JumpSegment(1);
            }
            GUILayout.Space(6);
            foreach (var sp in Speeds)
                if (GUILayout.Toggle(Mathf.Approximately(_p.speed, sp), $"x{sp:g}", GUI.skin.button, GUILayout.Width(42)))
                    _p.speed = sp;
            GUILayout.Space(6);
            _p.loop = GUILayout.Toggle(_p.loop, "Loop", GUILayout.Width(52));
            GUILayout.FlexibleSpace();
            if (GUILayout.Button("Hide [H]", GUILayout.Width(70))) show = false;
            GUILayout.EndHorizontal();

            // row 2: status
            var c = _p.Current;
            string err = c.att_err_deg < 0 ? "n/a" : $"{c.att_err_deg:F1}°";
            string seg = _p.SegmentCount > 0 ? (c.seg >= 0 ? $"seg {c.seg + 1}/{_p.SegmentCount}" : "seg -") : "";
            GUILayout.Label($"t {_p.CurrentTime:F2}/{_p.Duration:F1} s   log t {c.t_src:F2} s   {seg}   att.err {err}   " +
                            $"L {_p.ArmLength:0.###} m ({(_p.armFromInspector ? "Inspector" : "JSON")})   {(c.valid == 1 ? "" : "[GAP]")}");

            // row 3: time slider
            float nt = GUILayout.HorizontalSlider(_p.CurrentTime, 0f, _p.Duration);
            if (!Mathf.Approximately(nt, _p.CurrentTime)) _p.Seek(nt);
            GUILayout.EndArea();
            GUI.matrix = old;
        }

        void HandleKeys()
        {
            var e = Event.current;
            if (e == null || e.type != EventType.KeyDown) return;
            switch (e.keyCode)
            {
                case KeyCode.H: show = !show; e.Use(); break;
                case KeyCode.Space: _p.TogglePlay(); e.Use(); break;
                case KeyCode.LeftArrow: _p.Step(-1f); e.Use(); break;
                case KeyCode.RightArrow: _p.Step(1f); e.Use(); break;
                case KeyCode.UpArrow: _p.speed = Mathf.Min(_p.speed * 2f, 16f); e.Use(); break;
                case KeyCode.PageUp: _p.JumpSegment(-1); e.Use(); break;
                case KeyCode.PageDown: _p.JumpSegment(1); e.Use(); break;
                case KeyCode.DownArrow: _p.speed = Mathf.Max(_p.speed * 0.5f, 0.05f); e.Use(); break;
            }
        }
    }
}
