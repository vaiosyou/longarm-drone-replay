// FlightReplayGrid.cs  (optional)
// Ground grid in the replay origin's local frame, so you can read direction and distance in an empty scene.
// Red line = East (+X), blue line = North (+Z), labels N/E/S/W drawn on screen.
// One mesh with line topology; no extra packages needed.
using UnityEngine;

namespace FlightReplay
{
    public class FlightReplayGrid : MonoBehaviour
    {
        [Tooltip("Half size of the grid [m]")]
        public float halfSize = 20f;
        [Tooltip("Line spacing [m]")]
        public float spacing = 1f;
        public Color lineColor = new Color(0.5f, 0.5f, 0.5f, 0.35f);
        public Color eastColor = new Color(0.9f, 0.2f, 0.2f, 0.9f);
        public Color northColor = new Color(0.2f, 0.4f, 1f, 0.9f);
        public bool showLabels = true;
        [Tooltip("Height of the grid relative to this object [m]")]
        public float height = 0f;

        Transform _gridT;

        void Start() { Build(); }

        public void Build()
        {
            if (_gridT != null) Destroy(_gridT.gameObject);
            var go = new GameObject("ReplayGrid");
            _gridT = go.transform;
            _gridT.SetParent(transform, false);
            _gridT.localPosition = new Vector3(0f, height, 0f);

            int n = Mathf.Max(1, Mathf.RoundToInt(halfSize / Mathf.Max(spacing, 0.01f)));
            int lines = (2 * n + 1) * 2;
            var v = new Vector3[lines * 2];
            var c = new Color[lines * 2];
            var idx = new int[lines * 2];
            int k = 0;
            for (int i = -n; i <= n; i++)
            {
                float x = i * spacing;
                Color colZ = i == 0 ? northColor : lineColor;   // line along Z at x = 0 is the North axis
                Color colX = i == 0 ? eastColor : lineColor;    // line along X at z = 0 is the East axis
                v[k] = new Vector3(x, 0, -n * spacing); c[k] = colZ; k++;
                v[k] = new Vector3(x, 0, n * spacing); c[k] = colZ; k++;
                v[k] = new Vector3(-n * spacing, 0, x); c[k] = colX; k++;
                v[k] = new Vector3(n * spacing, 0, x); c[k] = colX; k++;
            }
            for (int i = 0; i < idx.Length; i++) idx[i] = i;

            var mesh = new Mesh { name = "ReplayGridMesh" };
            if (v.Length > 65000) mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
            mesh.vertices = v;
            mesh.colors = c;
            mesh.SetIndices(idx, MeshTopology.Lines, 0);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            var mr = go.AddComponent<MeshRenderer>();
            mr.sharedMaterial = new Material(Shader.Find("Sprites/Default"));   // uses vertex colours
        }

        void OnGUI()
        {
            if (!showLabels || _gridT == null) return;
            var cam = Camera.main;
            if (cam == null) return;
            float r = halfSize;
            Label(cam, _gridT.TransformPoint(new Vector3(0, 0, r)), "N", northColor);
            Label(cam, _gridT.TransformPoint(new Vector3(r, 0, 0)), "E", eastColor);
            Label(cam, _gridT.TransformPoint(new Vector3(0, 0, -r)), "S", lineColor);
            Label(cam, _gridT.TransformPoint(new Vector3(-r, 0, 0)), "W", lineColor);
        }

        static void Label(Camera cam, Vector3 world, string text, Color col)
        {
            Vector3 s = cam.WorldToScreenPoint(world);
            if (s.z <= 0) return;
            var style = new GUIStyle(GUI.skin.label) { fontSize = 18, fontStyle = FontStyle.Bold };
            style.normal.textColor = new Color(col.r, col.g, col.b, 1f);
            GUI.Label(new Rect(s.x - 8, Screen.height - s.y - 12, 30, 24), text, style);
        }
    }
}
