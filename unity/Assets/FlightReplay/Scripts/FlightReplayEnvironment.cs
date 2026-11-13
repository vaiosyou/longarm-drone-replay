// FlightReplayEnvironment.cs  (optional)
// Real-world scenery under the replay, switchable in the Inspector (also while playing):
//   None   : nothing (your own scene)
//   Cesium : Cesium for Unity (e.g. Google Photorealistic 3D Tiles). The CesiumGeoreference origin is set to the
//            replay's geo origin (meta.geo_origin), so Unity (0,0,0) = replay origin. Cesium uses
//            x = East, y = Up, z = North, the same axes as the replay.
//   GSI    : terrain + aerial photo built by python/gsi_terrain.py (Geospatial Information Authority of Japan tiles),
//            loaded from StreamingAssets/<gsiFolder>/.
// Height alignment: the map datum and the log altitude rarely match exactly, so the replay can be
// snapped automatically (body at the first frame placed 'Ground Clearance' above the ground found by a ray)
// and/or moved by hand ('Manual Height Offset', added on top, editable while playing).
//
// Cesium for Unity is NOT required to compile this script: it is accessed by reflection
// (CesiumForUnity.CesiumGeoreference.SetOriginLongitudeLatitudeHeight(lon, lat, height), height above the ellipsoid).
using System;
using System.IO;
using System.Reflection;
using UnityEngine;

namespace FlightReplay
{
    [DefaultExecutionOrder(-50)]   // move ReplayOrigin before FlightReplayPlayer applies the pose
    [RequireComponent(typeof(FlightReplayPlayer))]
    public class FlightReplayEnvironment : MonoBehaviour
    {
        public enum Mode { None, Cesium, GSI }

        [Header("Scenery")]
        public Mode mode = Mode.None;
        [Tooltip("Objects hidden while Cesium or GSI scenery is shown (e.g. the Hakoniwa ENV object)")]
        public GameObject[] hideWhenSceneryOn = new GameObject[0];
        [Tooltip("Hide the FlightReplayGrid (ground grid and N/E labels) while Cesium or GSI scenery is shown")]
        public bool hideGridWhenSceneryOn = true;

        [Header("Geo origin (default: meta.geo_origin of the replay JSON)")]
        public bool overrideGeoOrigin = false;
        public double latitude, longitude, altitude;

        [Header("Cesium")]
        [Tooltip("GameObject with the CesiumGeoreference component (parent of the 3D tileset)")]
        public GameObject cesiumGeoreference;
        [Tooltip("Geoid height N [m]: Cesium origin height = altitude + N (ellipsoidal). Leave 0 and use the ground snap if unknown.")]
        public double geoidHeight = 0.0;

        [Header("GSI (python/gsi_terrain.py)")]
        [Tooltip("Folder under Assets/StreamingAssets containing gsi_terrain.json and gsi_photo.jpg")]
        public string gsiFolder = "gsi";
        public bool showAttribution = true;

        [Header("Height alignment")]
        public bool autoSnapToGround = true;
        [Tooltip("Height of the body centre above the ground at the first frame [m] (landing gear)")]
        public float groundClearance = 0.2f;
        [Tooltip("Manual vertical offset [m], added on top of the automatic snap. Editable while playing.")]
        public float manualHeightOffset = 0f;
        [Tooltip("Keep refining the snap for this many seconds (Cesium tiles keep streaming in)")]
        public float snapRefineSeconds = 20f;

        public float SnapOffset { get; private set; }
        public bool HasSnap { get; private set; }

        FlightReplayPlayer _p;
        Mode _applied = (Mode)(-1);
        GameObject _gsiRoot;
        string _gsiAttribution = "";
        float _snapUntil, _nextSnap;

        void Awake() { _p = GetComponent<FlightReplayPlayer>(); }

        void Start() { ApplyMode(); }

        void Update()
        {
            if (_applied != mode) ApplyMode();

            // keep ReplayOrigin on the scenery anchor (Unity origin = geo origin), plus the height alignment
            Transform anchor = SceneryRoot();
            if (anchor != null)
            {
                transform.SetPositionAndRotation(anchor.position + Vector3.up * ((HasSnap ? SnapOffset : 0f) + manualHeightOffset),
                                                 anchor.rotation);
                if (autoSnapToGround && Time.time < _snapUntil && Time.time >= _nextSnap)
                {
                    _nextSnap = Time.time + 1f;
                    TrySnap(false);
                }
            }
            else if (mode == Mode.None)
            {
                // own scene: only the manual offset is applied, relative to where ReplayOrigin was placed
                Vector3 p = transform.position;
                p.y = _baseY + manualHeightOffset;
                transform.position = p;
            }
        }

        float _baseY;
        bool _baseStored;

        Transform SceneryRoot()
        {
            if (mode == Mode.Cesium && cesiumGeoreference != null && cesiumGeoreference.activeInHierarchy) return cesiumGeoreference.transform;
            if (mode == Mode.GSI && _gsiRoot != null && _gsiRoot.activeInHierarchy) return _gsiRoot.transform;
            return null;
        }

        bool GetGeoOrigin(out double lat, out double lon, out double alt)
        {
            lat = latitude; lon = longitude; alt = altitude;
            if (overrideGeoOrigin) return true;
            var g = (_p != null && _p.IsLoaded) ? _p.Log.meta.geo_origin : null;
            if (g == null || g.Length != 3) return false;
            lat = g[0]; lon = g[1]; alt = g[2];
            latitude = lat; longitude = lon; altitude = alt;   // show in the Inspector
            return true;
        }

        public void ApplyMode()
        {
            if (!_baseStored) { _baseY = transform.position.y; _baseStored = true; }
            _applied = mode;
            HasSnap = false;
            _snapUntil = Time.time + snapRefineSeconds;
            _nextSnap = Time.time + 0.5f;

            foreach (var go in hideWhenSceneryOn) if (go != null) go.SetActive(mode == Mode.None);
            var grid = GetComponent<FlightReplayGrid>();
            if (grid != null) grid.SetVisible(mode == Mode.None || !hideGridWhenSceneryOn);
            if (cesiumGeoreference != null) cesiumGeoreference.SetActive(mode == Mode.Cesium);
            if (_gsiRoot != null) _gsiRoot.SetActive(mode == Mode.GSI);

            if (mode == Mode.Cesium)
            {
                if (cesiumGeoreference == null) { Debug.LogWarning("[FlightReplay] Environment: assign the CesiumGeoreference object."); return; }
                if (!GetGeoOrigin(out var lat, out var lon, out var alt)) { Debug.LogWarning("[FlightReplay] Environment: no geo origin (re-convert the log, or tick Override Geo Origin)."); return; }
                SetCesiumOrigin(lon, lat, alt + geoidHeight);
            }
            else if (mode == Mode.GSI && _gsiRoot == null)
            {
                _gsiRoot = BuildGsi();
            }
            else if (mode == Mode.None)
            {
                Vector3 p = transform.position; p.y = _baseY + manualHeightOffset; transform.position = p;
            }
        }

        void SetCesiumOrigin(double lon, double lat, double height)
        {
            Component geo = null;
            foreach (var c in cesiumGeoreference.GetComponents<Component>())
                if (c != null && c.GetType().Name == "CesiumGeoreference") { geo = c; break; }
            if (geo == null) { Debug.LogWarning("[FlightReplay] Environment: no CesiumGeoreference component on the assigned object (is Cesium for Unity installed?)."); return; }
            var t = geo.GetType();
            var placement = t.GetProperty("originPlacement");
            if (placement != null && placement.PropertyType.IsEnum)
            {
                try { placement.SetValue(geo, Enum.Parse(placement.PropertyType, "CartographicOrigin")); } catch (Exception) { }
            }
            var m = t.GetMethod("SetOriginLongitudeLatitudeHeight", new[] { typeof(double), typeof(double), typeof(double) });
            if (m == null) { Debug.LogWarning("[FlightReplay] Environment: SetOriginLongitudeLatitudeHeight not found (Cesium for Unity version?)."); return; }
            m.Invoke(geo, new object[] { lon, lat, height });
            Debug.Log($"[FlightReplay] Cesium origin set: lat {lat:F7}, lon {lon:F7}, height {height:F1} m");
        }

        // ---------------- GSI terrain ----------------
        [Serializable]
        class GsiTerrain
        {
            public int nx, nz;
            public float dx, dz, x0, z0;
            public float[] heights;
            public string photo;
            public string attribution;
        }

        GameObject BuildGsi()
        {
            string dir = Path.Combine(Application.streamingAssetsPath, gsiFolder);
            string jp = Path.Combine(dir, "gsi_terrain.json");
            if (!File.Exists(jp)) { Debug.LogWarning($"[FlightReplay] Environment: {jp} not found (run python/gsi_terrain.py)."); return null; }
            var t = JsonUtility.FromJson<GsiTerrain>(File.ReadAllText(jp));
            if (t == null || t.heights == null || t.heights.Length != t.nx * t.nz) { Debug.LogWarning("[FlightReplay] Environment: invalid gsi_terrain.json"); return null; }
            _gsiAttribution = t.attribution ?? "";

            var verts = new Vector3[t.nx * t.nz];
            var uvs = new Vector2[verts.Length];
            for (int j = 0; j < t.nz; j++)          // j: North index
                for (int i = 0; i < t.nx; i++)      // i: East index
                {
                    int k = j * t.nx + i;
                    verts[k] = new Vector3(t.x0 + i * t.dx, t.heights[k], t.z0 + j * t.dz);
                    uvs[k] = new Vector2(i / (float)(t.nx - 1), j / (float)(t.nz - 1));   // photo: north up
                }
            var tris = new int[(t.nx - 1) * (t.nz - 1) * 6];
            int n = 0;
            for (int j = 0; j < t.nz - 1; j++)
                for (int i = 0; i < t.nx - 1; i++)
                {
                    int a = j * t.nx + i, b = a + t.nx, c = b + 1, d = a + 1;   // clockwise seen from above
                    tris[n++] = a; tris[n++] = b; tris[n++] = c;
                    tris[n++] = a; tris[n++] = c; tris[n++] = d;
                }
            var mesh = new Mesh { name = "GsiTerrain" };
            if (verts.Length > 65000) mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
            mesh.vertices = verts; mesh.uv = uvs; mesh.triangles = tris;
            mesh.RecalculateNormals(); mesh.RecalculateBounds();

            var root = new GameObject("GsiTerrain");
            root.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            root.AddComponent<MeshFilter>().sharedMesh = mesh;
            root.AddComponent<MeshCollider>().sharedMesh = mesh;     // for the ground snap
            var mr = root.AddComponent<MeshRenderer>();
            Shader sh = Shader.Find("Universal Render Pipeline/Unlit");
            if (sh == null) sh = Shader.Find("Unlit/Texture");
            if (sh == null) sh = Shader.Find("Standard");
            var mat = new Material(sh);
            string pp = Path.Combine(dir, string.IsNullOrEmpty(t.photo) ? "gsi_photo.jpg" : t.photo);
            if (File.Exists(pp))
            {
                var tex = new Texture2D(2, 2, TextureFormat.RGB24, true) { wrapMode = TextureWrapMode.Clamp };
                tex.LoadImage(File.ReadAllBytes(pp));
                mat.mainTexture = tex;
            }
            mr.sharedMaterial = mat;
            Debug.Log($"[FlightReplay] GSI terrain loaded: {t.nx}x{t.nz}, {t.dx} m grid");
            return root;
        }

        // ---------------- height alignment ----------------
        [ContextMenu("Snap to ground now")]
        public void SnapNow() { TrySnap(true); }

        bool TrySnap(bool log)
        {
            Transform root = SceneryRoot();
            if (root == null || _p == null || !_p.IsLoaded) return false;
            var f0 = _p.Log.frames[0];
            Vector3 body = root.TransformPoint(new Vector3(f0.px, f0.py, f0.pz));       // first frame, before offsets
            Vector3 from = new Vector3(body.x, body.y + 3000f, body.z);
            var hits = Physics.RaycastAll(from, Vector3.down, 6000f, ~0, QueryTriggerInteraction.Ignore);
            float best = float.NegativeInfinity;
            foreach (var h in hits)
                if (h.collider != null && h.collider.transform.IsChildOf(root) && h.point.y > best) best = h.point.y;
            if (float.IsNegativeInfinity(best))
            {
                if (log) Debug.LogWarning("[FlightReplay] Ground snap: no ground found under the first frame (tiles not loaded / no colliders?).");
                return false;
            }
            SnapOffset = best + groundClearance - body.y;
            if (log || !HasSnap) Debug.Log($"[FlightReplay] Ground snap: offset {SnapOffset:F2} m (ground {best:F2} m)");
            HasSnap = true;
            return true;
        }

        void OnGUI()
        {
            if (!showAttribution || mode != Mode.GSI || string.IsNullOrEmpty(_gsiAttribution)) return;
            var st = new GUIStyle(GUI.skin.label) { fontSize = 11, alignment = TextAnchor.LowerRight };
            st.normal.textColor = Color.white;
            GUI.Label(new Rect(0, Screen.height - 120, Screen.width - 8, 20), _gsiAttribution, st);
        }
    }
}
