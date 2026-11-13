// FlightReplayMenu.cs  (Editor only)
// Menu: Tools > FlightReplay > ...
//   Create Replay Rig                    : ReplayOrigin + Player/Visualizer/UI + grid (+ follow camera on Main Camera)
//   Create Replay Rig (selected = vehicle): same, using the selected object as the vehicle
using UnityEditor;
using UnityEngine;

namespace FlightReplay.EditorTools
{
    public static class FlightReplayMenu
    {
        [MenuItem("Tools/FlightReplay/Create Replay Rig", priority = 0)]
        static void CreateRig() => Create(null);

        [MenuItem("Tools/FlightReplay/Create Replay Rig (selected object = vehicle)", priority = 1)]
        static void CreateRigWithSelection() => Create(Selection.activeTransform);

        [MenuItem("Tools/FlightReplay/Create Replay Rig (selected object = vehicle)", true)]
        static bool CreateRigWithSelectionValidate() => Selection.activeTransform != null;

        static void Create(Transform vehicle)
        {
            var origin = new GameObject("ReplayOrigin");
            Undo.RegisterCreatedObjectUndo(origin, "Create Replay Rig");

            var player = Undo.AddComponent<FlightReplayPlayer>(origin);
            Undo.AddComponent<FlightReplayVisualizer>(origin);
            Undo.AddComponent<FlightReplayUI>(origin);
            Undo.AddComponent<FlightReplayGrid>(origin);
            Undo.AddComponent<FlightReplayPropellers>(origin);
            Undo.AddComponent<FlightReplayTelemetry>(origin);
            Undo.AddComponent<FlightReplayEnvironment>(origin);   // Mode = None by default   // auto-assigns props named "Pivot" at Play; harmless without motor data

            if (vehicle != null)
            {
                Undo.SetTransformParent(vehicle, origin.transform, "Parent vehicle to ReplayOrigin");
                player.target = vehicle;
            }
            // else: Player creates a placeholder body at runtime (shape selectable in the Inspector)

            var cam = Camera.main;
            if (cam != null && cam.GetComponent<FlightReplayFollowCamera>() == null)
            {
                var fc = Undo.AddComponent<FlightReplayFollowCamera>(cam.gameObject);
                fc.player = player;
            }

            Selection.activeGameObject = origin;
            Debug.Log("[FlightReplay] Rig created. Put the replay JSON in Assets/StreamingAssets/ and set 'Json Path' on FlightReplayPlayer, then press Play."
                      + (vehicle != null ? $" Vehicle: {vehicle.name} (disable any script that also moves it)." : " No vehicle selected: a placeholder box will be used."));
        }
    }
}
