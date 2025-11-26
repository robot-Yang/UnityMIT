// Place this file under an `Editor/` folder, e.g., Assets/Editor/CorridorAlignTools.cs
// Unity 2020+ compatible. Provides quick tools to align, center, and chain "corridor" pieces.
// It DOES NOT change your corridor geometry; it only moves/rotates transforms.

using System.Linq;
using UnityEngine;
using UnityEditor;

namespace Haptics.Editor
{
    public static class CorridorAlignTools
    {
        // Change this if your corridor model's length axis is along local X instead of local Z.
        private static readonly Vector3 kModelLengthAxisLocal = Vector3.forward; // or Vector3.right

        // Extra gap between corridors (in world units) when chaining
        private const float kGap = 0f;

        // ----------------------------------------------
        // 1) Align yaw of selected objects so "length axis" faces +X (world)
        // ----------------------------------------------
        [MenuItem("Tools/Corridor/Align Yaw to +X (World)")]
        public static void AlignYawToX()
        {
            var selection = Selection.transforms;
            if (selection == null || selection.Length == 0)
            {
                Debug.LogWarning("[CorridorAlign] Select some corridor root objects first.");
                return;
            }

            Undo.RecordObjects(selection, "Align Yaw to +X");

            foreach (var t in selection)
            {
                Vector3 lengthDirWorld = t.TransformDirection(kModelLengthAxisLocal);
                Quaternion delta = Quaternion.FromToRotation(lengthDirWorld, Vector3.right);

                t.rotation = delta * t.rotation;

                var e = t.eulerAngles;
                t.eulerAngles = new Vector3(0f, e.y, 0f);
            }
        }

        // ----------------------------------------------
        // 2A) Snap center of selected objects onto the world X-axis (set world Z of bounds center to 0)
        // ----------------------------------------------
        [MenuItem("Tools/Corridor/Center To X-Axis (Z=0)")]
        public static void CenterToXAxis()
        {
            var selection = Selection.transforms;
            if (selection == null || selection.Length == 0)
            {
                Debug.LogWarning("[CorridorAlign] Select some corridor root objects first.");
                return;
            }

            Undo.RecordObjects(selection, "Center To X-Axis");

            foreach (var t in selection)
            {
                if (!TryGetWorldBounds(t, out var b))
                {
                    Debug.LogWarning($"[CorridorAlign] No renderers under '{t.name}'. Skipped.");
                    continue;
                }

                // We want the bounds center's world Z to become 0 → move along Z
                float dz = -b.center.z;
                t.position += new Vector3(0f, 0f, dz);
            }
        }

        // ----------------------------------------------
        // 2B) Snap center of selected objects onto the world Z-axis (set world X of bounds center to 0)
        // ----------------------------------------------
        [MenuItem("Tools/Corridor/Center To Z-Axis (X=0)")]
        public static void CenterToZAxis()
        {
            var selection = Selection.transforms;
            if (selection == null || selection.Length == 0)
            {
                Debug.LogWarning("[CorridorAlign] Select some corridor root objects first.");
                return;
            }

            Undo.RecordObjects(selection, "Center To Z-Axis");

            foreach (var t in selection)
            {
                if (!TryGetWorldBounds(t, out var b))
                {
                    Debug.LogWarning($"[CorridorAlign] No renderers under '{t.name}'. Skipped.");
                    continue;
                }

                // We want the bounds center's world X to become 0 → move along X
                float dx = -b.center.x;
                t.position += new Vector3(dx, 0f, 0f);
            }
        }

        // ----------------------------------------------
        // 3) Arrange selected corridors as a chain along +X, end-to-end
        // ----------------------------------------------
        [MenuItem("Tools/Corridor/Chain Along +X (Auto-Length)")]
        public static void ChainAlongX()
        {
            var selection = Selection.transforms;
            if (selection == null || selection.Length == 0)
            {
                Debug.LogWarning("[CorridorAlign] Select some corridor root objects first.");
                return;
            }

            var ordered = selection.OrderBy(t => t.position.x).ToArray();

            // Optional pre-step: align + center on X-axis
            AlignYawToX();
            CenterToXAxis();

            Undo.RecordObjects(ordered, "Chain Along +X");

            float cursorX = 0f;

            for (int i = 0; i < ordered.Length; i++)
            {
                var t = ordered[i];
                if (!TryGetWorldBounds(t, out var b)) continue;

                Vector3 lengthDirWorld = t.TransformDirection(kModelLengthAxisLocal).normalized;
                if (Vector3.Dot(lengthDirWorld, Vector3.right) < 0f)
                {
                    t.Rotate(0f, 180f, 0f, Space.World);
                    lengthDirWorld = t.TransformDirection(kModelLengthAxisLocal).normalized;
                }

                float len = ProjectSizeOnDirection(b.size, Vector3.right);

                if (i == 0)
                {
                    float minX = b.center.x - len * 0.5f;
                    float dx = (cursorX - minX);
                    t.position += new Vector3(dx, 0f, 0f);
                }
                else
                {
                    var prev = ordered[i - 1];
                    if (!TryGetWorldBounds(prev, out var prevB)) continue;
                    float prevLen = ProjectSizeOnDirection(prevB.size, Vector3.right);
                    float prevMaxX = prevB.center.x + prevLen * 0.5f;
                    float targetMinX = prevMaxX + kGap;

                    float minX = b.center.x - len * 0.5f;
                    float dx = (targetMinX - minX);
                    t.position += new Vector3(dx, 0f, 0f);
                }
            }

            Debug.Log($"[CorridorAlign] Chained {ordered.Length} piece(s) along +X.");
        }

        // ----------------------------------------------
        // Helpers
        // ----------------------------------------------
        private static bool TryGetWorldBounds(Transform root, out Bounds total)
        {
            var renderers = root.GetComponentsInChildren<Renderer>();
            if (renderers == null || renderers.Length == 0)
            {
                total = default;
                return false;
            }

            total = renderers[0].bounds;
            for (int i = 1; i < renderers.Length; i++)
            {
                total.Encapsulate(renderers[i].bounds);
            }
            return true;
        }

        private static Bounds GetWorldBounds(Transform root)
        {
            TryGetWorldBounds(root, out var b);
            return b;
        }

        private static float ProjectSizeOnDirection(Vector3 size, Vector3 dirWorldNormalized)
        {
            dirWorldNormalized.Normalize();
            var abs = new Vector3(Mathf.Abs(dirWorldNormalized.x), Mathf.Abs(dirWorldNormalized.y), Mathf.Abs(dirWorldNormalized.z));
            return Vector3.Dot(size, abs);
        }
    }
}
