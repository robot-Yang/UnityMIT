using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Minimal saver that mirrors TestCourse.Save but assumes every direct child of
/// this GameObject is a wall to export. Attach it to the root, ensure children
/// are the walls you want, and press "Save Course" to write JSON compatible
/// with the plotting script.
/// </summary>
public class ObstacleSaver : MonoBehaviour
{
    [Header("Export Target")]
    [Tooltip("Relative to Assets/ (e.g. Data/default/ObstacleCourse)")]
    public string exportFolder = "Data/default/ObstacleCourse";
    public string exportFileName = "TestCourse.json";

    [System.Serializable]
    public class ObstacleWall
    {
        public string name;
        public float x;
        public float rotationY;
        public float z;
        public float width;
        public float length;
    }

    [System.Serializable]
    public class GapExport
    {
        public int index;
        public ObstacleWall left;
        public ObstacleWall right;
    }

    [System.Serializable]
    public class ObstacleCourseExport
    {
        public float courseWidth;
        public float courseLength;
        public ObstacleWall[] boundaryWalls;
        public GapExport[] gaps;
    }

    public void Save()
    {
        var walls = CollectDirectChildWalls();
        if (walls.Count == 0)
        {
            Debug.LogWarning("[ObstacleSaver] Cannot save: no child walls found under this object.");
            return;
        }

        var bounds = ComputeBounds(walls);

        var export = new ObstacleCourseExport
        {
            courseWidth = bounds.width,
            courseLength = bounds.length,
            boundaryWalls = walls.ToArray(),
            gaps = new GapExport[0]
        };

        string dir = Path.Combine(Application.dataPath, exportFolder);
        if (!Directory.Exists(dir))
            Directory.CreateDirectory(dir);

        string finalFileName = string.IsNullOrEmpty(exportFileName) ? "ObstacleCourse.json" : exportFileName;
        string path = Path.Combine(dir, finalFileName);
        string json = JsonUtility.ToJson(export, true);
        File.WriteAllText(path, json);
        Debug.Log("[ObstacleSaver] Saved obstacle course to " + path);
    }

    private List<ObstacleWall> CollectDirectChildWalls()
    {
        var result = new List<ObstacleWall>();
        for (int i = 0; i < transform.childCount; i++)
        {
            Transform child = transform.GetChild(i);
            result.Add(ToWall(child, child.name));
        }
        return result;
    }

    private (float width, float length) ComputeBounds(List<ObstacleWall> walls)
    {
        if (walls.Count == 0)
            return (0f, 0f);

        float minX = float.PositiveInfinity;
        float maxX = float.NegativeInfinity;
        float minZ = float.PositiveInfinity;
        float maxZ = float.NegativeInfinity;

        foreach (var w in walls)
        {
            float halfW = w.width * 0.5f;
            float halfL = w.length * 0.5f;
            minX = Mathf.Min(minX, w.x - halfW);
            maxX = Mathf.Max(maxX, w.x + halfW);
            minZ = Mathf.Min(minZ, w.z - halfL);
            maxZ = Mathf.Max(maxZ, w.z + halfL);
        }

        return (Mathf.Max(0f, maxX - minX), Mathf.Max(0f, maxZ - minZ));
    }

    private ObstacleWall ToWall(Transform t, string defaultName)
    {
        Vector3 p = t.position;
        Vector3 s = t.lossyScale;
        Vector3 r = t.rotation.eulerAngles;
        return new ObstacleWall
        {
            name = string.IsNullOrEmpty(t.name) ? defaultName : t.name,
            x = p.x,
            rotationY = r.y,
            z = p.z,
            width = s.x,
            length = s.z
        };
    }
}

#if UNITY_EDITOR
[CustomEditor(typeof(ObstacleSaver))]
public class ObstacleSaverEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        ObstacleSaver script = (ObstacleSaver)target;

        GUILayout.Space(10);
        EditorGUILayout.LabelField("Obstacle Saver", EditorStyles.boldLabel);

        if (GUILayout.Button("Save Course"))
            script.Save();
    }
}
#endif
