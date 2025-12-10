using UnityEngine;
using UnityEditor;

// ============================================================================
// MAIN RUNTIME SCRIPT
// ============================================================================
public class TestCourse : MonoBehaviour
{
    [Header("Lookup names")]
    public string pathHolderName = "Path Holder";
    public string floorName = "floor";
    public string startName = "Path (4)";
    public string endName = "Path (1)";
    public string groundName = "Path";


    [Header("Ground Scale")]
    public Vector3 groundScale = new Vector3(100f, 1f, 500f);

    [Header("Start Transform")]
    public Vector3 startPosition = new Vector3(0, 0.5f, 15);
    public Vector3 startRotation = new Vector3(0f, 0f, 0f);
    public Vector3 startScale    = new Vector3(100f, 3f, 10f);

    [Header("End Transform")]
    public Vector3 endPosition = new Vector3(0, 0.5f, 130);
    public Vector3 endRotation = new Vector3(0f, 0f, 0f);
    public Vector3 endScale    = new Vector3(100f, 3f, 10f);

    [Header("Gap Controller")]
    public Vector3 gc_position = new Vector3(177f, 2.25f, 72f);

    [Header("Gap Generation")]
    public int NB_GAPS = 7;

    [Header("Gap Positioning")]
    public float firstGapZOffset = 0f;

    public float[] initialGapCenters = new float[]
    {
        -10f, 30f, 0f, 20f, -30f, 10f, -20f
    };

    [Header("Wall Layout")]
    public float wallHeight = 50f;
    public float wallThickness = 5f;
    public float wallY = 25f;

    [Header("Prefab Auto-Load")]
    public string wallPrefabName = "ObstacleWall";
    public string wallPrefabFolder = "Assets/Prefab/DronePrefabUtilities";

    private GameObject wallPrefab;

    [HideInInspector]
    public Transform startLine;
    [HideInInspector]
    public Transform endLine;
    [HideInInspector]
    public Transform groundTile;

    // -------------------------------
    // AUTO-FIND
    // -------------------------------
    public void AutoFindReferences()
    {
        Transform parent = transform.parent;

        Transform pathHolder =
            (parent != null ? parent.Find(pathHolderName) : null);

        if (pathHolder == null)
        {
            Debug.LogError("[TestCourse] Path Holder not found.");
            return;
        }

        Transform floor = pathHolder.Find(floorName);
        if (floor == null)
        {
            Debug.LogError("[TestCourse] floor not found.");
            return;
        }

        startLine = floor.Find(startName);
        endLine   = floor.Find(endName);
        groundTile = floor.Find(groundName);

        if (startLine  == null) Debug.LogError("[TestCourse] Start not found");
        if (endLine    == null) Debug.LogError("[TestCourse] End not found");
        if (groundTile == null) Debug.LogError("[TestCourse] Ground not found");
    }

#if UNITY_EDITOR
    // -------------------------------
    // LOAD PREFAB FROM PROJECT FOLDER
    // -------------------------------
    private GameObject LoadPrefab(string name, string folder)
    {
        string[] guids = AssetDatabase.FindAssets(name + " t:Prefab", new[] { folder });

        if (guids.Length == 0)
        {
            Debug.LogError("[TestCourse] Prefab not found in folder: " + folder + " Name = " + name);
            return null;
        }

        string path = AssetDatabase.GUIDToAssetPath(guids[0]);
        return AssetDatabase.LoadAssetAtPath<GameObject>(path);
    }
#endif

    public void Generate()
    {
        Clean();
        PlaceStartEnd();
        GenerateGaps();
    }

    public void Clean()
    {
        // 1. Delete ALL children of TestCourse
        #if UNITY_EDITOR
        for (int i = transform.childCount - 1; i >= 0; i--)
            DestroyImmediate(transform.GetChild(i).gameObject);
        #else
        for (int i = transform.childCount - 1; i >= 0; i--)
            Destroy(transform.GetChild(i).gameObject);
        #endif

        // 2. Delete all FLOOR children except start, end, ground
        AutoFindReferences();

        if (startLine == null || endLine == null || groundTile == null)
            return;

        Transform floor = startLine.parent;

        #if UNITY_EDITOR
        for (int i = floor.childCount - 1; i >= 0; i--)
        {
            Transform c = floor.GetChild(i);
            if (c != startLine && c != endLine && c != groundTile)
                DestroyImmediate(c.gameObject);
        }
        #else
        for (int i = floor.childCount - 1; i >= 0; i--)
        {
            Transform c = floor.GetChild(i);
            if (c != startLine && c != endLine && c != groundTile)
                Destroy(c.gameObject);
        }
        #endif
    }

    // -------------------------------
    // BUTTON FUNCTION 1 — Place Start / End
    // -------------------------------
    public void PlaceStartEnd()
    {
        AutoFindReferences();
        if (startLine == null || endLine == null) return;

        startLine.position = startPosition;
        startLine.rotation = Quaternion.Euler(startRotation);
        startLine.localScale = startScale;

        endLine.position = endPosition;
        endLine.rotation = Quaternion.Euler(endRotation);
        endLine.localScale = endScale;

        // Basic scale for Y only
        groundTile.localScale = groundScale;

        // Align ground tile X position with the GapController
        Vector3 gp = groundTile.localPosition;
        gp.x = gc_position.x;
        gp.z = gc_position.z + (NB_GAPS-1) * 50f / 2f;
        groundTile.localPosition = gp;

        Debug.Log("[TestCourse] Start & End fully positioned (pos+rot+scale).");
    }

    public void GenerateGaps()
    {
        AutoFindReferences();

#if UNITY_EDITOR
        wallPrefab = LoadPrefab(wallPrefabName, wallPrefabFolder);
        if (wallPrefab == null)
        {
            Debug.LogError("[TestCourse] Cannot generate gaps because wall prefab failed to load.");
            return;
        }
#endif

        GameObject gc = new GameObject("GapController");
        gc.transform.SetParent(this.transform, worldPositionStays: false);

        // Clean identity transform
        gc.transform.localPosition = gc_position;
        gc.transform.localRotation = Quaternion.identity;
        gc.transform.localScale = Vector3.one;

        var controller = gc.AddComponent<GapsController>();

        // Create gaps RELATIVE TO gc_position
        for (int i = 0; i < NB_GAPS; i++)
            CreateGap(gc.transform, i);

        controller.Apply();
        Debug.Log("[TestCourse] Gaps generated and layout applied.");
    }

    private void CreateGap(Transform parent, int index)
    {
        GameObject gapGO = new GameObject("Gap (" + index + ")");
        gapGO.transform.parent = parent;

        Gap gap = gapGO.AddComponent<Gap>();

        // Assign initial center X if list has enough values
        if (initialGapCenters != null && index < initialGapCenters.Length)
        {
            gap.gapCenterX = initialGapCenters[index];
        }

        // Pre controller gap positioning
        float localZ = firstGapZOffset + index * 0.1f;

        // Place gap relative to the controller
        gapGO.transform.localPosition = new Vector3(
            0f,
            0f,
            localZ
        );
        gapGO.transform.localScale = Vector3.one;

        // Create side walls
        Transform L = Instantiate(wallPrefab, gapGO.transform).transform;
        L.name = "LeftWall";
        gap.leftWall = L;

        Transform R = Instantiate(wallPrefab, gapGO.transform).transform;
        R.name = "RightWall";
        gap.rightWall = R;

        // Side walls positioning
        Vector3 ls = L.localScale;
        ls.y = wallHeight;
        ls.x = wallThickness;
        L.localScale = ls;

        Vector3 rs = R.localScale;
        rs.y = wallHeight;
        rs.x = wallThickness;
        R.localScale = rs;

        Vector3 lp = L.localPosition;
        lp.y = wallY;
        L.localPosition = lp;

        Vector3 rp = R.localPosition;
        rp.y = wallY;
        R.localPosition = rp;

        gap.Initialize();
    }

}

// ============================================================================
// Inspector Buttons
// ============================================================================
#if UNITY_EDITOR

[CustomEditor(typeof(TestCourse))]
public class TestCourseEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        TestCourse script = (TestCourse)target;

        GUILayout.Space(10);
        EditorGUILayout.LabelField("TestCourse Tools", EditorStyles.boldLabel);

        if (GUILayout.Button("Generate"))
            script.Generate();
    }
}
#endif
