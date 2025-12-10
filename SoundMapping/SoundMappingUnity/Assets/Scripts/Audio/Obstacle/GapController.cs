using System.Linq;
using UnityEngine;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif


public class GapsController : MonoBehaviour
{
    [Header("Obstacle Geometry")]
    [Tooltip("Total width of the corridor in world units.")]
    [Range(100, 300)]
    public float corridorWidth = 200f;

    [Tooltip("Minimum gap size in world units before quantization.")]
    [Range(1f, 150f)]
    public float minGapSize = 7f;

    [Tooltip("Maximum gap size in world units before corridor cap and quantization.")]
    [Range(1f, 150f)]
    public float maxGapSize = 50f;

    [Tooltip("Width of the walls in world units.")]
    [Range(1, 10)]
    public float gapWidth = 5f;

    [Tooltip("Size step (units) used to quantize gaps and star grid.")]
    [Range(1, 25)]
    public float gapResolution = 5f;

    [Range(10, 100)]
    public float gapSpacing = 50f;

    private const string boundaryWallPrefabName = "ObstacleWall";
    private const string boundaryWallFolder     = "Assets/Prefab/DronePrefabUtilities";

    private Transform leftBoundaryWall;
    private Transform rightBoundaryWall;

    #if UNITY_EDITOR
    private GameObject LoadPrefab(string name, string folder)
    {
        string[] guids = AssetDatabase.FindAssets(name + " t:Prefab", new[] { folder });
        if (guids.Length == 0)
        {
            Debug.LogError("[GapsController] Boundary wall prefab not found: " + name);
            return null;
        }

        string path = AssetDatabase.GUIDToAssetPath(guids[0]);
        return AssetDatabase.LoadAssetAtPath<GameObject>(path);
    }
    #endif

    private void BuildOrUpdateBoundaryWalls(List<Gap> gaps)
    {
    #if UNITY_EDITOR
        if (gaps.Count == 0)
            return;

        // Load prefab
        GameObject prefab = LoadPrefab(boundaryWallPrefabName, boundaryWallFolder);
        if (prefab == null)
            return;

        // Reuse already-instantiated boundary walls if they exist in the hierarchy
        if (leftBoundaryWall == null)
            leftBoundaryWall = transform.Find("BoundaryLeft");
        if (rightBoundaryWall == null)
            rightBoundaryWall = transform.Find("BoundaryRight");

        // Read wall geometry from the first gap's left wall
        float wallHeight    = gaps[0].leftWall.localScale.y;
        float wallThickness = gaps[0].leftWall.localScale.z;

        // Compute Z positions
        float startZ = gaps[0].transform.localPosition.z;
        float totalLength = ((gaps.Count - 1) * gapSpacing) + wallThickness;
        float centerZ = startZ + ((gaps.Count - 1) * gapSpacing / 2);

        float halfCorridor = corridorWidth * 0.5f;

        // ---------- LEFT WALL ----------
        if (leftBoundaryWall == null)
        {
            Debug.Log("creating left wall");
            GameObject go = PrefabUtility.InstantiatePrefab(prefab, transform) as GameObject;
            go.name = "BoundaryLeft";
            leftBoundaryWall = go.transform;
        }

        leftBoundaryWall.localPosition = new Vector3(-halfCorridor, wallHeight/2, centerZ);
        leftBoundaryWall.localScale = new Vector3(
            wallThickness,
            wallHeight,
            totalLength
        );

        // ---------- RIGHT WALL ----------
        if (rightBoundaryWall == null)
        {
            Debug.Log("creating right wall");
            GameObject go = PrefabUtility.InstantiatePrefab(prefab, transform) as GameObject;
            go.name = "BoundaryRight";
            rightBoundaryWall = go.transform;
        }

        rightBoundaryWall.localPosition = new Vector3(+halfCorridor, wallHeight/2, centerZ);
        rightBoundaryWall.localScale = new Vector3(
            wallThickness,
            wallHeight,
            totalLength
        );
        UpdateGroundScale(totalLength);
    #endif
    }

    private void UpdateGroundScale(float totalLength)
    {
        TestCourse tc = GetComponentInParent<TestCourse>();
        if (tc == null)
            return;

        if (tc.groundTile == null)
            return;

        Vector3 s = tc.groundTile.localScale;
        s.x = corridorWidth + gapWidth;
        s.z = totalLength + 200;
        tc.groundTile.localScale = s;

        s = tc.startLine.localScale;
        s.x = corridorWidth + gapWidth;
        tc.startLine.localScale = s;
        tc.endLine.localScale = s;

        // Move endline
        Vector3 ep = tc.endLine.localPosition;
        ep.z = tc.gc_position.z + (tc.NB_GAPS-1)*gapSpacing + 50f;
        tc.endLine.localPosition = ep;

        // Move ground tile
        Vector3 gp = tc.groundTile.localPosition;
        gp.z = tc.gc_position.z + (tc.NB_GAPS-1)*gapSpacing/2;
        tc.groundTile.localPosition = gp;
    }

    #if UNITY_EDITOR
    private bool isApplying = false;

    void OnValidate()
    {
        // Delay execution to avoid SendMessage error in OnValidate/Awake
        if (!isApplying)
        {
            isApplying = true;
            UnityEditor.EditorApplication.delayCall += DelayedApply;
        }
    }

    private void DelayedApply()
    {
        // Important : retirer la fonction de la file d'attente
        UnityEditor.EditorApplication.delayCall -= DelayedApply;
        
        if (this == null) 
        {
            isApplying = false;
            return;
        }

        Apply();
        isApplying = false;
    }
    #else

    void OnValidate()
    {
        Apply();
    }
    #endif

    public void Apply()
    {
        // 1. Gather all Gap components
        var gaps = GetComponentsInChildren<Gap>(includeInactive: true).ToList();
        if (gaps.Count == 0)
            return;

        // 2. Sort by current Z position
        gaps = gaps.OrderBy(g => g.transform.localPosition.z).ToList();

        // 3. Assign global gap width and update layout
        float startZ = gaps[0].transform.localPosition.z;

        for (int i = 0; i < gaps.Count; i++)
        {
            Gap g = gaps[i];

            // Assign shared gap width
            Vector3 ls = g.leftWall.localScale;
            ls.z = gapWidth;
            g.leftWall.localScale = ls;
            Vector3 rs = g.rightWall.localScale;
            rs.z = gapWidth;
            g.rightWall.localScale = rs;

            // Reposition along Z
            Vector3 p = g.transform.localPosition;
            p.z = startZ + i * gapSpacing;
            g.transform.localPosition = p;

            float maxAllowedSize = Mathf.Min(maxGapSize, corridorWidth);
            float minAllowedSize = Mathf.Min(Mathf.Max(minGapSize, gapResolution), maxAllowedSize);

            int minSteps = Mathf.CeilToInt(minAllowedSize / gapResolution);
            int maxSteps = Mathf.Max(minSteps, Mathf.FloorToInt(maxAllowedSize / gapResolution));
            float minSizeAligned = minSteps * gapResolution;
            float maxSizeAligned = maxSteps * gapResolution;

            if (g.gapSize <= 0f)
            {
                int randomStep = Random.Range(minSteps, maxSteps + 1);
                g.gapSize = randomStep * gapResolution;
            }
            else if (g.gapSize < minSizeAligned)
            {
                g.gapSize = minSizeAligned;
            }
            else if (g.gapSize > maxSizeAligned)
            {
                g.gapSize = maxSizeAligned;
            }

            // Apply layout (this clamps gapCenterX too)
            g.Apply();
        }

        BuildOrUpdateBoundaryWalls(gaps);
    }
}
