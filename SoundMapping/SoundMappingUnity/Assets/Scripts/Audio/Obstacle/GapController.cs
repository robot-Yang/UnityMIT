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

    public bool generateBoundaryWalls = false;

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
            // Debug.LogError("[GapsController] Boundary wall prefab not found: " + name);
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

        float wallThickness = gaps[0].leftWall.localScale.z;
        float totalLength = ((gaps.Count - 1) * gapSpacing) + wallThickness;

        if (!generateBoundaryWalls)
        {
            ClearBoundaryWalls();
            UpdateGroundScale(totalLength);
            return;
        }

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
        // Compute Z positions
        float startZ = gaps[0].transform.localPosition.z;
        float centerZ = startZ + ((gaps.Count - 1) * gapSpacing / 2);

        float halfCorridor = corridorWidth * 0.5f;

        // ---------- LEFT WALL ----------
        if (leftBoundaryWall == null)
        {
            // Debug.Log("creating left wall");
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
            // Debug.Log("creating right wall");
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

    private void ClearBoundaryWalls()
    {
    #if UNITY_EDITOR
        if (leftBoundaryWall == null)
            leftBoundaryWall = transform.Find("BoundaryLeft");
        if (rightBoundaryWall == null)
            rightBoundaryWall = transform.Find("BoundaryRight");

        if (leftBoundaryWall != null)
        {
            DestroyImmediate(leftBoundaryWall.gameObject);
            leftBoundaryWall = null;
        }

        if (rightBoundaryWall != null)
        {
            DestroyImmediate(rightBoundaryWall.gameObject);
            rightBoundaryWall = null;
        }
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

        Transform trajectoryStartLine = tc.TrajectoryStartLine();
        Transform trajectoryEndLine = tc.TrajectoryEndLine();
        if (trajectoryStartLine == null || trajectoryEndLine == null)
            return;

        s = trajectoryStartLine.localScale;
        s.x = corridorWidth + gapWidth;
        trajectoryStartLine.localScale = s;
        trajectoryEndLine.localScale = s;

        float firstGapZ = tc.FirstGapWorldZ();
        float lastGapZ = tc.LastGapWorldZ(gapSpacing);

        // Move start and end lines close to the measured trajectory segment.
        Vector3 sp = trajectoryStartLine.localPosition;
        sp.x = tc.gc_position.x;
        sp.z = firstGapZ - tc.startLineGapOffset;
        trajectoryStartLine.localPosition = sp;

        Vector3 ep = trajectoryEndLine.localPosition;
        ep.x = tc.gc_position.x;
        ep.z = lastGapZ + tc.finishLineGapOffset;
        trajectoryEndLine.localPosition = ep;

        // Move ground tile
        Vector3 gp = tc.groundTile.localPosition;
        gp.z = (firstGapZ + lastGapZ) * 0.5f;
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
        TestCourse testCourse = GetComponentInParent<TestCourse>();

        for (int i = 0; i < gaps.Count; i++)
        {
            Gap g = gaps[i];

            // Assign shared wall depth
            Vector3 ls = g.leftWall.localScale;
            ls.z = gapWidth;
            g.leftWall.localScale = ls;
            Vector3 rs = g.rightWall.localScale;
            rs.z = gapWidth;
            g.rightWall.localScale = rs;
            if (g.topWall != null)
            {
                Vector3 ts = g.topWall.localScale;
                ts.z = gapWidth;
                g.topWall.localScale = ts;
            }
            if (g.bottomWall != null)
            {
                Vector3 bs = g.bottomWall.localScale;
                bs.z = gapWidth;
                g.bottomWall.localScale = bs;
            }

            // Reposition along Z
            Vector3 p = g.transform.localPosition;
            p.z = startZ + i * gapSpacing;
            g.transform.localPosition = p;

            if (g.useSquareHole && testCourse != null)
            {
                g.squareWallSize = testCourse.wallHeight;
                if (testCourse.squareGapSize > 0f)
                    g.gapSize = testCourse.squareGapSize;
                g.starEulerRotation = testCourse.starEulerRotation;
            }

            float squareWallCap = g.useSquareHole ? Mathf.Max(g.squareWallSize, gapResolution) : corridorWidth;
            if (!g.useSquareHole || testCourse == null || testCourse.squareGapSize <= 0f)
            {
                float maxAllowedSize = Mathf.Min(maxGapSize, corridorWidth, squareWallCap);
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
            }
            else
            {
                g.gapSize = Mathf.Clamp(g.gapSize, gapResolution, Mathf.Min(corridorWidth, squareWallCap));
            }

            if (g.useSquareHole)
                g.gapHeight = g.gapSize;

            // Apply layout (this clamps gapCenterX too)
            g.Apply();
        }

        BuildOrUpdateBoundaryWalls(gaps);
    }
}
