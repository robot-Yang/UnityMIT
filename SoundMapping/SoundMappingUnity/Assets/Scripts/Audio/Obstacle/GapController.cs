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

    [Tooltip("Width of the walls in world units.")]
    [Range(1, 10)]
    public float gapWidth = 5f;

    [Range(5, 50)]
    public float gapSize = 25f;

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
        s.x = corridorWidth + gapWidth;   // width of the ground = corridor width
        s.z = totalLength + 200;     // length matches course length
        tc.groundTile.localScale = s;

        s = tc.startLine.localScale;
        s.x = corridorWidth;
        tc.startLine.localScale = s;
        tc.endLine.localScale = s;
    }


    // GapsController.cs (Nouveau)

    #if UNITY_EDITOR
    private bool isApplying = false;

    void OnValidate()
    {
        // Décaler l'exécution pour éviter l'erreur SendMessage en OnValidate/Awake
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
        
        // Assurez-vous que l'objet existe toujours avant d'appliquer
        if (this == null) 
        {
            isApplying = false;
            return;
        }

        Apply();
        isApplying = false;
    }
    #else
    // Comportement normal si nous ne sommes pas dans l'éditeur
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

            g.gapWidth = gapSize;

            // Apply layout (this clamps gapCenterX too)
            g.Apply();
        }

        BuildOrUpdateBoundaryWalls(gaps);
    }
}
