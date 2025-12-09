using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class Gap : MonoBehaviour
{
    [Header("Gap")]
    [Tooltip("X position of the center of the gap, in parent's local space.")]
    [Range(-30, 30)]
    public float gapCenterX;
    
    [HideInInspector] 
    public float gapWidth;

    [Header("Walls (children of this object)")]
    public Transform leftWall;
    public Transform rightWall;

    // -------------------------------
    // Collectible settings (internal)
    // -------------------------------
    private Transform leftCollectible;
    private Transform rightCollectible;
    private const string collectiblePrefabName = "Star";
    private const string collectibleFolder     = "Assets/Prefab/";
    private float collectibleOffsetX = 5f;      // horizontal offset from gap center
    private float collectibleY = 25f;            // height at which stars appear


    // -------------------------------
    // Called manually from TestCourse
    // -------------------------------
    public void Initialize()
    {
    #if UNITY_EDITOR
        GameObject prefab = LoadPrefab(collectiblePrefabName, collectibleFolder);
        if (prefab == null)
            return;

        // Spawn left star
        GameObject left = PrefabUtility.InstantiatePrefab(prefab, this.transform) as GameObject;
        leftCollectible = left.transform;

        // Spawn right star
        GameObject right = PrefabUtility.InstantiatePrefab(prefab, this.transform) as GameObject;
        rightCollectible = right.transform;

        UpdateStars();  // Position initial stars
    #endif
    }

    private void UpdateStars()
    {
        if (leftCollectible == null || rightCollectible == null)
            return;

        leftCollectible.localPosition = new Vector3(
            gapCenterX - collectibleOffsetX,
            collectibleY,
            0f
        );

        rightCollectible.localPosition = new Vector3(
            gapCenterX + collectibleOffsetX,
            collectibleY,
            0f
        );
    }

#if UNITY_EDITOR
    // Loads prefab by name in a folder
    private GameObject LoadPrefab(string name, string folder)
    {
        string[] guids = AssetDatabase.FindAssets(name + " t:Prefab", new[] { folder });
        if (guids.Length == 0)
        {
            Debug.LogError("[Gap] Collectible prefab not found: " + name);
            return null;
        }

        string path = AssetDatabase.GUIDToAssetPath(guids[0]);
        return AssetDatabase.LoadAssetAtPath<GameObject>(path);
    }
#endif


    // -------------------------------
    // EXISTING APPLY (unchanged)
    // -------------------------------
    public void Apply()
    {
        if (leftWall == null || rightWall == null)
            return;

        // Fetch corridorWidth from parent GapsController
        GapsController controller = GetComponentInParent<GapsController>();
        if (controller == null)
            return;

        float corridorWidth = controller.corridorWidth;

        if (corridorWidth <= 0f)
            return;

        float halfCorridor = corridorWidth * 0.5f;

        // Clamp gap size to corridor
        gapWidth = Mathf.Clamp(gapWidth, 0f, corridorWidth);

        // Clamp center so gap always stays inside corridor
        float maxCenter = halfCorridor - gapWidth * 0.5f;
        gapCenterX = Mathf.Clamp(gapCenterX, -maxCenter, maxCenter);

        float halfGap = gapWidth * 0.5f;

        float leftEdge  = -halfCorridor;   
        float rightEdge =  halfCorridor;   

        float gapLeft  = gapCenterX - halfGap;
        float gapRight = gapCenterX + halfGap;

        // ---------- LEFT WALL ----------
        float leftWidth = Mathf.Max(0f, gapLeft - leftEdge);
        Vector3 ls = leftWall.localScale;
        ls.x = leftWidth;
        leftWall.localScale = ls;

        Vector3 lp = leftWall.localPosition;
        lp.x = leftEdge + leftWidth * 0.5f;
        leftWall.localPosition = lp;

        // ---------- RIGHT WALL ----------
        float rightWidth = Mathf.Max(0f, rightEdge - gapRight);
        Vector3 rs = rightWall.localScale;
        rs.x = rightWidth;
        rightWall.localScale = rs;

        Vector3 rp = rightWall.localPosition;
        rp.x = rightEdge - rightWidth * 0.5f;
        rightWall.localPosition = rp;

        UpdateStars();
    }

    private void OnValidate()
    {
        Apply();
    }
}
