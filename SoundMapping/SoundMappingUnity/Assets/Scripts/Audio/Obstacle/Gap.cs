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
    [System.Serializable]
    public struct StarConfig
    {
        public float offsetX;
        public float offsetY;
        public string starName;
        [HideInInspector] public Transform instance;
    }

    [Header("Stars")]
    public List<StarConfig> stars = new List<StarConfig>(); 
    public float gapCenterY = 18.3f;
    private const string collectiblePrefabName = "Star";
    private const string collectibleFolder     = "Assets/Prefab/";


    // -------------------------------
    // Called manually from TestCourse
    // -------------------------------
    public void Initialize()
    {
    #if UNITY_EDITOR
        GameObject prefab = LoadPrefab(collectiblePrefabName, collectibleFolder);
        if (prefab == null) return;

        char gapName = this.gameObject.name[5];

        // Instantiate all stars
        for (int i = 0; i < stars.Count; i++)
        {
            GameObject obj = PrefabUtility.InstantiatePrefab(prefab, this.transform) as GameObject;
            var sc = stars[i];
            string finalName = "Star_" + gapName + "_" + sc.starName; 
            obj.name = finalName;
            sc.instance = obj.transform;
            stars[i] = sc;
        }

        UpdateStars();
    #endif
    }

    private void ResetStarsIfEmpty()
    {
        if (stars == null || stars.Count == 0)
        {
            float size = 10f;
            float corner = 8.5f;
            stars = new List<StarConfig>
            {
                // Center
                new StarConfig { offsetX = 0f, offsetY = 0f, starName = "Center" },
                // X
                new StarConfig { offsetX =  size, offsetY = 0f, starName = "Right" },
                new StarConfig { offsetX =  -size, offsetY = 0f, starName = "Left" },
                // Y
                new StarConfig { offsetX =  0f, offsetY = size+2, starName = "Up" },
                new StarConfig { offsetX =  0f, offsetY = -size-2, starName = "Down"},
                // Corners
                new StarConfig { offsetX =  corner, offsetY = corner, starName = "UpRight"},
                new StarConfig { offsetX =  corner, offsetY = -corner, starName = "DownRight"},
                new StarConfig { offsetX =  -corner, offsetY = corner, starName = "UpLeft"},
                new StarConfig { offsetX =  -corner, offsetY = -corner, starName = "DownLeft"},
            };
        }
    }
    
    private void UpdateStars()
    {
        for (int i = 0; i < stars.Count; i++)
        {
            var sc = stars[i];
            if (sc.instance == null) continue;
            sc.instance.localPosition = new Vector3(
                gapCenterX + sc.offsetX,
                gapCenterY + sc.offsetY,
                0f
            );
        }
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
        ResetStarsIfEmpty();
        Apply();
    }

}
