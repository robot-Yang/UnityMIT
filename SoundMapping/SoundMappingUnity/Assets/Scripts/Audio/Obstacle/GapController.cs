using System.Linq;
using UnityEngine;

public class GapsController : MonoBehaviour
{
    [Header("Global gap width applied to all gaps")]
    public float globalGapWidth = 5f;

    [Header("Global gap size applied to all gaps")]
    public float globalGapSize = 6f;

    [Header("Spacing between gaps along Z")]
    public float gapOffsetZ = 10f;

    void OnValidate()
    {
        Apply();
    }

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
            ls.z = globalGapWidth;
            g.leftWall.localScale = ls;
            Vector3 rs = g.rightWall.localScale;
            rs.z = globalGapWidth;
            g.rightWall.localScale = rs;

            // Reposition along Z
            Vector3 p = g.transform.localPosition;
            p.z = startZ + i * gapOffsetZ;
            g.transform.localPosition = p;

            g.gapWidth = globalGapSize;

            // Apply layout (this clamps gapCenterX too)
            g.Apply();
        }
    }
}
