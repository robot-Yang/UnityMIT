using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using UnityEngine;

public class Gap : MonoBehaviour
{
    [Header("Corridor")]
    [Tooltip("Total width of the corridor in world units.")]
    public float corridorWidth = 20f;   // e.g. 20 → from -10 to +10

    [Header("Gap")]
    [Tooltip("X position of the center of the gap, in parent's local space.")]
    [Range(-30, 30)]
    public float gapCenterX;
    
    [HideInInspector] 
    public float gapWidth;

    [Header("Walls (children of this object)")]
    public Transform leftWall;
    public Transform rightWall;

    public void Apply()
    {
        if (leftWall == null || rightWall == null || corridorWidth <= 0f)
            return;

        float halfCorridor = corridorWidth * 0.5f;

        // Clamp gap size to corridor
        gapWidth = Mathf.Clamp(gapWidth, 0f, corridorWidth);

        // Clamp center so the gap always stays inside the corridor
        float maxCenter = halfCorridor - gapWidth * 0.5f;
        gapCenterX = Mathf.Clamp(gapCenterX, -maxCenter, maxCenter);

        float halfGap = gapWidth * 0.5f;

        float leftEdge  = -halfCorridor;   // x min of corridor
        float rightEdge =  halfCorridor;   // x max of corridor

        float gapLeft  = gapCenterX - halfGap;
        float gapRight = gapCenterX + halfGap;

        // ---------- LEFT WALL : [leftEdge, gapLeft] ----------
        float leftWidth = Mathf.Max(0f, gapLeft - leftEdge);
        Vector3 ls = leftWall.localScale;
        ls.x = leftWidth;                      // width = scale.x (1 → 1 unit)
        leftWall.localScale = ls;

        Vector3 lp = leftWall.localPosition;
        lp.x = leftEdge + leftWidth * 0.5f;    // center of its interval
        leftWall.localPosition = lp;

        // ---------- RIGHT WALL : [gapRight, rightEdge] ----------
        float rightWidth = Mathf.Max(0f, rightEdge - gapRight);
        Vector3 rs = rightWall.localScale;
        rs.x = rightWidth;
        rightWall.localScale = rs;

        Vector3 rp = rightWall.localPosition;
        rp.x = rightEdge - rightWidth * 0.5f;
        rightWall.localPosition = rp;
    }

    private void OnValidate()
    {
        Apply();
    }
}

