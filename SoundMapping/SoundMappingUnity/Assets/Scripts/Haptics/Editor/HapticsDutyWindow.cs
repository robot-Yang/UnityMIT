using UnityEditor;
using UnityEngine;

public class HapticsDutyWindow : EditorWindow
{
    const int TARGET_ROW = 2;

    [MenuItem("Window/Haptics/Duty Viewer")]
    public static void ShowWindow()
    {
        var window = GetWindow<HapticsDutyWindow>("Duty Viewer");
        window.minSize = new Vector2(260, 180);
    }

    void OnEnable() => EditorApplication.update += Repaint;
    void OnDisable() => EditorApplication.update -= Repaint;

    void OnGUI()
    {
        if (!Application.isPlaying)
        {
            EditorGUILayout.HelpBox("Enter Play Mode to see live duties.", MessageType.Info);
            return;
        }

        if (!HapticsTest.TryGetRowDuty(TARGET_ROW, out var duties))
        {
            EditorGUILayout.HelpBox("No HapticsTest active or row out of range.", MessageType.Warning);
            return;
        }

        EditorGUILayout.LabelField($"Row {TARGET_ROW} duty snapshot", EditorStyles.boldLabel);
        EditorGUILayout.LabelField($"Values: {string.Join(", ", duties)}");

        Rect rect = GUILayoutUtility.GetRect(10, 120, 10, 120);
        DrawBars(rect, duties);
    }

    void DrawBars(Rect rect, int[] duties)
    {
        if (duties == null || duties.Length == 0) return;
        float barWidth = rect.width / duties.Length;
        for (int i = 0; i < duties.Length; i++)
        {
            float h01 = duties[i] / 14f; // DUTY_MAX
            float h = rect.height * Mathf.Clamp01(h01);
            Rect bar = new Rect(rect.x + i * barWidth + 4, rect.yMax - h, barWidth - 8, h);
            EditorGUI.DrawRect(bar, new Color(0.2f, 0.7f, 1f, 0.8f));
            EditorGUI.DropShadowLabel(new Rect(bar.x, bar.y - 18, bar.width, 16), duties[i].ToString());
        }
        Handles.DrawSolidRectangleWithOutline(rect, Color.clear, new Color(0, 0, 0, 0.4f));
    }
}
