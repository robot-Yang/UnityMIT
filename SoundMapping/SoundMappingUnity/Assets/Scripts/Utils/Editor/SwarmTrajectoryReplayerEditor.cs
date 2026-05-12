#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using System;
using System.IO;

[CustomEditor(typeof(SwarmTrajectoryReplayer))]
public class SwarmTrajectoryReplayerEditor : Editor
{
    private void OnEnable()
    {
        EditorApplication.update += OnEditorUpdate;
    }

    private void OnDisable()
    {
        EditorApplication.update -= OnEditorUpdate;
    }

    private void OnEditorUpdate()
    {
        if (Application.isPlaying)
        {
            Repaint();
        }
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        AutoFillMostRecentIfEmpty();
        DrawDefaultInspector();

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("File Picker", EditorStyles.boldLabel);
        if (GUILayout.Button("Choose JSON for jsonFilePath"))
        {
            string baseDir = System.IO.Path.Combine(Application.dataPath, "Data", "default");
            string selected = EditorUtility.OpenFilePanel(
                "Select Trajectory JSON",
                baseDir,
                "json");
            if (!string.IsNullOrEmpty(selected))
            {
                SerializedProperty pathProp = serializedObject.FindProperty("jsonFilePath");
                if (pathProp != null)
                {
                    pathProp.stringValue = selected;
                }

                SerializedProperty assetProp = serializedObject.FindProperty("jsonFileAsset");
                if (assetProp != null)
                {
                    assetProp.objectReferenceValue = null;
                }

                serializedObject.ApplyModifiedProperties();
                EditorUtility.SetDirty(target);
            }
        }

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Replay Controls", EditorStyles.boldLabel);

        SwarmTrajectoryReplayer replayer = (SwarmTrajectoryReplayer)target;

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Progress (Seconds)", EditorStyles.boldLabel);
        using (new EditorGUI.DisabledScope(!Application.isPlaying))
        {
            float duration = replayer.GetDurationSeconds();
            float current = replayer.GetTimeSeconds();
            EditorGUI.BeginChangeCheck();
            float next = EditorGUILayout.Slider("Jump (s)", current, 0f, Mathf.Max(0.01f, duration));
            if (EditorGUI.EndChangeCheck())
            {
                replayer.SetTimeSeconds(next);
            }

            int frames = replayer.GetFrameCount();
            if (frames > 0)
            {
                EditorGUILayout.LabelField("Frames", frames.ToString());
                EditorGUILayout.LabelField("Time", $"{current:F2} / {duration:F2} s");
            }
        }

        using (new EditorGUI.DisabledScope(!Application.isPlaying))
        {
            if (GUILayout.Button("Load"))
            {
                replayer.Load();
            }

            if (GUILayout.Button("Play"))
            {
                replayer.Play();
            }

            if (GUILayout.Button("Pause"))
            {
                replayer.Pause();
            }

            if (GUILayout.Button("Stop"))
            {
                replayer.Stop();
            }

            if (GUILayout.Button("Step"))
            {
                replayer.StepFrame();
            }
        }

        if (!Application.isPlaying)
        {
            EditorGUILayout.HelpBox("Enter Play Mode to control replay.", MessageType.Info);
        }

        serializedObject.ApplyModifiedProperties();
    }

    private void AutoFillMostRecentIfEmpty()
    {
        SerializedProperty pathProp = serializedObject.FindProperty("jsonFilePath");
        SerializedProperty assetProp = serializedObject.FindProperty("jsonFileAsset");
        if (pathProp == null || assetProp == null) return;
        if (assetProp.objectReferenceValue != null) return;
        if (!string.IsNullOrEmpty(pathProp.stringValue)) return;

        string mostRecent = FindMostRecentJsonUnderDataDefault();
        if (string.IsNullOrEmpty(mostRecent)) return;

        pathProp.stringValue = mostRecent;
        serializedObject.ApplyModifiedProperties();
        EditorUtility.SetDirty(target);
    }

    private string FindMostRecentJsonUnderDataDefault()
    {
        string baseDir = Path.Combine(Application.dataPath, "Data", "default");
        if (!Directory.Exists(baseDir))
        {
            string fallback = Path.Combine(Application.dataPath, "Data", "Default");
            if (Directory.Exists(fallback))
            {
                baseDir = fallback;
            }
            else
            {
                return null;
            }
        }

        string[] files = Directory.GetFiles(baseDir, "*.json", SearchOption.AllDirectories);
        if (files == null || files.Length == 0) return null;

        string mostRecent = null;
        DateTime mostRecentTime = DateTime.MinValue;
        foreach (string f in files)
        {
            DateTime t = File.GetLastWriteTimeUtc(f);
            if (t > mostRecentTime)
            {
                mostRecentTime = t;
                mostRecent = f;
            }
        }

        if (string.IsNullOrEmpty(mostRecent)) return null;

        string rel = mostRecent.Replace("\\", "/");
        string assetsPath = Application.dataPath.Replace("\\", "/");
        if (rel.StartsWith(assetsPath, StringComparison.OrdinalIgnoreCase))
        {
            rel = rel.Substring(assetsPath.Length).TrimStart('/');
        }
        return rel;
    }
}
#endif
