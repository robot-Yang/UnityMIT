// Assets/Editor/LiveLogTail.cs
// Live log tail window with severity filters (Log / Warning / Error) and sticky follow.
using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class LiveLogTail : EditorWindow
{
    private struct Entry
    {
        public DateTime time;
        public LogType type;
        public string msg;
    }

    // Data
    private readonly List<Entry> entries = new List<Entry>();
    private const int MaxLines = 2000;

    // UI state
    private Vector2 scroll;
    private bool follow;
    private bool showLog;
    private bool showWarning;
    private bool showError;

    // Pref keys
    const string PREF_FOLLOW = "LiveLogTail.Follow";
    const string PREF_SHOW_LOG = "LiveLogTail.ShowLog";
    const string PREF_SHOW_WARNING = "LiveLogTail.ShowWarning";
    const string PREF_SHOW_ERROR = "LiveLogTail.ShowError";

    [MenuItem("Window/Analysis/Live Log Tail")]
    public static void Open() => GetWindow<LiveLogTail>("Live Log Tail");

    void OnEnable()
    {
        // Defaults on first run
        follow = EditorPrefs.GetBool(PREF_FOLLOW, true);
        showLog = EditorPrefs.GetBool(PREF_SHOW_LOG, true);
        showWarning = EditorPrefs.GetBool(PREF_SHOW_WARNING, true);
        showError = EditorPrefs.GetBool(PREF_SHOW_ERROR, true);

        Application.logMessageReceivedThreaded += OnLog;
        EditorApplication.playModeStateChanged += _ => Repaint();
    }

    void OnDisable()
    {
        Application.logMessageReceivedThreaded -= OnLog;

        // Persist
        EditorPrefs.SetBool(PREF_FOLLOW, follow);
        EditorPrefs.SetBool(PREF_SHOW_LOG, showLog);
        EditorPrefs.SetBool(PREF_SHOW_WARNING, showWarning);
        EditorPrefs.SetBool(PREF_SHOW_ERROR, showError);
    }

    // Capture logs (threaded callback)
    void OnLog(string condition, string stackTrace, LogType type)
    {
        lock (entries)
        {
            entries.Add(new Entry
            {
                time = DateTime.Now,
                type = type,
                msg = condition
            });
            if (entries.Count > MaxLines)
                entries.RemoveRange(0, entries.Count - MaxLines);
        }

        // Schedule repaint on main thread
        EditorApplication.delayCall += Repaint;
    }

    // Keep follow sticky even while idle
    void OnInspectorUpdate()
    {
        if (follow)
        {
            scroll.y = float.MaxValue;
            Repaint();
        }
    }

    void OnGUI()
    {
        // Toolbar
        using (new EditorGUILayout.HorizontalScope(EditorStyles.toolbar))
        {
            if (GUILayout.Button("Clear", EditorStyles.toolbarButton, GUILayout.Width(60)))
            {
                lock (entries) entries.Clear();
                scroll.y = float.MaxValue;
            }

            GUILayout.Space(6);
            follow = GUILayout.Toggle(follow, "Follow", EditorStyles.toolbarButton, GUILayout.Width(60));

            GUILayout.Space(12);
            GUILayout.Label("Show:", EditorStyles.miniLabel);

            showLog = GUILayout.Toggle(showLog, "Log", EditorStyles.toolbarButton);
            showWarning = GUILayout.Toggle(showWarning, "Warning", EditorStyles.toolbarButton);
            showError = GUILayout.Toggle(showError, "Error", EditorStyles.toolbarButton);

            GUILayout.FlexibleSpace();

            // Quick counts
            GetCounts(out int cLog, out int cWarn, out int cErr);
            GUILayout.Label($"L:{cLog}  W:{cWarn}  E:{cErr}", EditorStyles.miniLabel);
        }

        // Build GUIContent once for performance
        string text = BuildFilteredText();

        // Measure and draw
        var style = new GUIStyle(EditorStyles.label) { richText = true, wordWrap = false };
        Vector2 size = style.CalcSize(new GUIContent(text));
        Rect viewRect = new Rect(0, 0, Mathf.Max(position.width - 20, size.x + 20), size.y + 10);

        scroll = GUI.BeginScrollView(
            new Rect(0, EditorStyles.toolbar.fixedHeight, position.width,
                position.height - EditorStyles.toolbar.fixedHeight),
            scroll, viewRect);

        GUI.Label(viewRect, text, style);
        GUI.EndScrollView();
    }

    string BuildFilteredText()
    {
        // Build a single string with Rich Text and minimal allocations
        System.Text.StringBuilder sb = new System.Text.StringBuilder(entries.Count * 32);

        lock (entries)
        {
            for (int i = 0; i < entries.Count; i++)
            {
                var e = entries[i];

                // Filter by severity toggles
                if (e.type == LogType.Log && !showLog) continue;
                if (e.type == LogType.Warning && !showWarning) continue;
                if ((e.type == LogType.Error || e.type == LogType.Exception || e.type == LogType.Assert) && !showError) continue;

                // Color per type
                // (Uses simple colors that read well on both skins; you can tweak if desired)
                string hex =
                    e.type == LogType.Warning ? "E5A50A" :
                    (e.type == LogType.Error || e.type == LogType.Exception || e.type == LogType.Assert) ? "E01B24" :
                    "C0C0C0";

                sb.AppendFormat("<color=#{0}>{1:HH:mm:ss.fff} [{2}]</color> ", hex, e.time, e.type);
                sb.Append(e.msg);
                if (i < entries.Count - 1) sb.Append('\n');
            }
        }

        return sb.ToString();
    }

    void GetCounts(out int logs, out int warns, out int errs)
    {
        logs = warns = errs = 0;
        lock (entries)
        {
            for (int i = 0; i < entries.Count; i++)
            {
                switch (entries[i].type)
                {
                    case LogType.Log: logs++; break;
                    case LogType.Warning: warns++; break;
                    case LogType.Error:
                    case LogType.Exception:
                    case LogType.Assert: errs++; break;
                }
            }
        }
    }
}
