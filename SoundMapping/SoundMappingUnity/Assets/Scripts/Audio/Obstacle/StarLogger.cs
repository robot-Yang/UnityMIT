using System.Collections.Generic;
using System.IO;
using UnityEngine;
using System;
#if UNITY_EDITOR
using UnityEditor;
#endif

// --- Structure de données ---
[System.Serializable]
public class StarCollectionRecord
{
    public float t; // Time Collected
    public string name; // Star Name
    public int droneId;
    public float x, y, z; // Position
}

[System.Serializable]
public class StarCollectionLog
{
    public List<StarCollectionRecord> records = new List<StarCollectionRecord>();
}

// --- Logique de sauvegarde statique ---
public static class StarLogger
{
    private static StarCollectionLog _log = new StarCollectionLog();
    private static readonly string LogFileName = "stars.json";

    public static void RecordStar(string starName, float timeCollected, int droneId, Vector3 position)
    {
        _log.records.Add(new StarCollectionRecord
        {
            t = timeCollected,
            name = starName,
            droneId = droneId,
            x = position.x,
            y = position.y,
            z = position.z
        });
    }

    public static void SaveLog()
    {
        if (_log.records.Count == 0) return;

        // Récupération de l'ID PID et du chemin de sauvegarde (simplifié pour l'exemple)
        string pid = "PID_Default";
        var sceneSelectorType = Type.GetType("SceneSelectorScript");
        if (sceneSelectorType != null)
        {
            var pidField = sceneSelectorType.GetField("pid");
            if (pidField != null) pid = (string)pidField.GetValue(null);
        }

        string root = Path.Combine(Application.dataPath, "Data", pid, "Trajectories");
        if (!Directory.Exists(root)) Directory.CreateDirectory(root);

        string fullPath = Path.Combine(root, LogFileName);
        string json = JsonUtility.ToJson(_log, true);
        
        File.WriteAllText(fullPath, json);
        
        #if UNITY_EDITOR
        Debug.Log($"[StarLogger] Saved star log to: {fullPath}");
        AssetDatabase.Refresh();
        #endif
    }

    public static void ClearLog()
    {
        _log = new StarCollectionLog();
    }
}