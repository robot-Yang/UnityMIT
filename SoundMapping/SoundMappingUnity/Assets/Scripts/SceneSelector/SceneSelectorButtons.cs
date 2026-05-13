using UnityEngine;
using UnityEditor;
using System.IO;

[CustomEditor(typeof(SceneSelectorScript))]
public class SceneSelectorScriptEditor : Editor
{
    private const string SceneStudyPath = "Assets/Scenes/SceneStudy";

    public override void OnInspectorGUI()
    {
        // Draw the default inspector fields (any public fields, etc.)
        DrawDefaultInspector();

        // Reference to the actual script on the GameObject
        SceneSelectorScript myScript = (SceneSelectorScript)target;

        // Label for clarity

        //make  a text for Script.pid centered and in big font
        GUIStyle style = new GUIStyle();
        style.fontSize = 20;
        style.alignment = TextAnchor.MiddleCenter;
        style.normal.textColor = Color.white;
        EditorGUILayout.LabelField("ID: " + SceneSelectorScript.pid, style);
        //put in red the text Haptics if haptics is disabled
        if (!SceneSelectorScript._haptics)
        {
            style.normal.textColor = Color.red;
        }
        else
        {
            style.normal.textColor = Color.white;
        }
        EditorGUILayout.LabelField("Haptics: " + SceneSelectorScript._haptics, style);
        //same thing for order
        if (!SceneSelectorScript._order)
        {
            style.normal.textColor = Color.red;
        }
        else
        {
            style.normal.textColor = Color.white;
        }
        EditorGUILayout.LabelField("TDV First : " + SceneSelectorScript._order, style);

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Set FPVObs_2 Scene", EditorStyles.boldLabel);
        if (GUILayout.Button("Use FPVObs_2_startSqaure as FPVObs_2"))
        {
            ReplaceFpvObs2With("FPVObs_2_startSqaure");
        }
        if (GUILayout.Button("Use FPVObs_2_practice as FPVObs_2"))
        {
            ReplaceFpvObs2With("FPVObs_2_practice");
        }
        if (GUILayout.Button("Use FPVObs_2_trial_1 as FPVObs_2"))
        {
            ReplaceFpvObs2With("FPVObs_2_trial_1");
        }
        if (GUILayout.Button("Use FPVObs_2_trial_2 as FPVObs_2"))
        {
            ReplaceFpvObs2With("FPVObs_2_trial_2");
        }
        if (GUILayout.Button("Use FPVObs_3d as FPVObs_2"))
        {
            ReplaceFpvObs2With("FPVObs_3d");
        }

        // Find all scenes in "Assets/Scenes/Training"
        string[] sceneGuids = AssetDatabase.FindAssets("t:Scene", new[] { myScript.assetPathTraining });
        
        // Create a button for each scene found
        foreach (string guid in sceneGuids)
        {
            // Convert GUID to path, then extract the filename without extension
            string scenePath = AssetDatabase.GUIDToAssetPath(guid);
            string sceneName = Path.GetFileNameWithoutExtension(scenePath);

            if (GUILayout.Button(sceneName))
            {
                // Tell our script to load this scene (unload the previous one if any)
                myScript.SelectTrainingFromButton(sceneName);
            }
        }


        //start vertical layout
        EditorGUILayout.BeginVertical();

        EditorGUILayout.BeginHorizontal();

        //make a button for Demo Scene
        if (GUILayout.Button("Obstacles FPV "))
        {
            // Tell our script to load this scene (unload the previous one if any)
            myScript.SelectTrainingFromButton(myScript.ObstacleFPV);
        }

        if (GUILayout.Button("Obstacles TDV "))
        {
            // Tell our script to load this scene (unload the previous one if any)
            myScript.SelectTrainingFromButton(myScript.ObstacleTPV);
        }

        if (GUILayout.Button("Collectibles FPV "))
        {
            // Tell our script to load this scene (unload the previous one if any)
            myScript.SelectTrainingFromButton(myScript.CollectibleFPV);
        }

        if (GUILayout.Button("Collectibles TDV "))
        {
            // Tell our script to load this scene (unload the previous one if any)
            myScript.SelectTrainingFromButton(myScript.CollectibleTPV);
        }

        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal();

        //make a button for Demo Scene
        if (GUILayout.Button("Obstacles FPV 2"))
        {
            // Tell our script to load this scene (unload the previous one if any)
            myScript.SelectTrainingFromButton(myScript.ObstacleFPV1);
        }

        if (GUILayout.Button("Obstacles TDV 2"))
        {
            // Tell our script to load this scene (unload the previous one if any)
            myScript.SelectTrainingFromButton(myScript.ObstacleTPV1);
        }

        if (GUILayout.Button("Collectibles FPV 2"))
        {
            // Tell our script to load this scene (unload the previous one if any)
            myScript.SelectTrainingFromButton(myScript.CollectibleFPV1);
        }

        if (GUILayout.Button("Collectibles TDV 2"))
        {
            // Tell our script to load this scene (unload the previous one if any)
            myScript.SelectTrainingFromButton(myScript.CollectibleTPV1);
        }

        EditorGUILayout.EndHorizontal();

        //button unload all scenes

        //end vertical layout
        EditorGUILayout.EndVertical();





        //


    }

    private static void ReplaceFpvObs2With(string sourceSceneName)
    {
        string sourcePath = Path.Combine(SceneStudyPath, sourceSceneName + ".unity");
        string destPath = Path.Combine(SceneStudyPath, "FPVObs_2.unity");

        if (!File.Exists(sourcePath))
        {
            Debug.LogError("Source scene not found: " + sourcePath);
            return;
        }

        if (!EditorUtility.DisplayDialog(
                "Copy To FPVObs_2",
                $"Copy {sourceSceneName}.unity to FPVObs_2.unity? (Original stays untouched)",
                "Copy",
                "Cancel"))
        {
            return;
        }

        // Copy the source scene into FPVObs_2.unity without touching the original.
        File.Copy(sourcePath, destPath, true);

        AssetDatabase.ImportAsset(destPath);
        AssetDatabase.Refresh();
        Debug.Log($"FPVObs_2.unity updated from copy of {sourceSceneName}.unity");
    }
}
