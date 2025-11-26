using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class SceneSelectorScript : MonoBehaviour
{

    private string lastLoadedScene = null;
    public static int experimentNumber = 0;

    [HideInInspector]public bool isLoading = false;
    
    [HideInInspector] public List<string> scenes = new List<string>();
    [HideInInspector] public static List<string> scenesPlayed = new List<string>();

    [HideInInspector] private string setupScene = "Setup";
    [HideInInspector] public string ObstacleFPV = "FPVObs";
    [HideInInspector] public string ObstacleFPV1 = "FPVObs_1";
    [HideInInspector] public string ObstacleTPV = "TDVObs";
    [HideInInspector] public string ObstacleTPV1 = "TDVObs_1";

    [HideInInspector] public string CollectibleFPV = "FPVCollectibles";
    [HideInInspector] public string CollectibleFPV1 = "FPVCollectibles_1";
    [HideInInspector] public string CollectibleTPV = "TDVCollectibles";
    [HideInInspector] public string CollectibleTPV1 = "TDVCollectibles_1";

    [HideInInspector] public string assetPathTraining = "Assets/Scenes/TrainingFinal";


    public static string pid = "default";
    public static bool _order = false;
    public static bool _haptics = true;

    public bool hapticsEnabled = true;

    // === NEW: autoload toggle ===
    [Header("Auto-load on Play")]
    public bool autoLoadOnStart = true;

    // Use the exact scene name (file name without .unity)
    public string autoLoadSceneName = "FPVObs_2";

    void Start()
    {
     //   XboxScreenRecorder.StartRecording();
        // For initial cleanup.
        StartCoroutine(UnloadAllScenesExcept("Scene Selector"));

        // If using the UnityEditor to populate scenes:
        #if UNITY_EDITOR
        string[] sceneGuids = UnityEditor.AssetDatabase.FindAssets("t:Scene", new[] { assetPathTraining });
        foreach (string guid in sceneGuids)
        {
            string scenePath = UnityEditor.AssetDatabase.GUIDToAssetPath(guid);
            string sceneName = System.IO.Path.GetFileNameWithoutExtension(scenePath);
            scenes.Add(sceneName);
        }
        #endif
        
        // ✅ Use the experiment flow so all flags are set and swarm spawns
        if (autoLoadOnStart && !string.IsNullOrEmpty(autoLoadSceneName))
        {
            AutoSelectThroughExperimentFlow(autoLoadSceneName);  // <— use this
            // (Remove the direct StartCoroutine(LoadTrainingScene(...)) call.)
        }
    }

    private void AutoSelectThroughExperimentFlow(string sceneName)
    {
        _haptics = hapticsEnabled;

        // if your GUI disabling is required for state, keep it (guard against missing component)
        var gui = GetComponent<ExperimentSetupS>();
        if (gui) gui.GUIIDisable();

        scenesPlayed = new List<string>(scenes);
        addStudyScene();

        // Position experimentNumber so NextScene() advances to the desired scene
        experimentNumber = scenesPlayed.IndexOf(sceneName) - 1;

        if (experimentNumber < -1)
        {
            Debug.LogWarning($"[SceneSelector] '{sceneName}' not found in scenesPlayed; falling back to direct load.");
            StartCoroutine(LoadTrainingScene(sceneName)); // last-resort fallback
            return;
        }

        NextScene(); // this will call SelectTraining(...) → LoadTrainingScene(...)
    }

    public void OnHapticsChanged()
    {
    }

    IEnumerator UnloadAllScenesExcept(string sceneToKeep)
    {
        isLoading = true;
        List<Scene> loadedScenes = new List<Scene>();
        print(SceneManager.sceneCount);
        for (int i = 0; i < SceneManager.sceneCount; i++)
        {
            loadedScenes.Add(SceneManager.GetSceneAt(i));
        }

        foreach (Scene scene in loadedScenes)
        {
            if (scene.name != sceneToKeep)
            {
                AsyncOperation op = SceneManager.UnloadSceneAsync(scene);
                if (op != null)
                {
                    yield return new WaitUntil(() => op.isDone);
                    Debug.Log($"Unloaded scene: {scene.name}");
                }
                else
                {
                    Debug.LogWarning($"UnloadSceneAsync returned null for scene {scene.name}");
                }
            }
        }
        isLoading = false;
    }

    IEnumerator LoadTrainingScene(string sceneName)
    {
        print("Loading Scene: " + sceneName);
        if (isLoading)
        {
            Debug.LogWarning("Scene loading already in progress.");
            yield break;
        }
        // Unload all scenes except the persistent one.
        yield return StartCoroutine(UnloadAllScenesExcept("Scene Selector"));

        // Load the new training scene additively.
        AsyncOperation loadOp = SceneManager.LoadSceneAsync(sceneName, LoadSceneMode.Additive);
        yield return new WaitUntil(() => loadOp.isDone);
        lastLoadedScene = sceneName;
        Debug.Log($"Loaded training scene: {sceneName}");

        
        // Load the setup scene additively.
        AsyncOperation setupLoadOp = SceneManager.LoadSceneAsync(setupScene, LoadSceneMode.Additive);
        yield return new WaitUntil(() => setupLoadOp.isDone);
        Debug.Log($"Loaded setup scene: {setupScene}");
    }

    public void SelectTraining(string sceneName)
    {
        StartCoroutine(LoadTrainingScene(sceneName));
    }

    public void SelectTrainingFromButton(string sceneName)
    {
        _haptics = hapticsEnabled;
        this.GetComponent<ExperimentSetupS>().GUIIDisable();
        
        scenesPlayed = new List<string>(scenes);
        addStudyScene();

        experimentNumber = scenesPlayed.IndexOf(sceneName) - 1;
        
        print("Experiment Number fron training button: " + experimentNumber);
        NextScene();
    }

    public void StartTraining(string PID)
    {

        hapticsEnabled = _haptics;

        // Set up your experiment order.
        scenesPlayed = new List<string>(scenes);

        //scenesPlayed.Clear();

        addStudyScene();


        experimentNumber = -1;
//        print("Haptics: " + Haptics + " Order: " + Order + " PID: " + PID);
        NextScene();
    }

    public void addStudyScene()
    {
        
        if (!_order)
        {
            scenesPlayed.Add(ObstacleFPV);

            scenesPlayed.Add(ObstacleFPV1);
            tutorialPlayed.Add(scenesPlayed.Count - 1);

          //  scenesPlayed.Add(ObstacleFPV);
            scenesPlayed.Add(ObstacleTPV);
            tutorialPlayed.Add(scenesPlayed.Count - 1);

            scenesPlayed.Add(ObstacleTPV1);
            tutorialPlayed.Add(scenesPlayed.Count - 1);

            scenesPlayed.Add(CollectibleFPV);

         //   scenesPlayed.Add(ObstacleTPV);
            scenesPlayed.Add(CollectibleFPV1);
            tutorialPlayed.Add(scenesPlayed.Count - 1);

        //    scenesPlayed.Add(CollectibleFPV);
            scenesPlayed.Add(CollectibleTPV);
            tutorialPlayed.Add(scenesPlayed.Count - 1);

            scenesPlayed.Add(CollectibleTPV1);
            tutorialPlayed.Add(scenesPlayed.Count - 1);

        //    scenesPlayed.Add(CollectibleTPV);
        }
        else
        {
            scenesPlayed.Add(ObstacleTPV);

            scenesPlayed.Add(ObstacleTPV1);
            tutorialPlayed.Add(scenesPlayed.Count - 1);
           // scenesPlayed.Add(ObstacleTPV);
            scenesPlayed.Add(ObstacleFPV);
            tutorialPlayed.Add(scenesPlayed.Count - 1);

            scenesPlayed.Add(ObstacleFPV1);
            tutorialPlayed.Add(scenesPlayed.Count - 1);

            //  scenesPlayed.Add(ObstacleFPV);
            scenesPlayed.Add(CollectibleTPV);

            scenesPlayed.Add(CollectibleTPV1);
            tutorialPlayed.Add(scenesPlayed.Count - 1);
            //  scenesPlayed.Add(CollectibleTPV);
            scenesPlayed.Add(CollectibleFPV);
            tutorialPlayed.Add(scenesPlayed.Count - 1);

            scenesPlayed.Add(CollectibleFPV1);
            tutorialPlayed.Add(scenesPlayed.Count - 1);

            //   scenesPlayed.Add(CollectibleFPV);
        }
    }

    public static void nextScene()
    {
        GameObject.FindObjectOfType<SceneSelectorScript>().NextScene();
    }

    public void NextScene()
    {
        experimentNumber++;
        // if (experimentNumber >= scenesPlayed.Count)
        // {
        //     // End of experiment; unload all non-persistent scenes.
        //     StartCoroutine(UnloadAllScenesExcept("Scene Selector"));
        //     return;
        // }
        print("Experiment Number: " + experimentNumber);
        SelectTraining(scenesPlayed[experimentNumber]);
    }

    public static List<int> tutorialPlayed = new List<int>();
    public static bool needToWatchTutorial()
    {
        //print("Experiment Number: " + tutorialPlayed.Count);
        if(tutorialPlayed.Contains(experimentNumber))
        {
            return false;
        }
        else
        { 
            tutorialPlayed.Add(experimentNumber);
            return true;
        }
    }

    public void ResetScene()
    {
        if (experimentNumber >= 0 && experimentNumber < scenesPlayed.Count)
        {
            // Reload the current scene.
            SelectTraining(scenesPlayed[experimentNumber]);
        }
    }

    public static void reset()
    {
        GameObject.FindObjectOfType<SceneSelectorScript>().ResetScene();
    }

    public static string getNameScene()
    {
        return scenesPlayed[experimentNumber];
    }
}
