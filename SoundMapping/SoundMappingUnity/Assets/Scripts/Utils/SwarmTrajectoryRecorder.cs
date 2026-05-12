// SwarmTrajectoryRecorder.cs
// Records swarm trajectories into JSON, labels "main group" per sample using your runtime network,
// supports adjustable recording frequency, safely saves on scene changes / quit, and
// stores a single "Run" timing window (start/stop) for downstream analysis.
// NOW also labels the embodied drone both in-file (embodiedId/embodiedName) and per-frame flag 'e'.
//
// Call from other scripts:
//   SwarmTrajectoryRecorder.MarkTrialStart("Run");  // level/timer starts
//   SwarmTrajectoryRecorder.MarkTrialStop("Run");   // level/timer ends
//
// Attach this to a persistent GameObject (e.g., in Setup).

using System;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using UnityEngine;
using UnityEngine.SceneManagement;

#if UNITY_EDITOR
using UnityEditor;
#endif

public class SwarmTrajectoryRecorder : MonoBehaviour
{
    // -------------------- Configuration --------------------
    [Header("Swarm root discovery")]
    public string swarmRootTag = "Swarm";
    public string swarmRootName = "Swarm";
    [Tooltip("How often to re-check child count and rediscover drones (sec). 0 = never.")]
    public float rescanChildrenEverySec = 1f;

    [Header("Drone discovery")]
    [Tooltip("Type name of a component on each drone (e.g., 'DroneController'). Empty = any Transform child.")]
    public string droneComponentTypeName = "DroneController";

    [Header("Sampling")]
    [Tooltip("Samples per second (<=0 = every Update).")]
    public float sampleHz = 30f;
    [Tooltip("Wait to start sampling until at least one drone is found.")]
    public bool waitForDronesToStart = true;

    [Header("Recording frequency")]
    [Tooltip("Records per second (independent of sampleHz). 0 = record every sample.")]
    public float recordHz = 0f;
    [Tooltip("If recordHz==0, record every Nth sample (1 = every sample).")]
    public int recordEveryNthSample = 1;

    [Header("Lifecycle")]
    [Tooltip("Keep this recorder across scene loads.")]
    public bool dontDestroyOnLoad = true;
    [Tooltip("If true, keep trying to discover drones after start.")]
    public bool lazyDiscover = true;

    [Header("Output location")]
    [Tooltip("Subfolder under PID folder.")]
    public string outSubfolder = "Trajectories";
    [Tooltip("Override participant/session id.")]
    public string pidOverride = "";

    [Header("Output naming")]
    [Tooltip("Override scene label in filename (otherwise inferred).")]
    public string sceneLabelOverride = "";
    [Tooltip("Optional custom token to replace '<haptics>_<order>' in filename (e.g., 'FPV_only').")]
    public string namingTagOverride = "";
    [Tooltip("Name of the setup scene (only used to disambiguate labels).")]
    public string setupSceneName = "Scene Selector";

    [Header("Quality of life")]
    [Tooltip("Autosave interval in seconds. 0 = off.")]
    public float autosaveEverySec = 0f;
    [Tooltip("Enable F7 hotkey to save immediately.")]
    public bool enableHotkeySave = true;

    [Header("Main group selection")]
    [Tooltip("Use runtime network (swarmModel.network) to choose main group.")]
    public bool useNetworkForMainGroup = true;
    [Tooltip("If network is unavailable this frame, include everyone (true) or use proximity fallback (false).")]
    public bool includeAllUntilNetworkReady = true;

    [Header("Proximity fallback (only if network unavailable and includeAllUntilNetworkReady=false)")]
    [Tooltip("Meters: drones closer than this are linked into the same group.")]
    public float linkDistance = 3f;
    [Tooltip("Smallest cluster that qualifies as the 'main group'.")]
    public int minMainGroupSize = 3;
    [Tooltip("Use XZ-plane distance for grouping (recommended). If false, use full 3D distance.")]
    public bool useXZDistance = true;

    [Header("Run labeling")]
    [Tooltip("Label name to enforce single-run policy on.")]
    public string runLabelName = "Run";
    [Tooltip("If true, only one 'Run' window (start->stop) will be recorded per file.")]
    public bool singleRunMode = true;

    [Header("Embodied drone labeling")]
    [Tooltip("If set, this transform is treated as the embodied drone.")]
    public Transform embodiedDroneOverride;
    [Tooltip("Fallback: try to find an object with this tag.")]
    public string embodiedTag = "Embodied";
    [Tooltip("Fallback: try to find an object with this exact name.")]
    public string embodiedName = "EmbodiedDrone";
    [Tooltip("If your drone component (e.g., DroneController) has a bool flag like 'isEmbodied'/'IsEmbodied', we'll use it.")]
    public string embodiedFlagFieldOrProperty = "isEmbodied";
    public string embodiedFlagAltProperty = "IsEmbodied";

    [Header("Scene layout capture")]
    [Tooltip("Include a snapshot of path/obstacle layout in the JSON output.")]
    public bool captureSceneLayout = true;
    [Tooltip("Root GameObject name that contains path segments.")]
    public string pathRootName = "Path Holder";
    [Tooltip("Root GameObject name that contains obstacles.")]
    public string obstacleRootName = "Obstacles";
    [Tooltip("Path object name prefix (fallback mode).")]
    public string pathNamePrefix = "Path";
    [Tooltip("Obstacle tag (fallback mode).")]
    public string obstacleTag = "Obstacle";
    [Tooltip("Name of the starting square path.")]
    public string startSquareName = "Starting_square";
    [Tooltip("Name of the starting line path.")]
    public string startLineName = "Starting_line";
    [Tooltip("Name of the ending line path.")]
    public string endLineName = "Ending_line";
    [Tooltip("Multiply yaw by this value (matches plot_scene_positions_randomized.py).")]
    public float layoutYawSign = -1f;
    [Tooltip("Include inactive GameObjects when building layout.")]
    public bool layoutIncludeInactive = false;
    [Tooltip("How often to retry layout capture until success (sec).")]
    public float layoutCaptureRetrySec = 0.5f;
    [Tooltip("Stop retrying after this many seconds (0 = keep trying).")]
    public float layoutCaptureMaxWaitSec = 5f;
    [Tooltip("If true, allow an empty layout snapshot to be stored.")]
    public bool layoutCaptureAllowEmpty = false;

    // -------------------- Internals --------------------
    public Transform swarmRoot;
    private readonly Dictionary<int, DroneTraj> _trajById = new Dictionary<int, DroneTraj>();
    private readonly List<Transform> _droneTransforms = new List<Transform>();

    private float _accum, _discoverTimer, _swarmFindTimer, _childrenRescanTimer, _autosaveTimer;
    private int _lastChildCount = -1;
    private bool _samplingEnabled;

    // Recording schedule
    private float _recordAccum = 0f;
    private int _sampleIndex = 0;

    // UTC anchor for per-frame timestamps
    private long _utcStartMs = 0;
    private float _utcStartRealtime = 0f;

    // Singleton + save debounce
    private static SwarmTrajectoryRecorder _instance;
    private enum SaveReason { Auto, Manual, Final }
    private bool _finalized;
    private float _lastSaveRealtime;
    private const float SaveDebounceSec = 0.25f;

    // Scene/file labeling
    private string _sceneLabelForThisRun = null;
    private bool _justClearedForNewScene = false;

    // Coroutine to ensure swarm exists after scene load
    private Coroutine _ensureSwarmCoro;

    // --- Single-run state ---
    private bool _runFinalized = false;     // a 'Run' window has been closed in this file

    // --- Embodied detection cache ---
    private Transform _embodiedTransform;
    private int _embodiedStableId = int.MinValue;

    // --- Scene layout cache ---
    private SceneLayout _sceneLayoutCache;
    private bool _layoutCaptured;
    private float _layoutRetryTimer;
    private float _layoutElapsed;

    // -------------------- Data types --------------------
    [Serializable]
    public struct TrajFrame
    {
        public float t;
        public float x, y, z;
        public byte g; // 0 = not in main group; 1 = in main group
        public byte e; // 1 = embodied drone at this frame, 0 = not embodied
        // Drone orientation (Unity world rotation quaternion)
        public float qx, qy, qz, qw;
        // UTC timestamp in milliseconds since Unix epoch
        public long utcMs;
    }

    [Serializable]
    public class DroneTraj
    {
        public int id;
        public string name;
        public List<TrajFrame> frames = new List<TrajFrame>(4096);
    }

    [Serializable]
    public class TrialWindow
    {
        public string label;        // e.g., "Run", "Network"
        public float startGameTime; // Time.time when started
        public float startRealtime; // Time.realtimeSinceStartup when started
        public float endGameTime;   // Time.time when ended (0 if still open)
        public float endRealtime;   // Time.realtimeSinceStartup when ended (0 if open)
    }

    [Serializable]
    public class TrajectoryLog
    {
        public string scene;   // active scene at save time (for reference)
        public string pid;
        public string haptics; // "H" / "NH"
        public string order;   // "O" / "NO"
        public float sampleHz; // sampling cadence (record cadence implied by data)
        public List<DroneTraj> trajectories = new List<DroneTraj>();
        public List<TrialWindow> trials = new List<TrialWindow>(); // timing windows

        // --- NEW: embodied metadata written once per file ---
        public int embodiedId;        // -2147483648 (int.MinValue) if unknown
        public string embodiedName;   // empty if unknown

        // --- NEW: scene layout snapshot ---
        public SceneLayout layout;

        // --- NEW: UTC anchors (ms since Unix epoch) ---
        public long utcStartMs;
        public long utcEndMs;
    }

    [Serializable]
    public class SceneLayout
    {
        public string scene;
        public int seed = -1; // unknown at runtime
        public List<LayoutCorner> corners = new List<LayoutCorner>();
        public List<LayoutPath> paths = new List<LayoutPath>();
        public List<LayoutObstacle> obstacles = new List<LayoutObstacle>();
    }

    [Serializable]
    public class LayoutCorner
    {
        public float cx, cz, size;
    }

    [Serializable]
    public class LayoutPath
    {
        public string name;
        public int go_id;
        public int tr_id;
        public float cx, cz;
        public float width, length;
        public float angle;
    }

    [Serializable]
    public class LayoutObstacle
    {
        public string name;
        public int go_id;
        public int tr_id;
        public float cx, cz;
        public float width, length;
        public float angle;
    }

    // Trial buffers
    private readonly List<TrialWindow> _trialsBuffer = new List<TrialWindow>();
    private TrialWindow _openTrial = null;

    // -------------------- Unity lifecycle --------------------
    private void Awake()
    {
        if (_instance != null && _instance != this) { Destroy(gameObject); return; }
        _instance = this;

        if (dontDestroyOnLoad) DontDestroyOnLoad(gameObject);
        SceneManager.sceneLoaded   += OnSceneLoaded;
        SceneManager.sceneUnloaded += OnSceneUnloaded; // save AFTER the old scene truly unloads

        _accum = _autosaveTimer = 0f;
        _recordAccum = 0f;
        _sampleIndex = 0;

        ResetUtcAnchor();
    }

    private void OnDestroy()
    {
        if (_instance == this) _instance = null;
        SceneManager.sceneLoaded   -= OnSceneLoaded;
        SceneManager.sceneUnloaded -= OnSceneUnloaded;
    }

    private void OnEnable()
    {
        TryFindSwarmRootNow();
        CollectDrones();
        RefreshEmbodiedLabeling();
        _samplingEnabled = !waitForDronesToStart || _droneTransforms.Count > 0;
    }

    private void OnApplicationQuit() => TrySave(SaveReason.Final);

    private void OnDisable()
    {
        // Always flush whatever we have
        TrySave(SaveReason.Final);
    }

    private void OnSceneUnloaded(Scene s)
    {
        // Flush data from the scene that just finished unloading
        bool any = false;
        foreach (var kv in _trajById) { if (kv.Value.frames.Count > 0) { any = true; break; } }
        if (any)
        {
            Debug.Log($"[SwarmTrajectoryRecorder] Scene unloaded -> saving previous scene '{s.name}'");
            TrySave(SaveReason.Final);
        }

        // Prepare for next scene
        ClearBuffers();
    }

    private void OnSceneLoaded(Scene s, LoadSceneMode m)
    {
        if (!_justClearedForNewScene) ClearBuffers();
        _justClearedForNewScene = false;
        BeginSceneRecording(s);
    }

    private void Update()
    {
        if (enableHotkeySave && Input.GetKeyDown(KeyCode.F7)) TrySave(SaveReason.Manual);

        _swarmFindTimer += Time.deltaTime;
        if (!swarmRoot && _swarmFindTimer >= 0.5f)
        {
            _swarmFindTimer = 0f;
            TryFindSwarmRootNow();
            if (swarmRoot)
            {
                CollectDrones();
                RefreshEmbodiedLabeling();
                _samplingEnabled = !waitForDronesToStart || _droneTransforms.Count > 0;
            }
        }

        if (lazyDiscover && swarmRoot && _droneTransforms.Count == 0)
        {
            _discoverTimer += Time.deltaTime;
            if (_discoverTimer > 0.5f)
            {
                _discoverTimer = 0f;
                int before = _droneTransforms.Count;
                CollectDrones();
                if (_droneTransforms.Count > before)
                {
                    RefreshEmbodiedLabeling();
                    _samplingEnabled = true;
                }
            }
        }

        if (swarmRoot && rescanChildrenEverySec > 0f)
        {
            _childrenRescanTimer += Time.deltaTime;
            if (_childrenRescanTimer >= rescanChildrenEverySec)
            {
                _childrenRescanTimer = 0f;
                if (_lastChildCount != swarmRoot.childCount)
                {
                    _lastChildCount = swarmRoot.childCount;
                    CollectDrones();
                    RefreshEmbodiedLabeling();
                }
            }
        }

        TryCaptureSceneLayout();

        if (!_samplingEnabled || _droneTransforms.Count == 0) return;

        if (sampleHz <= 0f)
        {
            bool recordNow = ShouldRecordThisSample(Time.deltaTime);
            SampleOnce(recordNow);
        }
        else
        {
            float period = 1f / sampleHz;
            _accum += Time.deltaTime;
            while (_accum >= period)
            {
                bool recordNow = ShouldRecordThisSample(period);
                SampleOnce(recordNow);
                _accum -= period;
            }
        }

        if (autosaveEverySec > 0f)
        {
            _autosaveTimer += Time.deltaTime;
            if (_autosaveTimer >= autosaveEverySec)
            {
                _autosaveTimer = 0f;
                TrySave(SaveReason.Auto);
            }
        }
    }

    // -------------------- Public trial API --------------------
    public static void MarkTrialStart(string label = "Run")
    {
        if (_instance != null) _instance.MarkTrialStartInternal(label);
    }

    public static void MarkTrialStop(string label = "Run")
    {
        if (_instance != null) _instance.MarkTrialStopInternal(label);
    }

    // STRICT single-run policy: ignore duplicate starts, ignore stop when no run is open.
    private void MarkTrialStartInternal(string label)
    {
        label = string.IsNullOrEmpty(label) ? runLabelName : label;

        if (singleRunMode && _runFinalized)
        {
#if UNITY_EDITOR
            Debug.Log($"[SwarmTrajectoryRecorder] Ignoring Start('{label}') because a run was already finalized.");
#endif
            return;
        }
        if (_openTrial != null)
        {
#if UNITY_EDITOR
            Debug.Log($"[SwarmTrajectoryRecorder] Ignoring Start('{label}') because a run is already open.");
#endif
            return;
        }

        _openTrial = new TrialWindow
        {
            label = label,
            startGameTime = Time.time,
            startRealtime = Time.realtimeSinceStartup,
            endGameTime = 0f,
            endRealtime = 0f
        };
#if UNITY_EDITOR
        Debug.Log($"[SwarmTrajectoryRecorder] Trial START '{_openTrial.label}' at t={_openTrial.startGameTime:F2}s");
#endif
    }

    private void MarkTrialStopInternal(string label)
    {
        label = string.IsNullOrEmpty(label) ? runLabelName : label;

        if (_openTrial == null)
        {
#if UNITY_EDITOR
            Debug.Log($"[SwarmTrajectoryRecorder] Ignoring Stop('{label}') because no run is open.");
#endif
            return;
        }

        _openTrial.endGameTime = Time.time;
        _openTrial.endRealtime = Time.realtimeSinceStartup;
        _trialsBuffer.Add(_openTrial);
#if UNITY_EDITOR
        Debug.Log($"[SwarmTrajectoryRecorder] Trial STOP '{_openTrial.label}' at t={_openTrial.endGameTime:F2}s (dur {( _openTrial.endGameTime - _openTrial.startGameTime):F2}s)");
#endif
        if (singleRunMode && _openTrial.label == runLabelName) _runFinalized = true;
        _openTrial = null;
    }

    // -------------------- Scene prep / buffers --------------------
    private void BeginSceneRecording(Scene s)
    {
        // Stable label for THIS scene
        string activeName = s.IsValid() ? s.name : SceneManager.GetActiveScene().name;
        string label = !string.IsNullOrEmpty(sceneLabelOverride) ? sceneLabelOverride :
                       (ResolveSelectedSceneLabel() ?? ((activeName == setupSceneName) ? "UnknownScene" : activeName));
        _sceneLabelForThisRun = MakeFileSafe(label);

        // stop any previous search and start a fresh one
        if (_ensureSwarmCoro != null) StopCoroutine(_ensureSwarmCoro);
        _ensureSwarmCoro = StartCoroutine(EnsureSwarmAvailable());

        _accum = _autosaveTimer = 0f;
        _recordAccum = 0f; _sampleIndex = 0;
        _samplingEnabled = false; // will flip to true when drones found

        // reset single-run flags for the new file
        _runFinalized = false;
        _trialsBuffer.Clear();
        _openTrial = null;

        // also refresh embodied at scene begin
        RefreshEmbodiedLabeling();

        ResetUtcAnchor();

        // reset layout capture for the new scene
        _sceneLayoutCache = null;
        _layoutCaptured = false;
        _layoutRetryTimer = 0f;
        _layoutElapsed = 0f;
    }

    private void ResetUtcAnchor()
    {
        _utcStartMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        _utcStartRealtime = Time.realtimeSinceStartup;
    }

    private void ClearBuffers()
    {
        // We do NOT auto-close here; Save() on unload/quit/disable already captures any open run.
        _openTrial = null;
        _trialsBuffer.Clear();
        _trajById.Clear();
        _droneTransforms.Clear();
        _lastChildCount = -1;
        _justClearedForNewScene = true;
        _finalized = false; // allow a final save for the new scene
        _runFinalized = false; // new file can have its own single run

        _embodiedTransform = null;
        _embodiedStableId = int.MinValue;

        _sceneLayoutCache = null;
        _layoutCaptured = false;
        _layoutRetryTimer = 0f;
        _layoutElapsed = 0f;
    }

    private System.Collections.IEnumerator EnsureSwarmAvailable()
    {
        float timeout = 10f; // try up to 10s; set 0 for infinite
        float t0 = Time.unscaledTime;

        while (true)
        {
            TryFindSwarmRootNow();

            // If still not found, try to infer by scanning active scene roots for the drone component type
            if (!swarmRoot)
            {
                var type = GetTypeByName(droneComponentTypeName);
                if (type != null)
                {
                    var scene = SceneManager.GetActiveScene();
                    var roots = scene.IsValid() ? scene.GetRootGameObjects() : null;
                    if (roots != null)
                    {
                        foreach (var go in roots)
                        {
                            var tr = FindTransformHavingComponent(go.transform, type);
                            if (tr != null)
                            {
                                swarmRoot = tr.root;
                                break;
                            }
                        }
                    }
                }
            }

            if (swarmRoot)
            {
                CollectDrones();
                RefreshEmbodiedLabeling();
                if (_droneTransforms.Count > 0)
                {
                    _samplingEnabled = true;
                    Debug.Log($"[SwarmTrajectoryRecorder] Found Swarm with {_droneTransforms.Count} drones; recording enabled.");
                    yield break;
                }
            }

            if (timeout > 0f && (Time.unscaledTime - t0) > timeout)
            {
                Debug.LogWarning("[SwarmTrajectoryRecorder] Swarm not found within timeout; will keep idle and retry.");
                // light retry while idle
                while (!_samplingEnabled)
                {
                    TryFindSwarmRootNow();
                    if (swarmRoot)
                    {
                        CollectDrones();
                        RefreshEmbodiedLabeling();
                        if (_droneTransforms.Count > 0)
                        {
                            _samplingEnabled = true;
                            Debug.Log($"[SwarmTrajectoryRecorder] Late-found Swarm with {_droneTransforms.Count} drones; recording enabled.");
                            yield break;
                        }
                    }
                    yield return new WaitForSeconds(0.5f);
                }
            }

            yield return null; // next frame
        }
    }

    private Transform FindTransformHavingComponent(Transform root, Type compType)
    {
        if (root.GetComponent(compType) != null) return root;
        for (int i = 0; i < root.childCount; i++)
        {
            var hit = FindTransformHavingComponent(root.GetChild(i), compType);
            if (hit != null) return hit;
        }
        return null;
    }

    // -------------------- Recording schedule --------------------
    private bool ShouldRecordThisSample(float dt)
    {
        // Priority 1: rate-based throttle
        if (recordHz > 0f)
        {
            float recPeriod = 1f / recordHz;
            _recordAccum += dt;
            if (_recordAccum + 1e-6f >= recPeriod)
            {
                _recordAccum -= recPeriod;
                _sampleIndex++; // advance index for consistency
                return true;
            }
            _sampleIndex++;
            return false;
        }

        // Priority 2: decimation-based throttle
        if (recordEveryNthSample <= 1)
        {
            _sampleIndex++;
            return true; // record every sample
        }

        _sampleIndex++;
        return (_sampleIndex % recordEveryNthSample) == 0;
    }

    // -------------------- Discovery --------------------
    private void TryFindSwarmRootNow()
    {
        if (swarmRoot) return;

        if (!string.IsNullOrEmpty(swarmRootTag))
        {
            var byTag = GameObject.FindWithTag(swarmRootTag);
            if (byTag) { swarmRoot = byTag.transform; _lastChildCount = swarmRoot.childCount; return; }
        }
        if (!string.IsNullOrEmpty(swarmRootName))
        {
            var byName = GameObject.Find(swarmRootName);
            if (byName) { swarmRoot = byName.transform; _lastChildCount = swarmRoot.childCount; }
        }
    }

    private void CollectDrones()
    {
        _droneTransforms.Clear();
        if (!swarmRoot) return;

        var type = GetTypeByName(droneComponentTypeName);
        if (type != null)
        {
            var comps = swarmRoot.GetComponentsInChildren(type, true);
            foreach (var c in comps)
            {
                var tr = ((Component)c).transform;
                if (!_droneTransforms.Contains(tr)) _droneTransforms.Add(tr);
                EnsureTrajFor(tr);
            }
        }
        else
        {
            foreach (Transform tr in swarmRoot.GetComponentsInChildren<Transform>(true))
            {
                if (tr == swarmRoot) continue;
                if (!_droneTransforms.Contains(tr)) _droneTransforms.Add(tr);
                EnsureTrajFor(tr);
            }
        }
    }

    private void EnsureTrajFor(Transform tr)
    {
        int id = GetStableId(tr);
        if (!_trajById.ContainsKey(id))
            _trajById[id] = new DroneTraj { id = id, name = tr.name };
    }

    private static Type GetTypeByName(string typeName)
    {
        if (string.IsNullOrEmpty(typeName)) return null;
        foreach (var asm in AppDomain.CurrentDomain.GetAssemblies())
        {
            var t = asm.GetType(typeName);
            if (t != null) return t;
        }
        return null;
    }

    private int GetStableId(Transform t)
    {
        var compType = GetTypeByName(droneComponentTypeName);
        if (compType != null)
        {
            var comp = t.GetComponent(compType);
            if (comp != null)
            {
                var f = compType.GetField("Id") ?? compType.GetField("DroneId");
                if (f != null && f.FieldType == typeof(int)) return (int)f.GetValue(comp);
                var p = compType.GetProperty("Id") ?? compType.GetProperty("DroneId");
                if (p != null && p.PropertyType == typeof(int)) return (int)p.GetValue(comp, null);
            }
        }
        return t.GetInstanceID();
    }

    // -------------------- Embodied detection & labeling --------------------
    private void RefreshEmbodiedLabeling()
    {
        _embodiedTransform = TryFindEmbodiedTransform();
        _embodiedStableId = (_embodiedTransform != null) ? GetStableId(_embodiedTransform) : int.MinValue;

    }

    // -------------------- Scene layout capture --------------------
    private void TryCaptureSceneLayout()
    {
        if (!captureSceneLayout || _layoutCaptured) return;

        _layoutElapsed += Time.deltaTime;
        _layoutRetryTimer += Time.deltaTime;
        if (_layoutRetryTimer < layoutCaptureRetrySec) return;
        _layoutRetryTimer = 0f;

        var layout = BuildSceneLayout();
        if (layout == null) return;

        bool hasAny = (layout.paths.Count > 0) || (layout.obstacles.Count > 0) || (layout.corners.Count > 0);
        bool timedOut = (layoutCaptureMaxWaitSec > 0f) && (_layoutElapsed >= layoutCaptureMaxWaitSec);

        if (hasAny || layoutCaptureAllowEmpty || timedOut)
        {
            _sceneLayoutCache = layout;
            _layoutCaptured = true;
        }
    }

    private SceneLayout BuildSceneLayout()
    {
        var scene = SceneManager.GetActiveScene();
        if (!scene.IsValid()) return null;

        var layout = new SceneLayout
        {
            scene = scene.name,
            seed = -1
        };

        var pathRoot = !string.IsNullOrEmpty(pathRootName) ? GameObject.Find(pathRootName) : null;
        var obstacleRoot = !string.IsNullOrEmpty(obstacleRootName) ? GameObject.Find(obstacleRootName) : null;

        if (pathRoot != null || obstacleRoot != null)
        {
            if (pathRoot != null) CollectPathsFromRoot(pathRoot.transform, layout.paths);
            if (obstacleRoot != null) CollectObstaclesFromRoot(obstacleRoot.transform, layout.obstacles);
        }
        else
        {
            CollectLayoutFallback(layout);
        }

        layout.paths.Sort((a, b) => string.Compare(a.name, b.name, StringComparison.Ordinal));
        layout.obstacles.Sort((a, b) => string.Compare(a.name, b.name, StringComparison.Ordinal));

        return layout;
    }

    private void CollectPathsFromRoot(Transform root, List<LayoutPath> paths)
    {
        var transforms = root.GetComponentsInChildren<Transform>(layoutIncludeInactive);
        foreach (var tr in transforms)
        {
            if (tr == root) continue;
            if (!layoutIncludeInactive && !tr.gameObject.activeInHierarchy) continue;
            if (!IsPathName(tr.name)) continue;

            if (TryGetFootprint(tr, out var cx, out var cz, out var width, out var length, out var angle))
            {
                paths.Add(new LayoutPath
                {
                    name = tr.name,
                    go_id = tr.gameObject.GetInstanceID(),
                    tr_id = tr.GetInstanceID(),
                    cx = cx,
                    cz = cz,
                    width = width,
                    length = length,
                    angle = angle
                });
            }
        }
    }

    private void CollectObstaclesFromRoot(Transform root, List<LayoutObstacle> obstacles)
    {
        var transforms = root.GetComponentsInChildren<Transform>(layoutIncludeInactive);
        foreach (var tr in transforms)
        {
            if (tr == root) continue;
            if (!layoutIncludeInactive && !tr.gameObject.activeInHierarchy) continue;

            if (TryGetFootprint(tr, out var cx, out var cz, out var width, out var length, out var angle))
            {
                obstacles.Add(new LayoutObstacle
                {
                    name = tr.name,
                    go_id = tr.gameObject.GetInstanceID(),
                    tr_id = tr.GetInstanceID(),
                    cx = cx,
                    cz = cz,
                    width = width,
                    length = length,
                    angle = angle
                });
            }
        }
    }

    private void CollectLayoutFallback(SceneLayout layout)
    {
        var scene = SceneManager.GetActiveScene();
        if (!scene.IsValid()) return;

        var roots = scene.GetRootGameObjects();
        var all = new List<Transform>(256);
        foreach (var root in roots) CollectTransformsRecursive(root.transform, all);

        foreach (var tr in all)
        {
            if (!layoutIncludeInactive && !tr.gameObject.activeInHierarchy) continue;
            if (!string.IsNullOrEmpty(pathRootName) && tr.name == pathRootName) continue;
            if (!string.IsNullOrEmpty(obstacleRootName) && tr.name == obstacleRootName) continue;

            bool isPath = IsPathName(tr.name);
            bool isObstacle = false;
            if (!string.IsNullOrEmpty(obstacleTag))
            {
                try { isObstacle = tr.gameObject.CompareTag(obstacleTag); }
                catch { isObstacle = false; }
            }

            if (isPath && TryGetFootprint(tr, out var pcx, out var pcz, out var pwidth, out var plength, out var pangle))
            {
                layout.paths.Add(new LayoutPath
                {
                    name = tr.name,
                    go_id = tr.gameObject.GetInstanceID(),
                    tr_id = tr.GetInstanceID(),
                    cx = pcx,
                    cz = pcz,
                    width = pwidth,
                    length = plength,
                    angle = pangle
                });
            }

            if (isObstacle && TryGetFootprint(tr, out var ocx, out var ocz, out var owidth, out var olength, out var oangle))
            {
                layout.obstacles.Add(new LayoutObstacle
                {
                    name = tr.name,
                    go_id = tr.gameObject.GetInstanceID(),
                    tr_id = tr.GetInstanceID(),
                    cx = ocx,
                    cz = ocz,
                    width = owidth,
                    length = olength,
                    angle = oangle
                });
            }
        }
    }

    private void CollectTransformsRecursive(Transform tr, List<Transform> output)
    {
        if (!layoutIncludeInactive && !tr.gameObject.activeInHierarchy) return;
        output.Add(tr);
        for (int i = 0; i < tr.childCount; i++)
        {
            CollectTransformsRecursive(tr.GetChild(i), output);
        }
    }

    private bool IsPathName(string name)
    {
        if (string.IsNullOrEmpty(name)) return false;
        if (!string.IsNullOrEmpty(pathNamePrefix) && name.StartsWith(pathNamePrefix, StringComparison.Ordinal)) return true;
        if (!string.IsNullOrEmpty(startSquareName) && name == startSquareName) return true;
        if (!string.IsNullOrEmpty(startLineName) && name == startLineName) return true;
        if (!string.IsNullOrEmpty(endLineName) && name == endLineName) return true;
        return false;
    }

    private bool TryGetFootprint(Transform tr, out float cx, out float cz, out float width, out float length, out float angle)
    {
        cx = cz = width = length = angle = 0f;
        if (!tr) return false;
        if (!layoutIncludeInactive && !tr.gameObject.activeInHierarchy) return false;

        var go = tr.gameObject;
        Vector3 centerWorld = tr.position;
        Vector3 lossy = tr.lossyScale;
        float sx = Mathf.Abs(lossy.x);
        float sz = Mathf.Abs(lossy.z);

        var box = go.GetComponent<BoxCollider>();
        if (box != null)
        {
            if (!box.enabled) return false;
            width = box.size.x * sx;
            length = box.size.z * sz;
            centerWorld = tr.TransformPoint(box.center);
        }
        else
        {
            var capsule = go.GetComponent<CapsuleCollider>();
            if (capsule != null)
            {
                if (!capsule.enabled) return false;
                float radius = capsule.radius;
                float height = capsule.height;
                switch (capsule.direction)
                {
                    case 0:
                        width = height * sx;
                        length = (2f * radius) * sz;
                        break;
                    case 2:
                        width = (2f * radius) * sx;
                        length = height * sz;
                        break;
                    default:
                        width = (2f * radius) * sx;
                        length = (2f * radius) * sz;
                        break;
                }
                centerWorld = tr.TransformPoint(capsule.center);
            }
            else
            {
                width = sx;
                length = sz;
                centerWorld = tr.position;
            }
        }

        cx = centerWorld.x;
        cz = centerWorld.z;

        float yaw = tr.rotation.eulerAngles.y;
        if (yaw > 180f) yaw -= 360f;
        angle = layoutYawSign * yaw;

        return true;
    }

    private Transform TryFindEmbodiedTransform()
    {
        // 1) Inspector override
        if (embodiedDroneOverride) return embodiedDroneOverride;

        // 2) Tag
        if (!string.IsNullOrEmpty(embodiedTag))
        {
            try
            {
                var go = GameObject.FindWithTag(embodiedTag);
                if (go) return go.transform;
            }
            catch { /* tag may not exist */ }
        }

        // 3) Name
        if (!string.IsNullOrEmpty(embodiedName))
        {
            var byName = GameObject.Find(embodiedName);
            if (byName) return byName.transform;
        }

        // 4) HapticsTest.embodiedDrone (field or property), if present
        var htType = GetTypeByName("HapticsTest");
        if (htType != null)
        {
            var objs = UnityEngine.Object.FindObjectsOfType(htType);
            foreach (var o in objs)
            {
                Transform t = null;
                var f = htType.GetField("embodiedDrone");
                if (f != null && typeof(Transform).IsAssignableFrom(f.FieldType)) t = (Transform)f.GetValue(o);
                if (t == null)
                {
                    var p = htType.GetProperty("embodiedDrone");
                    if (p != null && typeof(Transform).IsAssignableFrom(p.PropertyType)) t = (Transform)p.GetValue(o, null);
                }
                if (t) return t;
            }
        }

        // 5) Look for a bool flag on the drone component type (e.g., DroneController.isEmbodied/IsEmbodied)
        var dcType = GetTypeByName(droneComponentTypeName);
        if (dcType != null && swarmRoot)
        {
            var comps = swarmRoot.GetComponentsInChildren(dcType, true);
            foreach (var c in comps)
            {
                bool isEmb = false;
                var f = dcType.GetField(embodiedFlagFieldOrProperty);
                if (f != null && f.FieldType == typeof(bool)) isEmb = (bool)f.GetValue(c);
                if (!isEmb)
                {
                    var p = dcType.GetProperty(embodiedFlagFieldOrProperty);
                    if (p != null && p.PropertyType == typeof(bool)) isEmb = (bool)p.GetValue(c, null);
                }
                if (!isEmb && !string.IsNullOrEmpty(embodiedFlagAltProperty))
                {
                    var p2 = dcType.GetProperty(embodiedFlagAltProperty);
                    if (p2 != null && p2.PropertyType == typeof(bool)) isEmb = (bool)p2.GetValue(c, null);
                    if (!isEmb)
                    {
                        var f2 = dcType.GetField(embodiedFlagAltProperty);
                        if (f2 != null && f2.FieldType == typeof(bool)) isEmb = (bool)f2.GetValue(c);
                    }
                }

                if (isEmb) return ((Component)c).transform;
            }
        }

        return null; // not found
    }

    // -------------------- Sampling --------------------
    private void SampleOnce(bool writeThisSample)
    {
        int n = _droneTransforms.Count;
        if (n == 0) return;

        float t = Time.time;
        long utcMs = _utcStartMs + (long)Math.Round((Time.realtimeSinceStartup - _utcStartRealtime) * 1000.0);

        if (!writeThisSample) return;

        // Refresh embodied selection every sample so runtime role switches are captured.
        RefreshEmbodiedLabeling();

        var positions = new Vector3[n];
        var rotations = new Quaternion[n];
        var ids       = new int[n];
        for (int i = 0; i < n; i++)
        {
            var tr = _droneTransforms[i];
            if (!tr) continue;
            positions[i] = tr.position;
            rotations[i] = tr.rotation;
            ids[i] = GetStableId(tr);
            if (!_trajById.TryGetValue(ids[i], out var _))
                _trajById[ids[i]] = new DroneTraj { id = ids[i], name = tr.name };
        }

        for (int i = 0; i < n; i++)
        {
            if (_trajById.TryGetValue(ids[i], out var traj))
            {
                // if name changed at runtime (rare), keep it updated
                traj.name = _droneTransforms[i] ? _droneTransforms[i].name : traj.name;
            }
        }

        var inMain = new bool[n];
        ComputeMainGroupFlags(positions, inMain);

        for (int i = 0; i < n; i++)
        {
            if (!_trajById.TryGetValue(ids[i], out var traj)) continue;
            Vector3 p = positions[i];
            traj.frames.Add(new TrajFrame
            {
                t = t,
                x = p.x, y = p.y, z = p.z,
                g = (byte)(inMain[i] ? 1 : 0),
                e = (byte)((ids[i] == _embodiedStableId) ? 1 : 0),
                qx = rotations[i].x, qy = rotations[i].y, qz = rotations[i].z, qw = rotations[i].w,
                utcMs = utcMs
            });
        }
    }

    // -------------------- Main-group logic --------------------
    private void ComputeMainGroupFlags(Vector3[] positions, bool[] inMain)
    {
        Array.Clear(inMain, 0, inMain.Length);
        int n = positions.Length;
        if (n == 0) return;

        if (useNetworkForMainGroup)
        {
            var net = swarmModel.network; // your runtime network
            if (net != null && net.largestComponent != null && net.largestComponent.Count > 0)
            {
                var mainSet = new HashSet<DroneFake>(net.largestComponent);
                for (int i = 0; i < n; i++)
                {
                    var tr = _droneTransforms[i];
                    if (!tr) continue;
                    var dc = tr.GetComponent<DroneController>();
                    var df = (dc != null) ? dc.droneFake : null;

                    bool isInMain =
                        (df != null && mainSet.Contains(df)) ||
                        (df != null && net.IsInMainNetwork(df));

                    inMain[i] = isInMain;
                }

                if (minMainGroupSize > 1)
                {
                    int count = 0; for (int i = 0; i < n; i++) if (inMain[i]) count++;
                    if (count < minMainGroupSize) Array.Clear(inMain, 0, n);
                }
                return;
            }

            if (includeAllUntilNetworkReady)
            {
                for (int i = 0; i < n; i++) inMain[i] = true;
                return;
            }
        }

        ProximityFallback(positions, inMain);
    }

    private void ProximityFallback(Vector3[] positions, bool[] inMain)
    {
        int n = positions.Length;
        if (n == 0) return;

        int[] parent = new int[n];
        int[] size = new int[n];
        for (int i = 0; i < n; i++) { parent[i] = i; size[i] = 1; }

        int Find(int a)
        {
            while (parent[a] != a) { parent[a] = parent[parent[a]]; a = parent[a]; }
            return a;
        }
        void Union(int a, int b)
        {
            a = Find(a); b = Find(b);
            if (a == b) return;
            if (size[a] < size[b]) { var t = a; a = b; b = t; }
            parent[b] = a; size[a] += size[b];
        }

        float r2 = linkDistance * linkDistance;
        for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++)
        {
            float dx = positions[i].x - positions[j].x;
            float dz = positions[i].z - positions[j].z;
            float d2 = useXZDistance
                ? (dx * dx + dz * dz)
                : (dx * dx + dz * dz + (positions[i].y - positions[j].y) * (positions[i].y - positions[j].y));
            if (d2 <= r2) Union(i, j);
        }

        var counts = new Dictionary<int, int>();
        int bestRoot = -1, best = 0;
        for (int i = 0; i < n; i++)
        {
            int r = Find(i);
            counts[r] = counts.TryGetValue(r, out var cur) ? (cur + 1) : 1;
            if (counts[r] > best) { best = counts[r]; bestRoot = r; }
        }

        if (best < Mathf.Max(1, minMainGroupSize))
        {
            Array.Clear(inMain, 0, n);
            return;
        }

        for (int i = 0; i < n; i++)
            inMain[i] = (Find(i) == bestRoot);
    }

    // -------------------- Saving --------------------
    private void TrySave(SaveReason reason)
    {
        if (_finalized && reason != SaveReason.Auto) return; // already wrote final file
        if (Time.realtimeSinceStartup - _lastSaveRealtime < SaveDebounceSec) return; // debounce
        _lastSaveRealtime = Time.realtimeSinceStartup;

        Save();

        if (reason == SaveReason.Final) _finalized = true;
#if UNITY_EDITOR
        Debug.Log($"[SwarmTrajectoryRecorder] Save() wrote file. finalized={_finalized} time={Time.time:F2}");
#endif
    }

    public void Save()
    {
        bool any = false;
        foreach (var kv in _trajById) { if (kv.Value.frames.Count > 0) { any = true; break; } }
        if (!any && _droneTransforms.Count > 0) SampleOnce(true); // ensure at least one sample

        // Auto-stop open run just before saving (so it is captured in this file)
        if (_openTrial != null && _openTrial.endGameTime <= 0f)
        {
            _openTrial.endGameTime = Time.time;
            _openTrial.endRealtime = Time.realtimeSinceStartup;
            _trialsBuffer.Add(_openTrial);
            if (singleRunMode && _openTrial.label == runLabelName) _runFinalized = true;
            _openTrial = null;
        }

        // Make sure embodied flags are up-to-date right before write
        RefreshEmbodiedLabeling();

        // Ensure layout snapshot is captured while the scene is still alive
        if (captureSceneLayout && !_layoutCaptured)
        {
            var layout = BuildSceneLayout();
            if (layout != null)
            {
                _sceneLayoutCache = layout;
                _layoutCaptured = true;
            }
        }

        var log = new TrajectoryLog
        {
            scene = SceneManager.GetActiveScene().name,  // for reference
            pid = ResolvePid(),
            haptics = ResolveHaptics(),
            order = ResolveOrder(),
            sampleHz = sampleHz <= 0 ? -1f : sampleHz,
            trajectories = new List<DroneTraj>(_trajById.Values),
            trials = new List<TrialWindow>(_trialsBuffer),

            // NEW: embodied metadata
            embodiedId = _embodiedStableId,
            embodiedName = _embodiedTransform ? _embodiedTransform.name : string.Empty,

            // NEW: scene layout snapshot
            layout = captureSceneLayout ? _sceneLayoutCache : null,

            // NEW: UTC anchors
            utcStartMs = _utcStartMs,
            utcEndMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()
        };

        string root;
#if UNITY_EDITOR
        root = Path.Combine(Application.dataPath, "Data", log.pid, outSubfolder);
#else
        root = Path.Combine(Application.persistentDataPath, "Data", log.pid, outSubfolder);
#endif
        Directory.CreateDirectory(root);

        // Use cached label for this scene (important when saving after scene switch)
        string safeScene = !string.IsNullOrEmpty(_sceneLabelForThisRun)
            ? _sceneLabelForThisRun
            : MakeFileSafe(SceneManager.GetActiveScene().name);

        string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string namingTag = ResolveNamingTag(log.haptics, log.order);
        string fileName = $"{safeScene}_{namingTag}_{stamp}_traj.json";
        string full = Path.Combine(root, fileName);

        File.WriteAllText(full, JsonUtility.ToJson(log, true));
        Debug.Log($"[SwarmTrajectoryRecorder] Saved {log.trajectories.Count} drones to:\n{full}");

#if UNITY_EDITOR
        if (full.StartsWith(Application.dataPath)) AssetDatabase.Refresh();
#endif
    }

    // -------------------- Helpers --------------------
    private static string MakeFileSafe(string s)
    {
        if (string.IsNullOrEmpty(s)) return "Scene";
        s = Regex.Replace(s, @"\s+", "_");
        foreach (char c in Path.GetInvalidFileNameChars()) s = s.Replace(c.ToString(), "");
        return s;
    }

    private string ResolveNamingTag(string haptics, string order)
    {
        if (!string.IsNullOrWhiteSpace(namingTagOverride))
        {
            string safe = MakeFileSafe(namingTagOverride.Trim());
            if (!string.IsNullOrEmpty(safe) && safe != "Scene")
                return safe;
        }
        return $"{haptics}_{order}";
    }

    private string ResolvePid()
    {
        if (!string.IsNullOrEmpty(pidOverride)) return pidOverride;
        var t = GetTypeByName("SceneSelectorScript");
        if (t != null)
        {
            var f = t.GetField("pid");
            if (f != null && f.FieldType == typeof(string))
            {
                var v = (string)f.GetValue(null);
                if (!string.IsNullOrEmpty(v)) return v;
            }
        }
        return "PID_Default";
    }

    private string ResolveHaptics()
    {
        var t = GetTypeByName("SceneSelectorScript");
        if (t != null)
        {
            var f = t.GetField("_haptics");
            if (f != null && f.FieldType == typeof(bool))
                return ((bool)f.GetValue(null)) ? "H" : "NH";
        }
        return "NH";
    }

    private string ResolveOrder()
    {
        var t = GetTypeByName("SceneSelectorScript");
        if (t != null)
        {
            var f = t.GetField("_order");
            if (f != null && f.FieldType == typeof(bool))
                return ((bool)f.GetValue(null)) ? "O" : "NO";
        }
        return "NO";
    }

    private string ResolveSelectedSceneLabel()
    {
        var t = GetTypeByName("SceneSelectorScript");
        if (t == null) return null;

        string[] fieldNames = { "selectedSceneName", "sceneToLoad", "targetScene", "detailScene", "SelectedLevel", "SelectedScene" };
        foreach (var fn in fieldNames)
        {
            var f = t.GetField(fn);
            if (f != null && f.FieldType == typeof(string))
            {
                var v = f.GetValue(null) as string;
                if (!string.IsNullOrEmpty(v)) return v;
            }
        }
        string[] propNames = { "SelectedSceneName", "SceneToLoad", "TargetScene", "DetailScene", "SelectedLevel", "SelectedScene" };
        foreach (var pn in propNames)
        {
            var p = t.GetProperty(pn);
            if (p != null && p.PropertyType == typeof(string))
            {
                var v = p.GetValue(null) as string;
                if (!string.IsNullOrEmpty(v)) return v;
            }
        }
        return null;
    }
}
