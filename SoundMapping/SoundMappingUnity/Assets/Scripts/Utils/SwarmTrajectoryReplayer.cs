// SwarmTrajectoryReplayer.cs
// Replays recorded swarm trajectories by updating DroneFake positions (and optionally transforms),
// and recomputes swarmModel.network/score for downstream haptics.
// Add-only component: no existing code changes required.

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;

[DefaultExecutionOrder(-1000)]
public class SwarmTrajectoryReplayer : MonoBehaviour
{
    [Header("Source")]
    [Tooltip("Absolute or relative path to the JSON file produced by SwarmTrajectoryRecorder.")]
    public string jsonFilePath = "";
    [Tooltip("Optional TextAsset; if set, this overrides jsonFilePath.")]
    public TextAsset jsonFileAsset;
    [Tooltip("Load the file automatically in Start().")]
    public bool autoLoadOnStart = true;

    [Header("Playback")]
    public bool playOnLoad = false;
    public bool loop = false;
    public float playbackSpeed = 1f;
    [Tooltip("Advance frames using recorded frame times (t).")]
    public bool useRecordedTimes = true;

    [Header("Bindings")]
    public bool updateDroneFakePositions = true;
    public bool updateTransformPositions = true;
    public bool updateTransformRotations = true;
    [Tooltip("If log contains embodiedId, set CameraMovement.embodiedDrone on load.")]
    public bool setEmbodiedOnLoad = true;

    [Header("SwarmModel Integration")]
    [Tooltip("Recompute swarmModel.network and swarmConnectionScore each frame.")]
    public bool updateSwarmNetwork = true;
    [Tooltip("Disable the swarmModel component during replay to avoid overwriting positions.")]
    public bool disableSwarmModelDuringReplay = true;

    [Header("Debug")]
    public bool verboseLogs = false;

    private TrajectoryLog _log;
    private Dictionary<int, DroneController> _byId = new Dictionary<int, DroneController>();
    private Dictionary<string, DroneController> _byName = new Dictionary<string, DroneController>();
    private List<float> _frameTimes = new List<float>();
    private int _currentFrame = 0;
    private float _playbackTime = 0f;
    private bool _loaded = false;
    private bool _playing = false;

    private swarmModel _swarmModel;
    private bool _swarmModelWasEnabled;

    // -------------------- Public Controls --------------------

    public bool Load()
    {
        string json = ReadJsonText();
        if (string.IsNullOrEmpty(json))
        {
            Debug.LogError("[SwarmTrajectoryReplayer] No JSON found. Check jsonFilePath/TextAsset.");
            _loaded = false;
            return false;
        }

        try
        {
            _log = JsonUtility.FromJson<TrajectoryLog>(json);
        }
        catch (Exception e)
        {
            Debug.LogError($"[SwarmTrajectoryReplayer] Failed to parse JSON: {e.Message}");
            _loaded = false;
            return false;
        }

        if (_log == null || _log.trajectories == null || _log.trajectories.Count == 0)
        {
            Debug.LogError("[SwarmTrajectoryReplayer] JSON parsed but no trajectories found.");
            _loaded = false;
            return false;
        }

        BuildDroneMaps();
        BuildFrameTimes();
        _currentFrame = 0;
        _playbackTime = (_frameTimes.Count > 0) ? _frameTimes[0] : 0f;
        _loaded = true;

        if (setEmbodiedOnLoad)
        {
            TrySetEmbodiedFromLog();
        }

        if (verboseLogs)
        {
            Debug.Log($"[SwarmTrajectoryReplayer] Loaded {_log.trajectories.Count} trajectories, frames: {_frameTimes.Count}.");
        }

        if (playOnLoad)
        {
            Play();
        }

        return true;
    }

    public void Play()
    {
        if (!_loaded)
        {
            if (!Load()) return;
        }

        if (disableSwarmModelDuringReplay)
        {
            DisableSwarmModel();
        }

        if (_frameTimes.Count > 0 && _playbackTime < _frameTimes[0])
        {
            _playbackTime = _frameTimes[0];
        }

        _playing = true;
    }

    public void Pause()
    {
        _playing = false;
    }

    public void Stop()
    {
        _playing = false;
        _currentFrame = 0;
        _playbackTime = (_frameTimes.Count > 0) ? _frameTimes[0] : 0f;
        RestoreSwarmModel();
        ApplyFrame(_currentFrame);
    }

    public void StepFrame()
    {
        if (!_loaded)
        {
            if (!Load()) return;
        }

        int next = Mathf.Min(_currentFrame + 1, _frameTimes.Count - 1);
        _currentFrame = next;
        _playbackTime = _frameTimes[_currentFrame];
        ApplyFrame(_currentFrame);
        if (updateSwarmNetwork)
        {
            UpdateSwarmNetworkFromReplay();
        }
    }

    public bool IsLoaded()
    {
        return _loaded;
    }

    public int GetFrameCount()
    {
        return _frameTimes.Count;
    }

    public float GetProgress01()
    {
        if (_frameTimes.Count == 0) return 0f;
        float start = _frameTimes[0];
        float end = _frameTimes[_frameTimes.Count - 1];
        if (Mathf.Approximately(start, end)) return 0f;
        return Mathf.InverseLerp(start, end, _playbackTime);
    }

    public float GetTimeSeconds()
    {
        if (_frameTimes.Count == 0) return 0f;
        float start = _frameTimes[0];
        return Mathf.Max(0f, _playbackTime - start);
    }

    public float GetDurationSeconds()
    {
        if (_frameTimes.Count == 0) return 0f;
        float start = _frameTimes[0];
        float end = _frameTimes[_frameTimes.Count - 1];
        return Mathf.Max(0f, end - start);
    }

    public void SetProgress01(float progress01)
    {
        if (!_loaded)
        {
            if (!Load()) return;
        }

        if (_frameTimes.Count == 0) return;

        float p = Mathf.Clamp01(progress01);
        float start = _frameTimes[0];
        float end = _frameTimes[_frameTimes.Count - 1];
        _playbackTime = Mathf.Lerp(start, end, p);

        if (useRecordedTimes)
        {
            int idx = 0;
            for (int i = 0; i < _frameTimes.Count; i++)
            {
                if (_frameTimes[i] <= _playbackTime)
                {
                    idx = i;
                }
                else
                {
                    break;
                }
            }
            _currentFrame = idx;
        }
        else
        {
            _currentFrame = Mathf.RoundToInt(p * (_frameTimes.Count - 1));
        }

        ApplyFrame(_currentFrame);
        if (updateSwarmNetwork)
        {
            UpdateSwarmNetworkFromReplay();
        }
    }

    public void SetTimeSeconds(float seconds)
    {
        if (!_loaded)
        {
            if (!Load()) return;
        }

        if (_frameTimes.Count == 0) return;

        float start = _frameTimes[0];
        float end = _frameTimes[_frameTimes.Count - 1];
        float target = Mathf.Clamp(start + Mathf.Max(0f, seconds), start, end);
        _playbackTime = target;

        if (useRecordedTimes)
        {
            int idx = 0;
            for (int i = 0; i < _frameTimes.Count; i++)
            {
                if (_frameTimes[i] <= _playbackTime)
                {
                    idx = i;
                }
                else
                {
                    break;
                }
            }
            _currentFrame = idx;
        }
        else
        {
            float duration = Mathf.Max(0f, end - start);
            float p = duration > 0f ? ((_playbackTime - start) / duration) : 0f;
            _currentFrame = Mathf.RoundToInt(Mathf.Clamp01(p) * (_frameTimes.Count - 1));
        }

        ApplyFrame(_currentFrame);
        if (updateSwarmNetwork)
        {
            UpdateSwarmNetworkFromReplay();
        }
    }

    public bool IsPlaying()
    {
        return _playing;
    }

    // -------------------- Unity Lifecycle --------------------

    private void Start()
    {
        if (autoLoadOnStart)
        {
            Load();
        }
    }

    private void OnDisable()
    {
        RestoreSwarmModel();
    }

    private void Update()
    {
        if (!_playing || !_loaded) return;

        AdvancePlaybackTime(Time.unscaledDeltaTime * Mathf.Max(0f, playbackSpeed));
        ApplyFrame(_currentFrame);

        if (updateSwarmNetwork)
        {
            UpdateSwarmNetworkFromReplay();
        }
    }

    // -------------------- Core Replay --------------------

    private void AdvancePlaybackTime(float delta)
    {
        if (_frameTimes.Count == 0) return;

        _playbackTime += delta;

        if (useRecordedTimes)
        {
            while (_currentFrame + 1 < _frameTimes.Count && _playbackTime >= _frameTimes[_currentFrame + 1])
            {
                _currentFrame++;
            }
        }
        else
        {
            int next = _currentFrame + 1;
            if (next < _frameTimes.Count)
            {
                _currentFrame = next;
            }
        }

        if (_currentFrame >= _frameTimes.Count - 1)
        {
            if (loop)
            {
                _currentFrame = 0;
                _playbackTime = _frameTimes[0];
            }
            else
            {
                _playing = false;
            }
        }
    }

    private void ApplyFrame(int frameIndex)
    {
        if (_log == null || _log.trajectories == null) return;

        foreach (var traj in _log.trajectories)
        {
            if (traj == null || traj.frames == null || traj.frames.Count == 0) continue;
            int idx = Mathf.Clamp(frameIndex, 0, traj.frames.Count - 1);
            TrajFrame f = traj.frames[idx];

            DroneController dc = ResolveDrone(traj);
            if (dc == null) continue;

            Vector3 pos = new Vector3(f.x, f.y, f.z);
            if (updateDroneFakePositions && dc.droneFake != null)
            {
                dc.droneFake.position = pos;
                dc.droneFake.velocity = Vector3.zero;
                dc.droneFake.acceleration = Vector3.zero;
            }

            if (updateTransformPositions)
            {
                dc.transform.position = pos;
            }

            if (updateTransformRotations)
            {
                dc.transform.rotation = new Quaternion(f.qx, f.qy, f.qz, f.qw);
            }
        }
    }

    private void UpdateSwarmNetworkFromReplay()
    {
        List<DroneFake> drones = new List<DroneFake>();
        foreach (var kvp in _byId)
        {
            if (kvp.Value != null && kvp.Value.droneFake != null)
            {
                drones.Add(kvp.Value.droneFake);
            }
        }

        if (drones.Count == 0) return;

        var net = new NetworkCreator(drones);
        net.refreshNetwork();
        swarmModel.network = net;

        // Mirror swarmModel.getSwarmConnexion() logic
        List<DroneFake> connected = net.drones.ToList();
        bool hasNonMovable = drones.Exists(d => !d.isMovable);
        if (hasNonMovable && net.largestComponent != null && net.largestComponent.Count > 0)
        {
            connected = net.largestComponent.ToList();
        }

        var net2 = new NetworkCreator(connected);
        net2.refreshNetwork();

        float avgDist;
        float energyDev = net2.ComputeNormalizedDeviationEnergy(out avgDist);
        swarmModel.swarmConnectionScore = energyDev;
        swarmModel.avgDist = avgDist;
    }

    // -------------------- Helpers --------------------

    private void BuildDroneMaps()
    {
        _byId.Clear();
        _byName.Clear();

        var controllers = FindObjectsOfType<DroneController>();
        foreach (var dc in controllers)
        {
            if (dc == null) continue;
            if (dc.droneFake != null)
            {
                if (!_byId.ContainsKey(dc.droneFake.id))
                {
                    _byId.Add(dc.droneFake.id, dc);
                }
                else if (verboseLogs)
                {
                    Debug.LogWarning($"[SwarmTrajectoryReplayer] Duplicate droneFake.id: {dc.droneFake.id} (using first)." );
                }
            }

            string key = dc.gameObject.name.Trim().ToLowerInvariant();
            if (!_byName.ContainsKey(key))
            {
                _byName.Add(key, dc);
            }
        }
    }

    private void BuildFrameTimes()
    {
        _frameTimes.Clear();
        var first = _log.trajectories.FirstOrDefault(t => t != null && t.frames != null && t.frames.Count > 0);
        if (first == null) return;

        foreach (var f in first.frames)
        {
            _frameTimes.Add(f.t);
        }

        if (_frameTimes.Count == 0)
        {
            _frameTimes.Add(0f);
        }
    }

    private DroneController ResolveDrone(DroneTraj traj)
    {
        if (traj == null) return null;

        if (_byId.TryGetValue(traj.id, out var dcById))
        {
            return dcById;
        }

        if (!string.IsNullOrEmpty(traj.name))
        {
            string key = traj.name.Trim().ToLowerInvariant();
            if (_byName.TryGetValue(key, out var dcByName))
            {
                return dcByName;
            }
        }

        if (verboseLogs)
        {
            Debug.LogWarning($"[SwarmTrajectoryReplayer] No matching drone for traj id={traj.id}, name={traj.name}");
        }
        return null;
    }

    private string ReadJsonText()
    {
        if (jsonFileAsset != null)
        {
            return jsonFileAsset.text;
        }

        if (string.IsNullOrWhiteSpace(jsonFilePath))
        {
            return null;
        }

        string resolved = ResolvePath(jsonFilePath);
        if (!File.Exists(resolved))
        {
            if (verboseLogs)
            {
                Debug.LogWarning($"[SwarmTrajectoryReplayer] File not found: {resolved}");
            }
            return null;
        }

        return File.ReadAllText(resolved);
    }

    private string ResolvePath(string path)
    {
        if (string.IsNullOrWhiteSpace(path)) return path;
        if (Path.IsPathRooted(path)) return path;

        string inAssets = Path.Combine(Application.dataPath, path);
        if (File.Exists(inAssets)) return inAssets;

        string inPersistent = Path.Combine(Application.persistentDataPath, path);
        if (File.Exists(inPersistent)) return inPersistent;

        return path;
    }

    private void DisableSwarmModel()
    {
        if (_swarmModel == null)
        {
            _swarmModel = FindObjectOfType<swarmModel>();
        }

        if (_swarmModel != null)
        {
            _swarmModelWasEnabled = _swarmModel.enabled;
            _swarmModel.enabled = false;
        }
    }

    private void RestoreSwarmModel()
    {
        if (_swarmModel != null)
        {
            _swarmModel.enabled = _swarmModelWasEnabled;
        }
    }

    private void TrySetEmbodiedFromLog()
    {
        if (_log == null) return;

        if (_log.embodiedId != int.MinValue && _byId.TryGetValue(_log.embodiedId, out var dc))
        {
            CameraMovement.embodiedDrone = dc.gameObject;
            return;
        }

        if (!string.IsNullOrEmpty(_log.embodiedName))
        {
            string key = _log.embodiedName.Trim().ToLowerInvariant();
            if (_byName.TryGetValue(key, out var dcByName))
            {
                CameraMovement.embodiedDrone = dcByName.gameObject;
            }
        }
    }

    // -------------------- Serializable data mirrors --------------------

    [Serializable]
    public class TrajectoryLog
    {
        public string scene;
        public string pid;
        public string haptics;
        public string order;
        public float sampleHz;
        public List<DroneTraj> trajectories = new List<DroneTraj>();
        public List<TrialWindow> trials = new List<TrialWindow>();
        public int embodiedId;
        public string embodiedName;
        public long utcStartMs;
        public long utcEndMs;
    }

    [Serializable]
    public class DroneTraj
    {
        public int id;
        public string name;
        public List<TrajFrame> frames = new List<TrajFrame>();
    }

    [Serializable]
    public struct TrajFrame
    {
        public float t;
        public float x, y, z;
        public byte g;
        public byte e;
        public float qx, qy, qz, qw;
        public long utcMs;
    }

    [Serializable]
    public class TrialWindow
    {
        public string label;
        public float startGameTime;
        public float startRealtime;
        public float endGameTime;
        public float endRealtime;
    }
}
