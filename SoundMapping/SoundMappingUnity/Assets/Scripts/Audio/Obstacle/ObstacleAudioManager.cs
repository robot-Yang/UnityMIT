// Assets/Scripts/Audio/Obstacle/ObstacleAudioManager.cs
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class ObstacleAudioManager : MonoBehaviour
{
    [Header("Scheduling")]
    [Tooltip("Update rate for audio parameter refresh, in Hz.")]
    [Range(5f, 90f)] public float updateRate = 30f;

    [Tooltip("Max obstacles updated per frame. 0 = unlimited.")]
    public int perFrameBudget = 0;

    [Header("Hard-coded thresholds (edit values in code or here)")]
    [Tooltip("Obstacles of size >= this are treated as 'large' in the hard-coded logic.")]
    public float largeSizeThreshold = 2.0f;

    [Header("Profiles")]
    [Tooltip("Drag in all available profiles manually")]
    public List<ObstacleAudioProfileBase> profiles;

    [Header("Mode Switching")]
    [Tooltip("Below this distance, switch from Beep to Continuous (car-like solid tone).")]
    [Min(0f)] public float continuousSwitchDistance = 1.5f;

    [Tooltip("Hysteresis band to avoid rapid toggling (meters). " +
             "Profile switches to Continuous at (threshold - hysteresis), " +
             "and back to Beep at (threshold + hysteresis).")]
    [Min(0f)] public float switchThreshold = 0.2f;

    [Tooltip("Smooth follow speed of the listener toward the swarm center.")]
    public float listenerFollowSpeed = 20f;

    // ===== Runtime containers =====
    private Transform listenerTransform;
    private readonly List<ObstacleAudio> _obstacles = new List<ObstacleAudio>();
    private readonly Dictionary<ObstacleAudio, Runtime> _rt = new Dictionary<ObstacleAudio, Runtime>();

    // Profiles loaded, addressed by asset name (case-insensitive)
    private Dictionary<string, ObstacleAudioProfileBase> _profilesByName;

    private float step_count;
    public static ObstacleAudioManager Instance { get; private set; }

    // Profiles
    private const string PROFILE_BEEP = "BeepAudioProfile";
    private const string PROFILE_CONT = "ContinuousAudioProfile";

    private Transform closestDrone;


    private class Runtime
    {
        public ObstacleAudio obstacle;
        public ObstacleAudioProfileBase profile;
        public bool isLoopingStarted;
        public float pulseTimer; // used only for Beep profiles
    }

    private struct AudioCtx
    {
        public float dt;
        public float distance;
        public float size;
    }

    // ===== Unity lifecycle =====
    private void Awake()
    {
        if (Instance != null && Instance != this) { Destroy(gameObject); return; }
        Instance = this;
    }

    private void Start()
    {
        // Automatically find the AudioListener among this GameObject's children
        if (listenerTransform == null)
        {
            var listener = GetComponentInChildren<AudioListener>();
            if (listener != null)
                listenerTransform = listener.transform;
            else
                Debug.LogWarning("[ObstacleAudioManager] No AudioListener found among children!");
        }

        _profilesByName = new Dictionary<string, ObstacleAudioProfileBase>(System.StringComparer.OrdinalIgnoreCase);

        if (profiles != null)
        {
            foreach (var prof in profiles)
            {
                if (prof != null && !_profilesByName.ContainsKey(prof.name))
                    _profilesByName.Add(prof.name, prof);
            }
        }

        // Auto-register obstacles etc.
        foreach (var o in FindObjectsOfType<ObstacleAudio>())
        {
            print("Found obstacle");
            Register(o);
            print("End of register");
        }

        WarnIfMissing(PROFILE_BEEP);
        WarnIfMissing(PROFILE_CONT);
    }

    private void Update()
    {
        if (listenerTransform == null || _obstacles.Count == 0) return;

        // Listener follows the center of the swarm
        List<DroneFake> drones = swarmModel.dronesInMainNetwork;
        if (drones != null && drones.Count > 0)
        {
            Vector3 center = Vector3.zero;
            foreach (DroneFake drone in drones)
                center += drone.position;
            center /= drones.Count;

            listenerTransform.position = Vector3.Lerp(
                listenerTransform.position,
                center,
                Time.deltaTime * listenerFollowSpeed
            );
        }

        step_count += Time.deltaTime;
        var step = 1f / Mathf.Max(1f, updateRate);
        if (step_count < step) return;
        float dt = step_count;   // accumulated dt for timer stability
        step_count = 0f;

        int processed = 0;
        for (int i = 0; i < _obstacles.Count; i++)
        {
            var o = _obstacles[i];
            if (o == null) continue;

            if (perFrameBudget > 0 && processed >= perFrameBudget) break;
            processed++;

            if (!_rt.TryGetValue(o, out var r))
                continue;

            float dist = ComputeDistanceToObstacle(o.transform);

            // Check switch conditions
            if (r.profile is BeepAudioProfile && dist <= (continuousSwitchDistance - switchThreshold))
            {
                if (TryGetProfile(PROFILE_CONT, out var cont))
                {
                    // Stop any currently playing continuous loop
                    if (o.source != null && o.source.isPlaying)
                        o.source.Stop();

                    r.profile = cont;
                    r.isLoopingStarted = false;
                    print($"Switched to CONT (dist={dist:F2})");
                }
            }

            else if (r.profile is ContinuousAudioProfile && dist >= (continuousSwitchDistance + switchThreshold))
            {
                if (TryGetProfile(PROFILE_BEEP, out var beep))
                {
                    // Stop any currently playing continuous loop
                    if (o.source != null && o.source.isPlaying)
                        o.source.Stop();

                    r.profile = beep;
                    r.isLoopingStarted = false;
                    print($"Switched to BEEP (dist={dist:F2})");
                }
            }

            var ctx = new AudioCtx
            {
                dt = dt,
                distance = dist,
                size = o.sizeScalar
            };

            ApplyProfile(r, ctx);
        }
    }

    // ===== Algorithms =====
    private int _lastTintedDroneID = -1;

    private static void SetDroneTint(Transform drone, Color c)
    {
        if (drone == null) return;

        // IMPORTANT: r.material instantiates a copy so we don’t overwrite the shared material
        foreach (Renderer r in drone.GetComponentsInChildren<Renderer>())
        {
            r.material.color = c;
        }
    }

    // ------------------------------------------------------------
    // ComputeDistanceToObstacle
    // 1) Compute swarm centroid C (using HapticsTest’s centroid util)
    // 2) Find closest point P on the obstacle's collider to C
    // 3) Express P and each drone position relative to C
    // 4) Find the drone with the highest dot((d - C), (P - C))
    // 5) Tint that drone red using SetDroneTint
    // 6) Return distance |P - d_closest|
    // ------------------------------------------------------------
    private Vector3 _lastPointP = Vector3.zero;
    private bool _hasPointP = false;
    private Vector3 _lastPointC = Vector3.zero;
    private bool _hasPointC = false;


    private float ComputeDistanceToObstacle(Transform obstacle)
    {
        // Build the same drone list pattern used in HapticsTest
        var drones = FindObjectsOfType<DroneController>()
                     .Select(d => d.transform)
                     .ToList();

        if (drones == null || drones.Count == 0)
        {
            // No drones? Fallback to listener → obstacle distance like before.
            return Vector3.Distance(listenerTransform.position, obstacle.position);
        }

        // Swarm centroid C (uses HapticsTest public static function as in your file)
        Vector3 C = HapticsTest.GetSwarmCentroid(drones);
        _lastPointC = C;
        _hasPointC = true;

        // Closest point P on obstacle collider to C
        Vector3 P;
        var col = obstacle.GetComponent<Collider>() ?? obstacle.GetComponentInParent<Collider>();
        if (col != null)
        {
            P = col.ClosestPoint(C);  // Unity’s projection onto collider surface
        }
        else
        {
            // If no collider -> obstacle’s position
            P = obstacle.position;
        }
        _lastPointP = P;
        _hasPointP = true;

        // Work in swarm-relative space
        Vector3 Pr = P - C;

        Transform bestDrone = null;
        float bestDot = float.NegativeInfinity;

        foreach (var d in drones)
        {
            Vector3 Dr = d.position - C;

            // Compute the dot product between Dr and Pr
            float dot = Vector3.Dot(Dr, Pr);

            if (dot > bestDot)
            {
                bestDot = dot;
                bestDrone = d;
            }
        }

        if (bestDrone == null)
        {
            // Shouldn’t happen if we have drones, but just in case
            return Vector3.Distance(listenerTransform.position, obstacle.position);
        }

        DroneController ctrl = bestDrone.GetComponent<DroneController>();
        if (ctrl == null) return Vector3.Distance(listenerTransform.position, obstacle.position);

        int currentID = ctrl.droneFake.id;


        // Return the distance from that closest (by direction) drone to P
        float distance = Vector3.Distance(bestDrone.position, P);

        // Tint the chosen drone RED, using the exact SetDroneTint behavior
        if (currentID != _lastTintedDroneID && (distance<6f))
        {
            // Reset previous drone’s tint
            if (_lastTintedDroneID != -1)
            {
                var prev = FindObjectsOfType<DroneController>()
                           .FirstOrDefault(d => d.droneFake.id == _lastTintedDroneID);
                if (prev != null)
                    SetDroneTint(prev.transform, Color.white);
            }

            // Tint the new closest drone
            SetDroneTint(bestDrone, Color.red);

            // Update cache
            _lastTintedDroneID = currentID;
        }
        if (distance<5f) print("Closest distance to obstacle: "+distance);
        return distance;
    }


    // ===== Public API =====
    public void Register(ObstacleAudio obstacle)
    {
        if (!_obstacles.Contains(obstacle))
        {
            _obstacles.Add(obstacle);
            _rt[obstacle] = new Runtime { obstacle = obstacle, profile = null, isLoopingStarted = false, pulseTimer = 0f };
            AssignProfile(obstacle);
        }
    }

    public void Unregister(ObstacleAudio obstacle)
    {
        _obstacles.Remove(obstacle);
        _rt.Remove(obstacle);
    }

#if UNITY_EDITOR
    public ObstacleAudioProfileBase GetAssignedProfileFor(ObstacleAudio obstacle)
    {
        if (obstacle != null && _rt.TryGetValue(obstacle, out var r)) return r.profile;
        return null;
    }
#endif

    // ===== Internals =====
    private void AssignProfile(ObstacleAudio obstacle)
    {
        var chosen = DecideProfileFor(obstacle);
        if (chosen == null)
        {
            Debug.LogWarning($"[ObstacleAudioManager] No profile could be assigned to '{obstacle.name}'. Check the 'profiles' list on the manager or rename/create profiles '{PROFILE_BEEP}' / '{PROFILE_CONT}'.");
        }

        var r = _rt[obstacle];
        if (r.profile != chosen)
        {
            if (obstacle.source != null && obstacle.source.isPlaying)
                obstacle.source.Stop();

            r.profile = chosen;
            r.isLoopingStarted = false;
            r.pulseTimer = 0f;
            _rt[obstacle] = r;
        }
    }

    private ObstacleAudioProfileBase DecideProfileFor(ObstacleAudio obstacle)
    {
        if (TryGetProfile(PROFILE_BEEP, out var beep))
        {
            print("Assigned beep");
            return beep;
        }
        print("Assign condition does not work");
        return null;
    }


    private bool TryGetProfile(string name, out ObstacleAudioProfileBase profile)
    {
        if (_profilesByName != null && _profilesByName.TryGetValue(name, out profile) && profile != null)
            return true;
        profile = null;
        return false;
    }

    private void WarnIfMissing(string name)
    {
        if (_profilesByName == null || !_profilesByName.ContainsKey(name))
            Debug.LogWarning($"[ObstacleAudioManager] Expected profile '{name}' not found in the manual list. Add a ScriptableObject named '{name}' to the 'profiles' list on the manager.");
    }

    private void ApplyProfile(Runtime r, AudioCtx ctx)
    {
        var o = r.obstacle;
        var p = r.profile;
        if (o == null || p == null || o.source == null) return;

        // Audible gate (computed once)
        bool audible = ctx.distance <= p.maxAudibleDistance;

        // Curves → final params (computed once)
        float volMul = Mathf.Clamp01(p.volumeByDistance.Evaluate(ctx.distance));
        float pitchDistMul = Mathf.Max(0.01f, p.pitchByDistance.Evaluate(ctx.distance));
        float pitchSizeMul = Mathf.Max(0.01f, p.pitchBySize.Evaluate(ctx.size));
        float finalVolume = p.baseVolume * volMul;
        float finalPitch = p.basePitch * pitchDistMul * pitchSizeMul;

        if (p is ContinuousAudioProfile cProf)
        {
            // Continuous: simple gate
            o.source.mute = !audible;
            if (!audible) return;

            if (!r.isLoopingStarted)
            {
                o.source.loop = true;
                o.source.clip = cProf.loopClip;
                if (o.source.clip != null) o.source.Play();
                r.isLoopingStarted = true;
                _rt[o] = r;
            }

            o.source.volume = finalVolume;
            o.source.pitch = finalPitch;
        }
        else if (p is BeepAudioProfile bProf)
        {
            // One-shot mode for beeps
            if (o.source.loop) o.source.loop = false;

            // Don’t mute while a beep is currently playing; let it finish naturally
            o.source.mute = !audible && !o.source.isPlaying;
            if (!audible && !o.source.isPlaying) return;

            // Apply pitch to the source; pass volume only in PlayOneShot to avoid double attenuation
            o.source.pitch = finalPitch;
            o.source.volume = 1f;

            float rateHz = bProf.pulseRateByDistance.Evaluate(ctx.distance);
            rateHz = Mathf.Clamp(rateHz, bProf.pulseRateClamp.x, bProf.pulseRateClamp.y);
            float interval = rateHz > 0f ? 1f / rateHz : 0.5f;

            r.pulseTimer += ctx.dt;
            if (r.pulseTimer >= interval)
            {
                r.pulseTimer = 0f;
                if (bProf.beepClip != null)
                    o.source.PlayOneShot(bProf.beepClip, finalVolume); // single volume application
            }
            _rt[o] = r;
        }
    }
    void OnDrawGizmos()
    {
        if (_hasPointP)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(_lastPointP, 0.1f);
        }
        if (_hasPointC)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(_lastPointC, 0.1f);
        }
    }

}
