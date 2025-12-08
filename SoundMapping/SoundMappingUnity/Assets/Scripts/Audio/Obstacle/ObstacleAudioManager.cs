// Assets/Scripts/Audio/Obstacle/ObstacleAudioManager.cs
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

#if UNITY_EDITOR
using UnityEditor;
#endif


public class ObstacleAudioManager : MonoBehaviour
{
    [Header("Scheduling")]
    [Tooltip("Update rate for audio parameter refresh, in Hz.")]
    [Range(1f, 100f)] public float updateRate = 30f;

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
    [Min(0f)] public float continuousSwitchDistance = 2.3f;

    [Tooltip("Hysteresis band to avoid rapid toggling (meters). " +
             "Profile switches to Continuous at (threshold - hysteresis), " +
             "and back to Beep at (threshold + hysteresis).")]
    [Min(0f)] public float switchThreshold = 0.2f;

    [Tooltip("Smooth follow speed of the listener toward the swarm center.")]
    public float listenerFollowSpeed = 20f;

    [SerializeField] private MigrationPointController mpc;

    [Header("Directional Attenuation")]
    public bool useDirectionalAttenuation = true;

    [Range(0f, 1f)] public float minDirectionalVolume = 0f;  // mul = 0 → this
    [Range(0f, 1f)] public float maxDirectionalVolume = 1f;  // mul = 1 → this

    [Min(0.01f)] public float facingExponent = 1.0f;         // optional shaping of mul in [0,1]

    [Header("Debug Highlight")]
    [Min(0f)] public float minHighlightDistance = 6f;   // "minimalrange"

    [Header("Swarm Envelope")]
    [Tooltip("Base length used before scaling by swarm speed.")]
    public float baseEnvelopeLength = 1f;

    [Tooltip("Minimal length of the swarm envelope, even if speed is zero.")]
    public float minEnvelopeLength = 30f;

    [Header("Look Envelope")]
    [Tooltip("Fixed length for the look-direction rectangle.")]
    public float lookEnvelopeLength = 30f;

    [Header("Velocity Envelope Tuning")]
    [Tooltip("If speed is below this, keep the last velocity direction (freeze).")]
    public float velocityDirectionThreshold = 1f;   // adjust as needed

    [Header("Safety Envelope C")]
    [Tooltip("Scale")]
    public float safety_radius_scale = 1.0f;

    [Tooltip("Offset")]
    public float safety_radius_offset = 2f;

    [Header("Width Safety")]
    public float envelopeWidthSafety = 2f;   // in meters





    // ===== Runtime containers =====
    private Transform listenerTransform;
    private readonly List<ObstacleAudio> _obstacles = new List<ObstacleAudio>();
    private readonly Dictionary<ObstacleAudio, Runtime> _rt = new Dictionary<ObstacleAudio, Runtime>();

    // Cached swarm drones (for this manager's computations)
    private readonly List<Transform> _cachedDrones = new List<Transform>();

    // Profiles loaded, addressed by asset name (case-insensitive)
    private Dictionary<string, ObstacleAudioProfileBase> _profilesByName;

    private float step_count;
    public static ObstacleAudioManager Instance { get; private set; }

    // Profiles
    private const string PROFILE_BEEP = "BeepAudioProfile";
    private const string PROFILE_CONT = "ContinuousAudioProfile";

    private Transform closestDrone;
    private List<Vector3> allPointsP = new List<Vector3>();


    private class Runtime
    {
        public ObstacleAudio obstacle;
        public ObstacleAudioProfileBase profile;
        public bool isLoopingStarted;
        public float pulseTimer; // used only for Beep profiles
        public Transform closestDrone;
        public float closestDistance = -1;
        public Vector3 dirDroneToP_XZ = Vector3.zero; // normalized, XZ
    }

    private struct AudioCtx
    {
        public float dt;
        public float distance;
        public float size;
    }

    // ===== Swarm envelope state =====
    private Vector3 _envelopeCenterXZ;
    private Vector3 _swarmForwardXZ = Vector3.forward;
    private Vector3 _swarmRightXZ = Vector3.right;
    private float _envelopeRadius = 0f;
    private float _envelopeLength = 0f;
    private bool _hasEnvelope = false;

    private Vector3 _lastCentroidXZ;
    private bool _hasLastCentroid = false;

    // Look-direction envelope
    private Vector3 _lookForwardXZ = Vector3.forward;
    private Vector3 _lookRightXZ = Vector3.right;


    // ===== Gizmo helpers (centroid C and last point P) =====
    private Vector3 _lastPointP = Vector3.zero;
    private bool _hasPointP = false;
    private Vector3 _lastPointC = Vector3.zero;
    private bool _hasPointC = false;

    // ===== Unity lifecycle =====
    private void Awake()
    {
        if (Instance != null && Instance != this) { Destroy(gameObject); return; }
        Instance = this;

        mpc = GetComponent<MigrationPointController>();
        if (mpc == null)
            Debug.LogError("[ObstacleAudioManager] MigrationPointController not found on the same GameObject.");
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

        // Listener follows the center of the swarm (using your existing swarm model)
        List<DroneFake> dronesForListener = swarmModel.dronesInMainNetwork;
        if (dronesForListener != null && dronesForListener.Count > 0)
        {
            Vector3 center = Vector3.zero;
            foreach (DroneFake drone in dronesForListener)
                center += drone.position;
            center /= dronesForListener.Count;

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

        // === Swarm envelope update (based on DroneController transforms) ===
        _cachedDrones.Clear();
        var droneControllers = FindObjectsOfType<DroneController>();
        foreach (var ctrl in droneControllers)
            _cachedDrones.Add(ctrl.transform);

        UpdateSwarmEnvelope(_cachedDrones, dt);

        allPointsP.Clear();

        int processed = 0;
        for (int i = 0; i < _obstacles.Count; i++)
        {
            var o = _obstacles[i];
            if (o == null) continue;

            if (perFrameBudget > 0 && processed >= perFrameBudget) break;
            processed++;

            if (!_rt.TryGetValue(o, out var r))
                continue;

            float dist = ComputeDistanceToObstacle(o);

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

            if (r.closestDrone != null && r.closestDistance >= 0f)
            {
                var ctrl = r.closestDrone.GetComponent<DroneController>();
                if (ctrl != null)
                {
                    ctrl.audioHighlight = (r.closestDistance <= minHighlightDistance);
                }
            }
        }
    }

    // ===== Swarm envelope computation =====
    private void UpdateSwarmEnvelope(List<Transform> drones, float dt)
    {
        if (drones == null || drones.Count == 0)
        {
            _hasEnvelope = false;
            return;
        }

        // Swarm centroid in XZ
        Vector3 centroid = HapticsTest.GetSwarmCentroid(drones);
        centroid.y = 0f;

        // Velocity from centroid motion
        Vector3 vel = Vector3.zero;
        if (_hasLastCentroid && dt > 0f)
            vel = (centroid - _lastCentroidXZ) / dt;

        vel.y = 0f;
        float speed = vel.magnitude;

        // Direction: keep last direction when speed is near zero
        // --- Velocity-direction freeze threshold ---
        if (speed >= velocityDirectionThreshold)
        {
            // valid, stable movement → update direction
            _swarmForwardXZ = vel.normalized;
        }
        else
        {
            // below threshold → keep last direction (freeze)
            // except the very first frame
            if (!_hasEnvelope)
                _swarmForwardXZ = Vector3.forward;
        }


        _swarmForwardXZ.y = 0f;
        if (_swarmForwardXZ.sqrMagnitude < 1e-4f)
            _swarmForwardXZ = Vector3.forward;
        _swarmForwardXZ.Normalize();

        // Perpendicular axis (right); left is simply -_swarmRightXZ
        _swarmRightXZ = new Vector3(_swarmForwardXZ.z, 0f, -_swarmForwardXZ.x);

        // Width: distance between furthest drones along the side axis
        float minSide = float.PositiveInfinity;
        float maxSide = float.NegativeInfinity;
        foreach (var d in drones)
        {
            Vector3 local = d.position;
            local.y = 0f;
            local -= centroid;
            float side = Vector3.Dot(local, _swarmRightXZ);
            if (side < minSide) minSide = side;
            if (side > maxSide) maxSide = side;
        }
        _envelopeRadius = (Mathf.Max(0.1f, maxSide - minSide)) * 0.5f + envelopeWidthSafety;

        // Length: baseLength scaled by speed, with minimal length so it never collapses
        float rawLength = baseEnvelopeLength * speed;
        _envelopeLength = Mathf.Max(minEnvelopeLength, rawLength);

        _envelopeCenterXZ = centroid;
        _hasEnvelope = true;

        _lastCentroidXZ = centroid;
        _hasLastCentroid = true;

        // For your previous gizmo usage
        _lastPointC = centroid;
        _hasPointC = true;

        // --- Look-direction envelope axes ---
        // Direction = operator look direction from MigrationPointController
        Vector3 lookDir = mpc.GetSwarmHeading();
        lookDir.y = 0f;

        if (lookDir.sqrMagnitude < 1e-4f)
            lookDir = _swarmForwardXZ;  // fallback if mpc returns zero

        _lookForwardXZ = lookDir.normalized;
        _lookRightXZ = new Vector3(_lookForwardXZ.z, 0f, -_lookForwardXZ.x);

    }

    // Check if a world-space point lies inside the oriented swarm envelope (XZ)
    private bool IsPointInsideEnvelope(Vector3 point)
    {
        if (!_hasEnvelope)
            return true; // Fallback: if we have no envelope yet, do not gate anything

        Vector3 flatPoint = point;
        flatPoint.y = 0f;

        Vector3 flatCenter = _envelopeCenterXZ;
        flatCenter.y = 0f;

        Vector3 local = flatPoint - flatCenter;

        float forwardCoord = Vector3.Dot(local, _swarmForwardXZ);
        float sideCoord = Vector3.Dot(local, _swarmRightXZ);

        float fullLength = _envelopeLength;   // same length as before

        bool inSide = Mathf.Abs(sideCoord) <= _envelopeRadius;
        bool inFront = forwardCoord >= 0f && forwardCoord <= fullLength;

        return inSide && inFront;

    }

    // Check if a world-space point lies inside the look-direction envelope (Rectangle B)
    private bool IsInsideLookEnvelope(Vector3 point)
    {
        if (!_hasEnvelope)
            return false;

        Vector3 flatPoint = point;
        flatPoint.y = 0f;

        Vector3 flatCenter = _envelopeCenterXZ;
        flatCenter.y = 0f;

        Vector3 local = flatPoint - flatCenter;

        float forwardCoord = Vector3.Dot(local, _lookForwardXZ);
        float sideCoord = Vector3.Dot(local, _lookRightXZ);

        float fullLength = lookEnvelopeLength;

        bool inSide = Mathf.Abs(sideCoord) <= _envelopeRadius;
        bool inFront = forwardCoord >= 0f && forwardCoord <= fullLength;

        return inSide && inFront;

    }

    private bool IsInsideSafetyEnvelope(Vector3 point)
    {
        if (!_hasEnvelope)
            return false;

        // radius = a*w + b
        float radius = safety_radius_scale * _envelopeRadius + safety_radius_offset;

        Vector3 C = _envelopeCenterXZ;
        C.y = 0f;

        Vector3 P = point;
        P.y = 0f;

        float distXZ = Vector3.Distance(P, C);
        return distXZ <= radius;
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
    // ComputeDistanceToObstacle (new version with swarm envelope gating)
    //
    // 1) Use cached swarm envelope (centroid, axes, width, length).
    // 2) Compute closest point P on obstacle collider to centroid C.
    // 3) Discard obstacle if P lies outside the swarm envelope rectangle.
    // 4) If inside, find nearest drone to P.
    // 5) Cache closest drone + distance + direction.
    // 6) Return |P - d_closest| used for beeps and volume.
    // ------------------------------------------------------------
    private float ComputeDistanceToObstacle(ObstacleAudio o)
    {
        Transform obstacleTf = o.transform;

        var drones = _cachedDrones;
        if (drones == null || drones.Count == 0)
            return Vector3.Distance(listenerTransform.position, obstacleTf.position);

        // Get collider
        var col = obstacleTf.GetComponent<Collider>() ?? obstacleTf.GetComponentInParent<Collider>();
        if (col == null)
            return float.MaxValue;

        // === 1. Evaluate ALL drones: compute each Pᵢ and distance dᵢ ===
        Transform bestDrone = null;
        Vector3 bestPoint = Vector3.zero;
        float bestDistSqr = float.PositiveInfinity;

        foreach (var d in drones)
        {
            // Closest point from THIS drone to the obstacle surface
            Vector3 Pi = col.ClosestPoint(d.position);

            // Compute squared distance
            float sq = (d.position - Pi).sqrMagnitude;
            if (sq < bestDistSqr)
            {
                bestDistSqr = sq;
                bestDrone = d;
                bestPoint = Pi;
            }
        }
        // Record projection point for gizmos (optional)
        allPointsP.Add(bestPoint);

        // No drone found (should not happen)
        if (bestDrone == null)
            return float.MaxValue;

        float bestDistance = Mathf.Sqrt(bestDistSqr);

        // === 2. Envelope gating (A/B/C) ONLY on XZ using bestPoint ===
        bool insideA = IsPointInsideEnvelope(bestPoint);
        bool insideB = IsInsideLookEnvelope(bestPoint);
        bool insideC = IsInsideSafetyEnvelope(bestPoint);

        if (!(insideA || insideB || insideC))
        {
            if (_rt.TryGetValue(o, out var rOutside))
            {
                rOutside.closestDrone = null;
                rOutside.closestDistance = -1f;
                rOutside.dirDroneToP_XZ = Vector3.zero;
                _rt[o] = rOutside;
            }

            float cap = 10f;
            if (_rt.TryGetValue(o, out var rProf) && rProf.profile != null)
                cap = rProf.profile.maxAudibleDistance;

            return cap + 1f;
        }

        // === 3. Cache final result for audio ===
        if (_rt.TryGetValue(o, out var rForThis))
        {
            rForThis.closestDrone = bestDrone;
            rForThis.closestDistance = bestDistance;

            // Direction drone -> obstacle (XZ)
            Vector3 dir = bestPoint - bestDrone.position;
            dir.y = 0f;
            rForThis.dirDroneToP_XZ = (dir.sqrMagnitude > 1e-6f) ? dir.normalized : Vector3.zero;

            _rt[o] = rForThis;
        }

        return bestDistance;
    }


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

    private float ComputeDirectionalVolumeMultiplier(Runtime r)
    {
        if (!useDirectionalAttenuation || mpc == null || r == null || r.closestDrone == null)
            return 1f;

        Vector3 heading = mpc.GetSwarmHeading();
        Vector3 toObs = r.dirDroneToP_XZ;

        if (heading == Vector3.zero || toObs == Vector3.zero)
            return 1f;

        float dot = Vector3.Dot(heading, toObs);
        float mul01 = 1f - Mathf.Abs(dot);
        mul01 = (dot + 1) / 2;

        if (facingExponent != 1f) mul01 = Mathf.Pow(mul01, facingExponent);

        // Map mul01 to [minDirectionalVolume, maxDirectionalVolume]
        float lo = Mathf.Min(minDirectionalVolume, maxDirectionalVolume);
        float hi = Mathf.Max(minDirectionalVolume, maxDirectionalVolume);
        float mapped = Mathf.Lerp(lo, hi, mul01);

        return mapped; // this is the multiplier you apply to finalVolume
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
        float volMul = ComputeDirectionalVolumeMultiplier(r);

        // Use distance curves again (clamp distance to avoid crazy values)
        float clampedDist = p.maxAudibleDistance > 0f
            ? Mathf.Min(ctx.distance, p.maxAudibleDistance)
            : ctx.distance;

        float volDistMul = Mathf.Max(0f, p.volumeByDistance.Evaluate(clampedDist));
        float pitchDistMul = Mathf.Max(0.01f, p.pitchByDistance.Evaluate(clampedDist));
        float pitchSizeMul = Mathf.Max(0.01f, p.pitchBySize.Evaluate(ctx.size));

        // Final volume now uses volumeByDistance again
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

            // Do not mute while a beep is currently playing; let it finish naturally
            o.source.mute = !audible && !o.source.isPlaying;
            if (!audible && !o.source.isPlaying) return;

            // Apply pitch to the source; pass volume only in PlayOneShot to avoid double attenuation
            o.source.pitch = finalPitch;
            o.source.volume = 1f;

            float rateHz = bProf.GetPulseRate(clampedDist);
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

#if UNITY_EDITOR
private void OnDrawGizmos()
{
    // ===== Forward-only Velocity Envelope (Yellow) =====
    if (_hasEnvelope)
    {
        Vector3 center = _envelopeCenterXZ;
        float y = (listenerTransform != null) ? listenerTransform.position.y : center.y;
        center.y = y;

        float fullL = _envelopeLength;      // full forward length
        float halfW = _envelopeRadius;

        Vector3 f = _swarmForwardXZ.normalized * fullL;
        Vector3 r = _swarmRightXZ.normalized * halfW;

        // Back edge at the centroid
        Vector3 p0 = center - r;
        Vector3 p1 = center + r;

        // Front edge fullL forward
        Vector3 front = center + f;
        Vector3 p2 = front + r;
        Vector3 p3 = front - r;

        Handles.color = new Color(1f, 1f, 0f, 0.8f); // thick yellow
        Handles.DrawAAPolyLine(
            6f,
            p0, p1, p2, p3, p0
        );
    }

    // ===== Forward-only Look Envelope (Blue) =====
    if (_hasEnvelope)
    {
        Vector3 center = _envelopeCenterXZ;
        float y = (listenerTransform != null) ? listenerTransform.position.y : center.y;
        center.y = y;

        float fullL = lookEnvelopeLength;   // full forward length
        float halfW = _envelopeRadius;

        Vector3 f = _lookForwardXZ.normalized * fullL;
        Vector3 r = _lookRightXZ.normalized * halfW;

        // Back edge at centroid
        Vector3 p0b = center - r;
        Vector3 p1b = center + r;

        // Front edge
        Vector3 frontB = center + f;
        Vector3 p2b = frontB + r;
        Vector3 p3b = frontB - r;

        Handles.color = new Color(0f, 0.4f, 1f, 0.8f); // blue
        Handles.DrawAAPolyLine(
            6f,
            p0b, p1b, p2b, p3b, p0b
        );
    }

    // ===== Safety Envelope (Circle C) =====
    if (_hasEnvelope)
    {
        float radius = safety_radius_scale * _envelopeRadius + safety_radius_offset;

        Vector3 center = _envelopeCenterXZ;
        float y = (listenerTransform != null) ? listenerTransform.position.y : center.y;
        center.y = y;

        Handles.color = new Color(1f, 0.2f, 0.2f, 0.9f);

        const int segments = 64;
        Vector3[] pts = new Vector3[segments + 1];

        for (int i = 0; i <= segments; i++)
        {
            float t = (float)i / segments;
            float ang = t * Mathf.PI * 2f;

            float x = Mathf.Cos(ang) * radius;
            float z = Mathf.Sin(ang) * radius;

            pts[i] = center + new Vector3(x, 0f, z);
        }

        Handles.DrawAAPolyLine(6f, pts);
    }

    // ===== Centroid Debug Marker =====
    if (_hasPointC)
    {
        Gizmos.color = Color.cyan;
        Vector3 c = _lastPointC;
        c.y += 0.2f;
        Gizmos.DrawSphere(c, 0.2f);
    }

    // ===== Projection Debug Marker =====
    Gizmos.color = Color.red;
    foreach (var p in allPointsP)
    {
        Vector3 pos = p;
        pos.y += 0.2f;
        Gizmos.DrawSphere(pos, 0.2f);
    }

}
#endif
}
