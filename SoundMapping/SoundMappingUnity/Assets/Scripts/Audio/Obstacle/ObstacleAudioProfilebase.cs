using UnityEngine;

public abstract class ObstacleAudioProfileBase : ScriptableObject
{
    [Header("Audibility")]
    [Tooltip("Max distance at which an obstacle should be audible.")]
    public float maxAudibleDistance = 15f;
    
    [Header("Base")]
    [Tooltip("Base output volume applied before curve evaluation.")]
    [Range(0f, 1f)] public float baseVolume = 0.8f;

    [Tooltip("Base pitch applied before curve evaluation.")]
    [Range(0.1f, 3f)] public float basePitch = 0.755f;

    [Header("Distance/Size Mappings")]
    [Tooltip("Volume multiplier as a function of distance [0..maxAudibleDistance].")]
    public AnimationCurve volumeByDistance = AnimationCurve.EaseInOut(0f, 1f, 1f, 1f);

    [Tooltip("Pitch multiplier as a function of distance [0..maxAudibleDistance].")]
    public AnimationCurve pitchByDistance = AnimationCurve.Linear(0f, 1f, 1f, 1f);

    [Tooltip("Additional pitch multiplier by obstacle 'size' (arbitrary scalar per obstacle).")]
    public AnimationCurve pitchBySize = AnimationCurve.Linear(0f, 1f, 1f, 1f);

    public virtual float GetPulseRate(float distance)
    {
        // Default: profiles that do not override use a simple curve
        // Debug.Log("default pulse no override");
        return 1f;
    }

}
