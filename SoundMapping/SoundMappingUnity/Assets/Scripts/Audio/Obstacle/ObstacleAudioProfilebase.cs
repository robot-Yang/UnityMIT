using UnityEngine;

public abstract class ObstacleAudioProfileBase : ScriptableObject
{
    [Header("Audibility")]
    [Tooltip("Max distance at which an obstacle should be audible.")]
    public float maxAudibleDistance = 6f;
    
    [Header("Base")]
    [Tooltip("Base output volume applied before curve evaluation.")]
    [Range(0f, 1f)] public float baseVolume = 0.8f;

    [Tooltip("Base pitch applied before curve evaluation.")]
    [Range(0.1f, 3f)] public float basePitch = 1f;

    [Header("Distance/Size Mappings")]
    [Tooltip("Volume multiplier as a function of distance [0..maxAudibleDistance].")]
    public AnimationCurve volumeByDistance = AnimationCurve.EaseInOut(0f, 1f, 60f, 0f);

    [Tooltip("Pitch multiplier as a function of distance [0..maxAudibleDistance].")]
    public AnimationCurve pitchByDistance = AnimationCurve.Linear(0f, 1.2f, 60f, 0.8f);

    [Tooltip("Additional pitch multiplier by obstacle 'size' (arbitrary scalar per obstacle).")]
    public AnimationCurve pitchBySize = AnimationCurve.Linear(0.5f, 0.9f, 3f, 1.1f);

    //[Header("Optional Low-Pass")]
    //[Tooltip("Cutoff (Hz) as a function of distance. Set to 0 to disable.")]
    //public AnimationCurve lowpassCutoffByDistance = new AnimationCurve(
    //    new Keyframe(0f, 18000f),
    //    new Keyframe(60f, 3000f)
    //);
}
