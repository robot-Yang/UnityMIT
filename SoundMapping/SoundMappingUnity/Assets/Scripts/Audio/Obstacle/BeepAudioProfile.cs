using UnityEngine;

[CreateAssetMenu(fileName = "BeepAudioProfile", menuName = "Audio/Profiles/Beep")]
public class BeepAudioProfile : ObstacleAudioProfileBase
{
    [Header("Pulse")]
    [Tooltip("Pulse rate in Hz as a function of distance.")]
    public AnimationCurve pulseRateByDistance = AnimationCurve.Linear(0f, 4f, 60f, 0.5f);

    [Tooltip("Clamp the min and max pulse rates.")]
    public Vector2 pulseRateClamp = new Vector2(0.2f, 6f);

    [Header("Clip")]
    [Tooltip("Short beep to be triggered at the pulse rate.")]
    public AudioClip beepClip;
}
