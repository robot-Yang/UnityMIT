using UnityEngine;

[CreateAssetMenu(fileName = "ContinuousAudioProfile", menuName = "Audio/Profiles/Continuous")]
public class ContinuousAudioProfile : ObstacleAudioProfileBase
{
    [Header("Clip")]
    [Tooltip("Looping clip for continuous spatial sound.")]
    public AudioClip loopClip;
}
