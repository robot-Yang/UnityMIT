using UnityEngine;

[CreateAssetMenu(fileName = "BeepAudioProfile", menuName = "Audio/Profiles/Beep")]
public class BeepAudioProfile : ObstacleAudioProfileBase
{
    [Tooltip("Clamp the min and max pulse rates.")]
    public Vector2 pulseRateClamp = new Vector2(1f, 6f);

    [Header("Clip")]
    [Tooltip("Short beep to be triggered at the pulse rate.")]
    public AudioClip beepClip;

    
    [Header("Pulse Inverse-Square Settings")]
    public float k = 120;
    public float eps = 2f;
    public float offset = 1f;

    public override float GetPulseRate(float distance)
    {
        float pulse = (k / ((distance + eps) * (distance + eps))) + offset;
        Debug.Log("Distance: "+distance+"  pulse: "+pulse);
        return pulse;
    }

}
