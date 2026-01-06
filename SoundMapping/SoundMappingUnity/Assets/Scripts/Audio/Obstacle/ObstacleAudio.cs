using UnityEngine;

[RequireComponent(typeof(AudioSource))]
public class ObstacleAudio : MonoBehaviour
{
    [Tooltip("Approximate size used for audio mapping. Can be collider bounds size or any scalar you like.")]
    public float sizeScalar = 1f;

    [HideInInspector] public AudioSource source;

    private void Reset()
    {
        var a = GetComponent<AudioSource>();
        a.playOnAwake = false;
        a.loop = false;
        a.spatialBlend = 1f;
        a.rolloffMode = AudioRolloffMode.Linear;
        a.minDistance = 1f;
        a.maxDistance = 80f;
        a.dopplerLevel = 0f;
        a.priority = 0;
    }


    private void Awake()
    {
        source = GetComponent<AudioSource>();
    }

    private void OnEnable()
    {
        if (ObstacleAudioManager.Instance != null)
            ObstacleAudioManager.Instance.Register(this);
    }

    private void OnDisable()
    {
        if (ObstacleAudioManager.Instance != null)
            ObstacleAudioManager.Instance.Unregister(this);
    }

#if UNITY_EDITOR
    private void OnDrawGizmosSelected()
    {
        var prof = ObstacleAudioManager.Instance != null
            ? ObstacleAudioManager.Instance.GetAssignedProfileFor(this)
            : null;

        if (prof == null) return;
        Gizmos.color = new Color(0.2f, 0.7f, 1f, 0.35f);
        // Gizmos.DrawWireSphere(transform.position, prof.maxAudibleDistance);
    }
#endif
}
