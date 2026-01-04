using UnityEngine;
using System.Collections;
using System.Collections.Generic;


public class WindSoundManager : MonoBehaviour
{
    public GameObject windSoundPrefab;
    public Transform windSoundParent;
    public int maxWindSounds = 10;
    public float spawnInterval = 0.05f;
    public float speedThreshold = 5f;
    public float windSoundLifetime = 0.1f;
    public float windSoundSpeedMultiplier = 2f;
    public float spawnDistanceAhead = 1f;

    public GameObject player;
    private float lastSpawnTime;
    private List<GameObject> activeWindSounds = new List<GameObject>();
    private List<Coroutine> activeCoroutines = new List<Coroutine>();

    private Vector3 lastPosition;
    private Vector3 speedVec;
    private float lastBelowThresholdTime;



    void Start()
    {

        if (windSoundPrefab == null)
        {
            // Debug.LogError("WindSoundPrefab is not assigned.");
        }
    }

    void Update()
    {
        // Get the player's Rigidbody component
        Vector3 playerPosition = player.transform.position;
        speedVec = (lastPosition -playerPosition) / Time.deltaTime;
        float speed = speedVec.magnitude;

        lastPosition = playerPosition;


        // Check if the player is moving above the speed threshold
        if (speed > speedThreshold)
        {
            // Spawn wind sounds at intervals
            if (Time.time - lastSpawnTime >= spawnInterval && activeWindSounds.Count < maxWindSounds)
            {
                SpawnWindSound(speed);
                lastSpawnTime = Time.time;
            }
            lastBelowThresholdTime = Time.time;
        }
        else
        {
            if (Time.time - lastBelowThresholdTime > 0.2f)
            {
                // Kill all wind sounds and stop coroutines
                foreach (GameObject sound in activeWindSounds)
                {
                    Destroy(sound);
                }
                foreach (Coroutine coroutine in activeCoroutines)
                {
                    StopCoroutine(coroutine);
                }
                activeCoroutines.Clear();
            }
        }

        // Clean up null references from the list
        activeWindSounds.RemoveAll(sound => sound == null);
    }

    void SpawnWindSound(float speed)
    {
        GameObject windSound = Instantiate(windSoundPrefab);
        if (windSoundParent != null)
        {
            windSound.transform.parent = windSoundParent;
        }

        // Set the position of the wind sound ahead of the player
        Vector3 spawnPosition = windSoundParent.position + speedVec.normalized * spawnDistanceAhead;
        windSound.transform.position = spawnPosition;

        // Adjust the pitch based on speed
        AudioSource audioSource = windSound.GetComponent<AudioSource>();
        if (audioSource != null)
        {
            audioSource.pitch = Mathf.Clamp(1f + (speed / 20f), 1f, 3f);
        }

        // Start the coroutine to move the wind sound and handle its lifetime
        Coroutine coroutine = null;
        coroutine = StartCoroutine(MoveWindSound(windSound, speed, () => activeCoroutines.Remove(coroutine)));
        activeCoroutines.Add(coroutine);

        // Add to active wind sounds list
        activeWindSounds.Add(windSound);
    }

    IEnumerator MoveWindSound(GameObject windSound, float speed, System.Action onComplete)
    {
        // Calculate movement direction and speed
        Vector3 direction = -speedVec.normalized;
        float windSpeed = speed * windSoundSpeedMultiplier;

        // Get the AudioSource component
        AudioSource audioSource = windSound.GetComponent<AudioSource>();

        // Record the start time
        float startTime = Time.time;

        while (Time.time - startTime < windSoundLifetime)
        {
            // Move the wind sound
            windSound.transform.position += direction * windSpeed * Time.deltaTime;

            // Fade out volume over lifetime
            if (audioSource != null)
            {
                float elapsedTime = Time.time - startTime;
                audioSource.volume = Mathf.Lerp(0.3f, 0f, elapsedTime / windSoundLifetime);
            }

            yield return null;
        }

        // Destroy the wind sound after its lifetime expires
        Destroy(windSound);

        // Remove the coroutine from the active list
        onComplete();
    }
}
