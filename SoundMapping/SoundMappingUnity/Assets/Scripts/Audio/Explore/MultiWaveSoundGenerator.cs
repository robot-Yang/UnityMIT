using UnityEngine;

[RequireComponent(typeof(AudioSource))]
public class MultiWaveSoundGenerator : MonoBehaviour
{
    public int sampleRate = 44100; // CD-quality sample rate
    public WaveType[] waveTypes;   // Array to hold wave types
    public float[] frequencies;    // Frequencies of the waves
    public float duration = 1.0f;  // Duration of the sound in seconds

    private AudioSource audioSource;
    private AudioClip audioClip;

    public enum WaveType { Sine, Square, Triangle, Sawtooth }

    void Start()
    {
    //    audioSource = GetComponent<AudioSource>();
    //    
    //    // Generate the audio clip with multiple sound waves
    //    audioClip = CreateMultiWaveAudioClip(frequencies, waveTypes, duration);
    //    
    //    audioSource.clip = audioClip;
    //    audioSource.loop = true;  // Loop the sound
    //    audioSource.Play();       // Play the sound
    }

    // Generate an AudioClip with multiple sound waves of specified frequencies and waveforms
    AudioClip CreateMultiWaveAudioClip(float[] frequencies, WaveType[] waveTypes, float duration)
    {
        if (frequencies.Length != waveTypes.Length)
        {
            // Debug.LogError("Frequencies and waveTypes arrays must have the same length.");
            return null;
        }

        int sampleLength = Mathf.CeilToInt(sampleRate * duration);  // Total number of samples
        float[] samples = new float[sampleLength];

        // Generate each wave and add to the sample array
        for (int w = 0; w < frequencies.Length; w++)
        {
            float[] waveSamples = GenerateWave(frequencies[w], waveTypes[w], duration);

            // Add the generated wave to the overall sample array
            for (int i = 0; i < sampleLength; i++)
            {
                samples[i] += waveSamples[i];  // Combine the waves by summing the sample values
            }
        }

        // Normalize the final sound to prevent clipping
        float maxAmplitude = Mathf.Max(Mathf.Abs(Mathf.Min(samples)), Mathf.Max(samples));
        if (maxAmplitude > 1.0f)
        {
            for (int i = 0; i < sampleLength; i++)
            {
                samples[i] /= maxAmplitude;  // Normalize to prevent distortion
            }
        }

        AudioClip audioClip = AudioClip.Create("MultiWave", sampleLength, 1, sampleRate, false);
        audioClip.SetData(samples, 0);
        return audioClip;
    }

    // Generates an array of samples for a given wave type and frequency
    float[] GenerateWave(float frequency, WaveType waveType, float duration)
    {
        int sampleLength = Mathf.CeilToInt(sampleRate * duration);
        float[] samples = new float[sampleLength];

        for (int i = 0; i < sampleLength; i++)
        {
            float time = i / (float)sampleRate; // Time in seconds
            float angle = 2.0f * Mathf.PI * frequency * time;

            switch (waveType)
            {
                case WaveType.Sine:
                    samples[i] = Mathf.Sin(angle);
                    break;
                case WaveType.Square:
                    samples[i] = Mathf.Sign(Mathf.Sin(angle));
                    break;
                case WaveType.Triangle:
                    samples[i] = Mathf.PingPong(frequency * time, 1.0f) * 2.0f - 1.0f;
                    break;
                case WaveType.Sawtooth:
                    samples[i] = (2.0f * (time * frequency - Mathf.Floor(0.5f + time * frequency)));
                    break;
            }
        }

        return samples;
    }
}
