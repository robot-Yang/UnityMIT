using UnityEngine;
using UnityEngine.Video;
using UnityEngine.Networking;
using System.Collections;
using System.IO;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

[RequireComponent(typeof(VideoPlayer), typeof(AudioSource))]
public class TutorialPlayer : MonoBehaviour
{
    [Header("Video UI Settings")]
    [SerializeField] private RenderTexture targetRenderTexture; // Assign in Inspector
    public GameObject tutoVideo;

    private static GameObject _thisGameObject;
     // (Optional) To auto-assign

        private VideoPlayer videoPlayer;
        private AudioSource audioSource;

        private void OnEnable()
        {
            videoPlayer.loopPointReached += OnVideoFinished;
        }

        private void OnDisable()
        {
            videoPlayer.loopPointReached -= OnVideoFinished;
        }

        private void Update()
        {
            if (Input.GetKeyDown(KeyCode.V))
            {
                PlayTutorial(LevelConfiguration.sceneNumber);
            }
            if (Input.GetKeyDown(KeyCode.S))
            {
                StopVideo();
            }
        }

        public static void stopVideo()
        {
            _thisGameObject.GetComponent<TutorialPlayer>().StopVideo();
        }

        public void StopVideo()
        {
            videoPlayer.Stop();
            audioSource.Stop();

            OnVideoFinished(videoPlayer);
        }

        private void OnVideoFinished(VideoPlayer vp)
        {
            AudioPriorityManager.Restore();
            MigrationPointController.InControl = true;
            tutoVideo.SetActive(false);
        }

        private void RestartVideo()
        {
            AudioPriorityManager.Mute();
            MigrationPointController.InControl = false;
            MigrationPointController.alignementVector = Vector3.zero;
            tutoVideo.SetActive(true);
            videoPlayer.Stop();
            audioSource.Stop();
            videoPlayer.Play();
            audioSource.Play();
        }

    private void Awake()
    {
        videoPlayer = GetComponent<VideoPlayer>();
        audioSource = GetComponent<AudioSource>();
        _thisGameObject = this.gameObject;

        
        // Configure VideoPlayer to output to RenderTexture
        videoPlayer.renderMode = VideoRenderMode.RenderTexture;
        videoPlayer.targetTexture = targetRenderTexture;

        // We handle audio from a separate MP3, so set this to None
        videoPlayer.audioOutputMode = VideoAudioOutputMode.None;


        MigrationPointController.InControl = false;
        MigrationPointController.alignementVector = Vector3.zero;
        // Get references

    }

    void Start()
    {
        // PlayTutorial(LevelConfiguration.sceneNumber);
    }


    public static void playTuto(int tutoNumber)
    {
        AudioPriorityManager.Mute();
        
        if(!SceneSelectorScript.needToWatchTutorial())
        {
            stopVideo();
            return;
        }

        _thisGameObject.GetComponent<TutorialPlayer>().PlayTutorial(tutoNumber);
    }

    public void PlayTutorial(int tutoNumber)
    {

        tutoVideo.SetActive(true);

        tutoVideo.GetComponent<RawImage>().texture = targetRenderTexture;
        string path = Application.dataPath + "/Scenes/TrainingFinal/VideosTuto/";
        string tutorialFolder = "Tuto " + tutoNumber.ToString();


        // (Same code as before)
        string folderPath = Path.Combine(path, tutorialFolder);

        if (!Directory.Exists(folderPath))
        {
            OnVideoFinished(videoPlayer);
            return;
        }

        string[] mp4Files = Directory.GetFiles(folderPath, "*.mp4", SearchOption.TopDirectoryOnly);
        if (mp4Files.Length == 0)
        {
            // Debug.LogError("No .mp4 file found in folder: " + folderPath);
            return;
        }

        string[] mp3Files = Directory.GetFiles(folderPath, "*.mp3", SearchOption.TopDirectoryOnly);
        if (mp3Files.Length == 0)
        {
            // Debug.LogError("No .mp3 file found in folder: " + folderPath);
            return;
        }

        // Assign the first MP4 file to the VideoPlayer
        string mp4FullPath = mp4Files[0];
        videoPlayer.url = "file://" + mp4FullPath;

        // Load and assign the first MP3 file to the AudioSource
        string mp3FullPath = mp3Files[0];
        StartCoroutine(LoadAudio(mp3FullPath));
    }

    private IEnumerator LoadAudio(string filePath)
    {
        using (UnityWebRequest www = UnityWebRequestMultimedia.GetAudioClip("file://" + filePath, AudioType.MPEG))
        {
            yield return www.SendWebRequest();

            if (www.result != UnityWebRequest.Result.Success)
            {
                // Debug.LogError("Error loading audio: " + www.error);
            }
            else
            {
                AudioClip clip = DownloadHandlerAudioClip.GetContent(www);
                audioSource.clip = clip;

                // Start both video + audio
                videoPlayer.Play();
                audioSource.Play();
            }
        }
    }
}
