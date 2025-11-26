using System.Data.SqlTypes;
using UnityEngine;
using TMPro;

public class LevelConfiguration : MonoBehaviour
{
    public bool SoftStart = true;
    public static string _textTutorial  = "";

    public static int _CollectibleNumber = 0;
    public static int sceneNumber = 0;


    [Header("Control Settings")]
    [SerializeField] public bool controlMovement = true;
    [SerializeField] public bool controlSpreadness = true;
    [SerializeField] public bool controlEmbodiement = true;
    [SerializeField] public bool controlDesembodiement = false;
    [SerializeField] public bool controlSelection = true;
    [SerializeField] public bool controlRotation = true;

    [Header("Haptics Settings")]
    [SerializeField] public bool hapticsObstacle = true;
    [SerializeField] public bool hapticsNetwork = true;
    [SerializeField] public bool hapticsForces = true;
    [SerializeField] public bool hapticsCrash = true;
    [SerializeField] public bool hapticsController = true;

    [Header("Start Configuration")]
    [SerializeField] public bool startEmbodied = false;
    [SerializeField] public int droneID = 0;


    [Header("Audio Settings")]
    [SerializeField] public bool audioIsolation = true;
    [SerializeField] public bool audioSpreadness = true;

    [Header("Spawn Settings")]
    [SerializeField] public bool needToSpawn = true;
    [SerializeField] public int numDrones = 20;
    [SerializeField] public float spawnRadius = 3f;
    [SerializeField] public float startSperation = 1f;

    [Header("Other")]
    [SerializeField] public bool saveData = false;
    [SerializeField] public bool miniMap = false;
    [SerializeField] public bool showText = false;

    [Header("Timing Settings")]
    [SerializeField] public bool timeSensitive = false;
    [SerializeField] public float timeToComplete = 60f;

    // Corresponding static variables
    public static bool _control_movement;
    public static bool _control_spreadness;
    public static bool _control_embodiement;
    public static bool _control_desembodiement;
    public static bool _control_selection;
    public static bool _control_rotation;

    public static bool _Haptics_Obstacle;
    public static bool _Haptics_Network;
    public static bool _Haptics_Forces;
    public static bool _Haptics_Crash;
    public static bool _Haptics_Controller;

    public static bool _startEmbodied;
    public static int _droneID;



    public static bool _Audio_isolation;
    public static bool _Audio_spreadness;

    public static bool _NeedToSpawn;
    public static int _NumDrones;
    public static float _SpawnRadius;
    public static float _StartSperation;

    public static bool _SaveData;
    public static bool _MiniMap;
    public static bool _ShowText;

    public static bool _TimeSensitive;
    public static float _TimeToComplete;





    void OnValidate()
    {
        if (Time.timeSinceLevelLoad < 2.9f && SoftStart)
        {
            return;
        }


        _control_movement = controlMovement;
        _control_spreadness = controlSpreadness;
        _control_embodiement = controlEmbodiement;
        _control_desembodiement = controlDesembodiement;
        _control_selection = controlSelection;
        _control_rotation = controlRotation;

        _Haptics_Obstacle =  SceneSelectorScript._haptics ? hapticsObstacle : false;
        _Haptics_Network = SceneSelectorScript._haptics ? hapticsNetwork : false;
        _Haptics_Forces = SceneSelectorScript._haptics ? hapticsForces : false;
        _Haptics_Crash = SceneSelectorScript._haptics ? hapticsCrash : false;
        _Haptics_Controller = SceneSelectorScript._haptics ? hapticsController : false;

        _startEmbodied = startEmbodied;
        _droneID = droneID;

        _Audio_isolation = SceneSelectorScript._haptics ? audioIsolation : false;
        _Audio_spreadness = SceneSelectorScript._haptics ? audioSpreadness : false;

        _NeedToSpawn = needToSpawn;
        _NumDrones = numDrones;
        _SpawnRadius = spawnRadius;
        _StartSperation = startSperation;

        _SaveData = saveData;
        _MiniMap = miniMap;
        _ShowText = showText;

        _TimeSensitive = timeSensitive;
        _TimeToComplete = timeToComplete;
    }

    void SoftStartFunc()
    {
        //put all the hapticd and ausdio to false
        _Haptics_Obstacle = false;
        _Haptics_Network = false;
        _Haptics_Forces = false;
        _Haptics_Crash = false;
        _Haptics_Controller = false;
        
        _Audio_isolation = false;
        _Audio_spreadness = false;

        _control_movement = controlMovement;
        _control_spreadness = controlSpreadness;
        _control_embodiement = controlEmbodiement;
        _control_desembodiement = controlDesembodiement;
        _control_selection = controlSelection;
        _control_rotation = controlRotation;

        _startEmbodied = startEmbodied;
        _droneID = droneID;

        _NeedToSpawn = needToSpawn;
        _NumDrones = numDrones;
        _SpawnRadius = spawnRadius;
        _StartSperation = startSperation;

        _SaveData = saveData;
        _MiniMap = miniMap;
        _ShowText = showText;

        //call the onvalidate 3 seconds later
        Invoke("lateStart", 3f);
    }

    void lateStart()
    {
        OnValidate();
        HapticsTest.lateStart();
    }

    // make a dico with the scene number and the text


    void setTextTuto()
    {
        string name = "";
        foreach (var scene in UnityEngine.SceneManagement.SceneManager.GetAllScenes())
        {
            if (char.IsDigit(scene.name[0]))
            {
//                print(scene.name);
                name = scene.name;
                break;
            }
        }
        //the names are 1 blabla 2 blabla 3 blabl
        
        //get the number

        //check if it is a number
        if(SceneSelectorScript.experimentNumber >=10){
            if(SceneSelectorScript.experimentNumber < 15)   
            {
                sceneNumber = 100;
            }
            else
            {
                sceneNumber = 200;
            }
        }else{
            //get the number
            sceneNumber = int.Parse(name.Split(' ')[0].ToString());
        }
        print("Scene number: " + sceneNumber);
        string haptics = SceneSelectorScript._haptics? "Haptics" : "NonHaptics";

       // string fileName = "Scene" + sceneNumber + haptics + ".txt";
        string fileName = "0000Scene" + sceneNumber + haptics + ".txt";
        string path = Application.dataPath + "/SceneDescription/" + fileName;

        //check if the files exists if not create it
        if(!System.IO.File.Exists(path))
        {
           // System.IO.File.WriteAllText(path, "Scene" + sceneNumber + haptics + "\n");
            System.IO.File.WriteAllText(path, "");
        }
    
        string[] lines = System.IO.File.ReadAllLines(path);

        string text = "";
        for(int i = 0; i < lines.Length; i++)
        {
            text += lines[i] + "\n";
        }

        _textTutorial = text;


        
    }

    void Awake()
    {        
        setTextTuto();

     //   GameObject.FindGameObjectWithTag("SceneDescription").GetComponent<TextMesh>().text = name;


        _CollectibleNumber = GameObject.FindGameObjectsWithTag("Collectibles").Length;
        
        showText = !SceneSelectorScript._haptics;

        showText = true;

        if(SoftStart)
        {
            SoftStartFunc();
        }
        else
        {
            OnValidate();
        }
    }

    public static GameObject swarmHolder
    {
        get
        {
            return GameObject.FindGameObjectWithTag("SpawnLess");
        }
    }

    void Update()
    {
    }

}
