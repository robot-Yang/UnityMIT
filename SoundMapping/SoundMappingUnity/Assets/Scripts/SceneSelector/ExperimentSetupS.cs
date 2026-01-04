using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

using TMPro;

public class ExperimentSetupS : MonoBehaviour
{
    public string  PID;
    public Toggle Haptics;
    public Toggle Order;

    //text TMPro
    public TMP_InputField PIDInput;
    public TMP_Text confirmText;

    public GameObject confirmGO;


    public GameObject mainMenu;
    public GameObject experimentMenu;


    public TMP_Text savingText;
    public GameObject nextButton;

    // Start is called before the first frame update
    void Start()
    {
        confirmGO.SetActive(false);
        mainMenu.SetActive(true);
        experimentMenu.SetActive(false);
    }

    public void GUIIDisable()
    {
        confirmGO.SetActive(false);
        mainMenu.SetActive(false);
        experimentMenu.SetActive(false);
    }


    public void confirm()
    {
        GUIIDisable();

        this.GetComponent<SceneSelectorScript>().StartTraining(PID);
    }

    public void cancel()
    {
        confirmGO.SetActive(false);
    }


    // Update is called once per frame

    public void StartExperiment()
    {
        PID = PIDInput.text;
        if (PID.Length < 3)
        {
            // Debug.LogError("PID must be at least 3 characters long.");
            return;
        }

        SceneSelectorScript._haptics = PID[0] == 'H';
        SceneSelectorScript._order = PID[1] == 'T';
        SceneSelectorScript.pid = PID.Substring(2);


        confirmText.text = "Are you sure you want to start the experiment? \n\n" +
            "PID: " + SceneSelectorScript.pid + "\n" +
            "Haptics: " + SceneSelectorScript._haptics + "\n" +
            "TDV first : " + SceneSelectorScript._order;


        confirmGO.SetActive(true);
    }


}
