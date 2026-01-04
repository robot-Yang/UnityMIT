// Name this script "ColorSpacerExample"

using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(BrownToBlueNoise))]
public class BrownToBlueNoiseInspector : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        BrownToBlueNoise myScript = (BrownToBlueNoise)target;
        EditorGUILayout.BeginHorizontal();
        if(GUILayout.Button("Shrink"))
        {
            myScript.Shrink();
        }

        if(GUILayout.Button("Expand"))
        {
            myScript.Expand();
        }
        EditorGUILayout.EndHorizontal();
    }
}

//same thing for Pitch
[CustomEditor(typeof(PitchNoise))]
public class PitchNoiseInspector : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        PitchNoise myScript = (PitchNoise)target;
        EditorGUILayout.BeginHorizontal();
        if(GUILayout.Button("Shrink"))
        {
            myScript.Shrink();
        }

        if(GUILayout.Button("Expand"))
        {
            myScript.Expand();
        }
        EditorGUILayout.EndHorizontal();
    }
}

//same thing for BrownToPinkNoise
[CustomEditor(typeof(BrownToPinkNoise))]
public class buttonInspector : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        BrownToPinkNoise myScript = (BrownToPinkNoise)target;
        EditorGUILayout.BeginHorizontal();
        if(GUILayout.Button("Shrink"))
        {
            myScript.Shrink();
        }

        if(GUILayout.Button("Expand"))
        {
            myScript.Expand();
        }
        EditorGUILayout.EndHorizontal();
    }
}

//same thing for BandpassNoise
[CustomEditor(typeof(BandpassNoise))]
public class BandpassNoiseInspector : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        BandpassNoise myScript = (BandpassNoise)target;
        EditorGUILayout.BeginHorizontal();
        if(GUILayout.Button("Shrink"))
        {
            myScript.Shrink();
        }

        if(GUILayout.Button("Expand"))
        {
            myScript.Expand();
        }
        EditorGUILayout.EndHorizontal();
    }
}

//same thing for ArmyShrink
[CustomEditor(typeof(ArmyShrink))]
public class ArmyShrinkInspector : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        ArmyShrink myScript = (ArmyShrink)target;
        EditorGUILayout.BeginHorizontal();
        if(GUILayout.Button("Shrink"))
        {
            myScript.Shrink();
        }

        if(GUILayout.Button("Expand"))
        {
            myScript.Expand();
        }
        EditorGUILayout.EndHorizontal();
    }
}


[CustomEditor(typeof(SwarmDisconnection))]
public class swarmDisconnectionInspector : Editor
{
    // Keep track of scroll position in the inspector
    private Vector2 scrollPos;

    public override void OnInspectorGUI()
    {

        // Draw default inspector fields
        DrawDefaultInspector();

        SwarmDisconnection myScript = (SwarmDisconnection)target;
        EditorGUILayout.BeginVertical();

        // Example buttons
        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("Test Sound"))
        {
            myScript.PlaySound();
        }
        if (GUILayout.Button("Regenerate Drone Disconnection"))
        {
            myScript.RegenerateSwarm();
        }
        EditorGUILayout.EndHorizontal();

        // Another button row
        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("Randomize Drone Disconnection Test"))
        {
            myScript.RandomizeSwarm();
        }
        EditorGUILayout.EndHorizontal();

        
        // Make a selectable list of strings
        EditorGUILayout.BeginHorizontal();
        EditorGUILayout.LabelField("Select option for drone Isolation sound :");
        scrollPos = EditorGUILayout.BeginScrollView(scrollPos, GUILayout.Height(100));
        int selected = GUILayout.SelectionGrid(myScript.selectedOption, myScript.options.ToArray(), 1);
        if (selected != -1)
        {
            myScript.selectedOption = selected;
            
            //call callback only if mouse is pressed
            if (Event.current.type == EventType.Used)
            {
                // Debug.Log(Event.GetEventCount());
                // Debug.Log("Selected option changed to: " + selected);
                OnSelectionChanged(selected);
            }
        }
        EditorGUILayout.EndScrollView();
        EditorGUILayout.EndHorizontal();    

        // Show the selected option
        EditorGUILayout.LabelField("Selected Option:", myScript.options[myScript.selectedOption]);

        EditorGUILayout.EndVertical();
    }

    private void OnSelectionChanged(int selected)
    {
        // Handle the selection change
        // Debug.Log("Selected option changed to: " + selected);
        // Add any additional logic you need here
        SwarmDisconnection myScript = (SwarmDisconnection)target;
        myScript.StopAndPlaySound(selected);
    }
}