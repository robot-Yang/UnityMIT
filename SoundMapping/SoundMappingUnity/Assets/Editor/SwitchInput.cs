using UnityEditor;
using System.IO;

public static class InputManagerSwitcher
{
    const string inputPath = "ProjectSettings/InputManager.asset";

    [MenuItem("Tools/Input/Use InputManager A")]
    static void UseA()
    {
        File.Copy("ProjectSettings/InputManager_A.asset", inputPath, true);
        AssetDatabase.Refresh();
    }

    [MenuItem("Tools/Input/Use InputManager B")]
    static void UseB()
    {
        File.Copy("ProjectSettings/InputManager_B.asset", inputPath, true);
        AssetDatabase.Refresh();
    }
}
