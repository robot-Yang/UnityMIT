using UnityEngine;
using UnityEngine.UI;

public class TactorVisualizer : MonoBehaviour
{
    [Header("Assign in Inspector")]
    public HapticsTest haptics;          // drag your existing script
    public Canvas prefab;               // drag the prefab from step 1
    public Gradient colourRamp;         // edit → blue (0) to red (14)

    Image[] _cells;

    void Start()
    {
        // Spawn UI in front of camera or as a child of the GameObject that owns this script
        Canvas panel = Instantiate(prefab, transform);
        panel.transform.localPosition = new Vector3(0.3f, 120f, 0.6f); // tweak in Scene
        panel.transform.localRotation = Quaternion.identity;

        _cells = panel.GetComponentsInChildren<Image>();
        // if (_cells.Length != 20)
        //     Debug.LogWarning("Need exactly 20 Image children for 5×4 matrix.");
    }

    void Update()
    {
        // if (haptics == null) { Debug.LogError("Visualizer: Haptics reference missing"); return; }

        // Read the latest duty table from HapticsTest
        int[] dutyByTile = haptics.GetDutySnapshot();  // ← add this accessor below

        for (int i = 0; i < _cells.Length; i++)
        {
            // if (i == 0 || i == 3 || i == 16 || i == 19)  // skip the corner cell if they are not used
            //     continue; // skip the first cell (index 0) if it's not used
            float t = dutyByTile[i] / 14f;             // 0…1
            _cells[i].color = colourRamp.Evaluate(t);
            // Debug.Log($"Cell {i}: Duty = {dutyByTile[i]}, Color = {_cells[i].color}");
        }
    }
}
