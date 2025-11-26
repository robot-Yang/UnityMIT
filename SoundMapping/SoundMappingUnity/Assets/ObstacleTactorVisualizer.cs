using UnityEngine;
using UnityEngine.UI;

public class ObstacleTactorVisualizer : MonoBehaviour
{
    [SerializeField] private Image[] cells;          // 8 个格子，按 addr 顺序
    [SerializeField] private Gradient dutyGrad;      // 红→黄 渐变
    [SerializeField] private Color    idleColor = new Color(0,0,0,0); // 无马达时

    private static readonly int[] addr = HapticsTest.ObstacleAddrs;

    void Update()
    {
        int[] duty = HapticsTest.GetObstacleDutySnapshot();  // 长度 40
        // Debug.Log($"Obs duty = {string.Join(",", duty)}");

        for (int i = 0; i < cells.Length && i < addr.Length; i++)
        {
            int d = duty[addr[i]];                  // 0‥14
            // Debug.Log($"Cell {i}: Addr = {addr[i]}, Duty = {d}");
            float t = d / 14f;                      // 0‥1
            cells[i].color = d == 0
                           ? idleColor
                           : dutyGrad.Evaluate(t);  // 根据占空比取色
        }
    }
}
