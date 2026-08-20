using UnityEngine;
using System.Collections;

/// <summary>
/// Управляет гонкой: считает круги и перезапускает ILC между кругами
/// </summary>
public class RaceManager : MonoBehaviour
{
    [Header("Controller Reference")]
    public SingularPID_ILC controller;

    [Header("Race Settings")]
    public int   totalLaps             = 5;
    public int   finishIndexThreshold  = 5;

    [Header("Debug")]
    public bool showDebugInfo = true;

    private int     currentLap   = 0;
    private float[] lapTimes;
    private bool    raceFinished = false;

    void Start()
    {
        if (controller == null) controller = FindObjectOfType<SingularPID_ILC>();
        if (controller == null) { Debug.LogError("RaceManager: контроллер не назначен!"); return; }
        lapTimes = new float[totalLaps];
        StartCoroutine(RaceLoop());
    }

    IEnumerator RaceLoop()
    {
        Debug.Log($"[RaceManager] Старт! Кругов: {totalLaps}");
        while (currentLap < totalLaps && !raceFinished)
        {
            Debug.Log($"[RaceManager] Круг {currentLap + 1} начат...");
            yield return StartCoroutine(WaitForLapCompletion());

            if (currentLap < totalLaps)
            {
                Debug.Log($"[RaceManager] ✓ Круг {currentLap + 1}/{totalLaps} — {lapTimes[currentLap]:F2}с");
                controller.ResetLapIndex();
                currentLap++;
            }
        }
        raceFinished = true;
        OnRaceFinished();
    }

    IEnumerator WaitForLapCompletion()
    {
        float lapStart        = Time.time;
        bool  hasLeftStart    = false;
        int   leaveThreshold  = finishIndexThreshold * 3;

        while (true)
        {
            if (controller == null || controller.trackFollower == null || controller.vehicle == null)
            { Debug.LogError("RaceManager: потеряна ссылка!"); yield break; }

            Vector2 pos = new Vector2(controller.vehicle.x, controller.vehicle.y);
            int idx = controller.trackFollower.trackPath.GetNearestPointIndex(pos);

            if (!hasLeftStart && idx > leaveThreshold) hasLeftStart = true;

            if (hasLeftStart && idx < finishIndexThreshold)
            {
                lapTimes[currentLap] = Time.time - lapStart;
                if (showDebugInfo) Debug.Log($"[RaceManager] Финиш круга! Время: {lapTimes[currentLap]:F2}с");
                yield break;
            }
            yield return new WaitForFixedUpdate();
        }
    }

    void OnRaceFinished()
    {
        Debug.Log("🏁 ГОНКА ЗАВЕРШЕНА! 🏁");
        float total = 0f; float best = float.MaxValue; int bestLap = 0;
        for (int i = 0; i < lapTimes.Length; i++)
        {
            total += lapTimes[i];
            Debug.Log($"  Круг {i + 1}: {lapTimes[i]:F2}с");
            if (lapTimes[i] < best) { best = lapTimes[i]; bestLap = i + 1; }
        }
        Debug.Log($"  Лучший круг: #{bestLap} — {best:F2}с");
        Debug.Log($"  Среднее:     {total / totalLaps:F2}с");

        if (controller?.vehicle != null) controller.vehicle.v = 0f;
    }

    void OnGUI()
    {
        if (!showDebugInfo || controller == null) return;
        GUILayout.BeginArea(new Rect(Screen.width - 260, 10, 250, 120));
        GUILayout.Box("🏎 PID-ILC Syngular Race");
        GUILayout.Label(raceFinished ? "ФИНИШ" : $"Круг {currentLap + 1}/{totalLaps}");
        if (currentLap > 0)
        {
            float best = float.MaxValue;
            for (int i = 0; i < currentLap; i++) if (lapTimes[i] < best) best = lapTimes[i];
            GUILayout.Label($"Лучший круг: {best:F2}с");
        }
        if (controller.enableILC) GUILayout.Label($"ILC: обучение {currentLap + 1}");
        GUILayout.EndArea();
    }

    public void ResetRace()
    {
        StopAllCoroutines();
        currentLap = 0; raceFinished = false;
        lapTimes = new float[totalLaps];
        controller?.ResetController();
        StartCoroutine(RaceLoop());
    }

    public int   CurrentLap      => currentLap;
    public bool  IsRaceFinished  => raceFinished;
    public float[] LapTimes      => lapTimes;
}