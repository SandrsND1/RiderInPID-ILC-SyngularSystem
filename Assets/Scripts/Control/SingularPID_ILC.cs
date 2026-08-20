using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(CarVisual))]
public class SingularPID_ILC : MonoBehaviour
{
    // ── Ссылки ──────────────────────────────────────────────
    [Header("References")]
    public CarVisual      carVisual;
    public TrackFollower  trackFollower;

    // ── PID ─────────────────────────────────────────────────
    [Header("PID Parameters")]
    [Tooltip("Пропорциональный коэффициент. Больше → резче реакция, но риск колебаний.")]
    public float kp = 3.5f;

    [Tooltip("Интегральный коэффициент. Убирает статическую ошибку. Держи ≤ 0.15.")]
    public float ki = 0.08f;

    [Tooltip("Дифференциальный коэффициент. Гасит колебания. Держи ≤ 1.2.")]
    public float kd = 0.9f;

    [Tooltip("Ограничение накопленного интеграла (anti-windup).")]
    public float maxIntegral = 0.4f;

    [Tooltip("Фильтр производной [0-1]. 0.7 = хорошее подавление шума.")]
    [Range(0f, 1f)]
    public float derivativeFilterK = 0.7f;

    // ── ILC ─────────────────────────────────────────────────
    [Header("ILC Parameters")]
    [Tooltip("Включить обучение ILC (начинает работать со 2-го круга).")]
    public bool enableILC = true;

    [Tooltip("Скорость обучения. 0.02 — стабильно сходится за 3–5 кругов.")]
    [Range(0.001f, 0.15f)]
    public float learningRate = 0.02f;

    [Tooltip("Максимальная ILC-поправка (рад). Не больше 20% от maxSteering.")]
    public float maxILC = 0.16f;

    // ── Параметры движения ───────────────────────────────────
    [Header("Vehicle Limits")]
    [Tooltip("Максимальный угол руля (рад). 0.7–0.8 для обычного авто.")]
    public float maxSteering = 0.75f;

    [Tooltip("Скорость изменения угла руля (рад/с). Меньше → плавнее.")]
    public float steeringRate = 6.0f;

    [Tooltip("Целевая скорость (м/с). Начни с 3–5, потом увеличивай.")]
    public float targetSpeed = 4.0f;

    [Tooltip("Дистанция предпросмотра (м). ~1–2× скорость. 5–8 м хорошо при 4 м/с.")]
    public float lookaheadDistance = 6.0f;

    [Tooltip("Ускорение/торможение (м/с²).")]
    public float acceleration = 3.0f;

    // ── Отладка ──────────────────────────────────────────────
    [Header("Debug")]
    public bool showILCDebug = true;

    // ── Приватные поля ───────────────────────────────────────
    [HideInInspector] public SingularVehicleModel vehicle;

    private float currentSteering;
    private float integralError;
    private float previousError;
    private float filteredDerivative;

    private List<float> ilcCorrections    = new List<float>();
    private List<float> ilcPreviousErrors = new List<float>();
    private int   ilcIndex   = 0;
    private int   lapCount   = 0;
    private float lapDistance = 0f;
    private Vector2 lastPosition;

    // ─────────────────────────────────────────────────────────
    void Start()
    {
        carVisual = GetComponent<CarVisual>();
        vehicle   = carVisual?.GetVehicleModel();
        if (vehicle == null) { Debug.LogError("SyngularPID_ILC: CarVisual не нашёл модель!"); return; }
        if (trackFollower == null) trackFollower = FindObjectOfType<TrackFollower>();
        if (trackFollower == null) { Debug.LogError("SyngularPID_ILC: TrackFollower не найден!"); return; }

        lastPosition = new Vector2(vehicle.x, vehicle.y);
        ResetController();
    }

    // ─────────────────────────────────────────────────────────
    void FixedUpdate()
    {
        if (vehicle == null || trackFollower == null) return;

        float dt = Mathf.Max(Time.fixedDeltaTime, 0.001f);

        CheckLapCompletion();

        // ── Позиция и трек ──
        Vector2 position = new Vector2(vehicle.x, vehicle.y);
        float   heading  = vehicle.theta;
        float   cte      = trackFollower.GetCurrentCTE();

        int     nearestIdx = trackFollower.trackPath.GetNearestPointIndex(position);
        Vector2 nearest    = trackFollower.trackPath.points[nearestIdx];
        int     nextIdx    = (nearestIdx + 1) % trackFollower.trackPath.points.Length;
        Vector2 next       = trackFollower.trackPath.points[nextIdx];

        Vector2 trackDir    = (next - nearest).normalized;
        Vector2 trackNormal = new Vector2(-trackDir.y, trackDir.x);

        // ── Точка предпросмотра + коррекция по CTE ──
        Vector2 lookahead = GetLookahead(position, nearestIdx, lookaheadDistance);
        Vector2 dir       = (lookahead - position).normalized;

        if      (Mathf.Abs(cte) > 1.5f)
            dir = ((lookahead - position).normalized + (nearest - position).normalized * 2f).normalized;
        else if (Mathf.Abs(cte) > 0.3f)
            dir = (dir + trackNormal * (-cte * 0.4f)).normalized;

        // ── Ошибка курса ──
        float desiredHeading = Mathf.Atan2(dir.y, dir.x);
        float headingError   = Mathf.DeltaAngle(
            heading * Mathf.Rad2Deg,
            desiredHeading * Mathf.Rad2Deg) * Mathf.Deg2Rad;

        // ── PID ──
        float pTerm = kp * headingError;

        // Anti-windup: накапливаем интеграл только когда руль не в упоре
        if (Mathf.Abs(currentSteering) < maxSteering * 0.9f)
        {
            integralError += headingError * dt;
            integralError  = Mathf.Clamp(integralError, -maxIntegral, maxIntegral);
        }
        else
        {
            integralError *= 0.95f;
        }
        float iTerm = ki * integralError;

        float rawD          = (headingError - previousError) / dt;
        filteredDerivative  = Mathf.Lerp(rawD, filteredDerivative, derivativeFilterK);
        float dTerm         = kd * filteredDerivative;
        previousError       = headingError;

        // ── ILC поправка ──
        float ilcCorrection = GetILCCorrection(headingError, cte);

        // ── Итоговое управление ──
        float control     = pTerm + iTerm + dTerm + ilcCorrection;
        float targetSteer = Mathf.Clamp(control, -maxSteering, maxSteering);

        float maxChange    = steeringRate * dt;
        currentSteering   += Mathf.Clamp(targetSteer - currentSteering, -maxChange, maxChange);

        vehicle.v = Mathf.MoveTowards(vehicle.v, targetSpeed, acceleration * dt);
        vehicle.Step(currentSteering, dt);
    }

    // ─────────────────────────────────────────────────────────
    void CheckLapCompletion()
    {
        Vector2 currentPos = new Vector2(vehicle.x, vehicle.y);
        lapDistance += Vector2.Distance(lastPosition, currentPos);
        lastPosition = currentPos;

        float trackLength = GetTrackLength();
        if (lapDistance >= trackLength && ilcIndex > 50)
            CompleteLap();
    }

    void CompleteLap()
    {
        lapCount++;
        lapDistance = 0f;
        ilcIndex    = 0;

        DataRecorder recorder = FindObjectOfType<DataRecorder>();
        if (recorder != null) recorder.OnLapCompleted(lapCount);

        float avgError = 0f;
        if (ilcPreviousErrors.Count > 0)
        {
            foreach (float err in ilcPreviousErrors) avgError += Mathf.Abs(err);
            avgError /= ilcPreviousErrors.Count;
        }
        Debug.Log($"[PID-ILC Syngular] Круг {lapCount} завершён. Средняя ошибка: {avgError:F4} рад");
    }

    // ─────────────────────────────────────────────────────────
    float GetILCCorrection(float currentError, float currentCTE)
    {
        if (!enableILC || lapCount == 0)
        {
            if (ilcIndex >= ilcCorrections.Count)
            {
                ilcCorrections.Add(0f);
                ilcPreviousErrors.Add(currentCTE); // запоминаем CTE
            }
            ilcIndex++;
            return 0f;
        }

        if (ilcIndex < ilcCorrections.Count)
        {
            float prevCTE = ilcPreviousErrors[ilcIndex];
            
            // Учимся убирать поперечное отклонение
            ilcCorrections[ilcIndex] += learningRate * (-prevCTE);
            ilcCorrections[ilcIndex] = Mathf.Clamp(ilcCorrections[ilcIndex], -maxILC, maxILC);
            
            ilcPreviousErrors[ilcIndex] = currentCTE;
            
            float correction = ilcCorrections[ilcIndex];
            ilcIndex++;
            return correction;
        }

        ilcIndex++;
        return 0f;
    }

    // ─────────────────────────────────────────────────────────
    float GetTrackLength()
    {
        if (trackFollower == null || trackFollower.trackPath == null ||
            trackFollower.trackPath.points == null ||
            trackFollower.trackPath.points.Length < 2) return 1000f;

        float length = 0f;
        var pts = trackFollower.trackPath.points;
        for (int i = 0; i < pts.Length; i++)
            length += Vector2.Distance(pts[i], pts[(i + 1) % pts.Length]);
        return length;
    }

    Vector2 GetLookahead(Vector2 pos, int startIdx, float dist)
    {
        float accumulated = 0f;
        int   idx = startIdx;
        var   pts = trackFollower.trackPath.points;

        for (int i = 0; i < 300; i++)
        {
            int     next = (idx + 1) % pts.Length;
            float   seg  = Vector2.Distance(pts[idx], pts[next]);
            if (seg < 0.001f) { idx = next; continue; }

            if (accumulated + seg >= dist)
                return Vector2.Lerp(pts[idx], pts[next], (dist - accumulated) / seg);

            accumulated += seg;
            idx = next;
        }
        return pts[idx];
    }

    // ─────────────────────────────────────────────────────────
    void OnGUI()
    {
        if (!showILCDebug) return;

        GUILayout.BeginArea(new Rect(10, 10, 280, 210));
        GUILayout.Box("=== PID-ILC Syngular System ===");
        GUILayout.Label($"Lap:           {lapCount + 1}");
        GUILayout.Label($"ILC index:     {ilcIndex}");
        GUILayout.Label($"ILC points:      {ilcPreviousErrors.Count}");
        GUILayout.Label($"Distance:      {lapDistance:F1} м");
        GUILayout.Label($"Velocity:       {(vehicle != null ? vehicle.v : 0f):F1} м/с");
        GUILayout.Label($"CTE:            {(trackFollower != null ? trackFollower.GetCurrentCTE() : 0f):F3} м");
        GUILayout.Label($"Steering:           {currentSteering:F3} рад");

        if (lapCount > 0 && ilcCorrections.Count > 0)
        {
            float maxCorr = 0f;
            foreach (float c in ilcCorrections)
                if (Mathf.Abs(c) > maxCorr) maxCorr = Mathf.Abs(c);
            GUILayout.Label($"Макс ILC корр: {maxCorr:F4} рад");
        }
        GUILayout.EndArea();
    }

    // ─────────────────────────────────────────────────────────
    public void ResetController()
    {
        currentSteering    = 0f;
        integralError      = 0f;
        previousError      = 0f;
        filteredDerivative = 0f;
        ilcIndex           = 0;
        lapCount           = 0;
        lapDistance        = 0f;
        ilcPreviousErrors?.Clear();
        ilcCorrections?.Clear();
        if (vehicle != null) lastPosition = new Vector2(vehicle.x, vehicle.y);
    }

    public void ResetLapIndex()
    {
        ilcIndex    = 0;
        lapDistance = 0f;
        lastPosition = new Vector2(vehicle != null ? vehicle.x : 0,
                                   vehicle != null ? vehicle.y : 0);
    }

    // ─── Getters для DataRecorder ────────────────────────────
    #region DataRecorder Getters
    public SingularVehicleModel GetVehicleModel()   => vehicle;
    public float GetCurrentSteering()               => currentSteering;
    public float GetPTerm()                         => kp * previousError;
    public float GetITerm()                         => ki * integralError;
    public float GetDTerm()                         => kd * filteredDerivative;
    public float GetIntegralError()                 => integralError;
    public float GetHeadingError()                  => previousError;
    public int   GetILCIndex()                      => ilcIndex;
    public float GetCurrentCurvature()              => 0f;

    public float GetILCCorrection()
    {
        if (enableILC && ilcIndex > 0 && ilcIndex <= ilcCorrections.Count)
            return ilcCorrections[ilcIndex - 1];
        return 0f;
    }

    public float GetCrossTrackError() =>
        trackFollower != null ? trackFollower.GetCurrentCTE() : 0f;

    public int GetNearestTrackIndex()
    {
        if (trackFollower == null || vehicle == null) return 0;
        return trackFollower.trackPath.GetNearestPointIndex(new Vector2(vehicle.x, vehicle.y));
    }
    #endregion
}