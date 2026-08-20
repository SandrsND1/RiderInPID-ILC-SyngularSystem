using UnityEngine;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Globalization;

// ================================================================
//  DataRecorder — расширенная запись для подбора параметров PID-ILC
//  Версия 2.0
// ================================================================
public class DataRecorder : MonoBehaviour
{
    [Header("References")]
    public SingularPID_ILC controller;

    [Header("Recording")]
    public bool  autoRecord         = true;
    [Range(1, 50)]
    public int   recordingFrequency = 20;   // 20 Гц — достаточно для анализа
    public float maxRecordTime      = 600f;

    [Header("What to Record")]
    public bool recordPID          = true;
    public bool recordILC          = true;
    public bool recordVehicleState = true;
    public bool recordErrors       = true;
    public bool recordTrackInfo    = true;
    public bool recordDerivatives  = true;  // скорость изменения ошибок
    public bool recordSaturation   = true;  // насыщение руля

    [Header("Export")]
    public bool useSemicolon  = true;
    public bool useDotDecimal = true;

    // ── Параметры сессии (для записи в header) ──────────────────
    [Header("Session Parameters (auto-filled)")]
    [Tooltip("Заполняется автоматически из контроллера при старте")]
    public float session_kp, session_ki, session_kd;
    public float session_lookahead, session_maxSteering, session_steeringRate;
    public float session_targetSpeed, session_learningRate, session_maxILC;
    public float session_derivativeFilterK;

    // ── Структура точки данных ───────────────────────────────────
    [System.Serializable]
    public struct DataPoint
    {
        // Время
        public float time;
        public int   lapNumber;
        public int   ilcIndex;

        // PID компоненты
        public float pTerm, iTerm, dTerm;
        public float integralError;
        public float steeringOutput;
        public bool  steeringSaturated;   // |steering| >= maxSteering*0.98

        // ILC
        public float ilcCorrection;
        public float ilcShare;            // доля ILC в итоговом управлении (%)

        // Состояние машины
        public float vehicleX, vehicleY;
        public float heading, speed;
        public float steeringAngle;

        // Ошибки
        public float headingError;
        public float crossTrackError;
        public float absCTE;

        // Производные ошибок
        public float dCTE_dt;            // скорость изменения CTE (м/с)
        public float dHeadingError_dt;   // скорость изменения ошибки курса (рад/с)

        // Трек
        public float curvature;
        public int   nearestTrackIndex;
        public float distanceAlongTrack; // пройденное расстояние по треку (м)
    }

    // ── Статистика круга ─────────────────────────────────────────
    private struct LapStats
    {
        public int   lapNum;
        public float duration;
        public float avgCTE, maxCTE, minCTE;
        public float avgAbsHeadingError;
        public float avgAbsPTerm, avgAbsITerm, avgAbsDTerm;
        public float avgAbsILC;
        public float saturationRatio;    // доля точек с насыщением [0-1]
        public float ilcActiveRatio;     // доля точек где ILC != 0
        public float avgSpeed;
        public int   dataPoints;
        public float rmsHeadingError;
        public float rmsCTE;
    }

    // ── Приватные поля ───────────────────────────────────────────
    private List<DataPoint> dataPoints = new List<DataPoint>(2000);
    private List<LapStats>  lapHistory = new List<LapStats>();

    private float  recordTimer, lastRecordTime;
    private bool   isRecording;
    private string filePath, sessionFolder;

    // Для производных
    private float prevCTE, prevHeadingError;
    private float prevRecordTime;
    private float distanceAlongTrack;
    private Vector2 prevPosition;
    private int   currentLap;

    // Накопители для статистики
    private float sumCTE, sumAbsCTE, maxCTELap, minCTELap;
    private float sumHE, sumAbsP, sumAbsI, sumAbsD, sumAbsILC, sumSpeed;
    private int   satCount, ilcActiveCount, cteCount;
    private float lapStartTime;
    private float sumSqHE, sumSqCTE;

    // ─────────────────────────────────────────────────────────────
    void Start()
    {
        if (controller == null) controller = FindObjectOfType<SingularPID_ILC>();
        if (controller == null) { Debug.LogError("DataRecorder: контроллер не найден!"); return; }

        ReadSessionParams();

        sessionFolder = Application.dataPath + "/DataRecordings/";
        if (!Directory.Exists(sessionFolder)) Directory.CreateDirectory(sessionFolder);

        string timestamp = System.DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
        filePath = sessionFolder + "PID_ILC_" + timestamp + ".csv";

        if (autoRecord) StartRecording();
    }

    void ReadSessionParams()
    {
        session_kp               = controller.kp;
        session_ki               = controller.ki;
        session_kd               = controller.kd;
        session_lookahead        = controller.lookaheadDistance;
        session_maxSteering      = controller.maxSteering;
        session_steeringRate     = controller.steeringRate;
        session_targetSpeed      = controller.targetSpeed;
        session_learningRate     = controller.learningRate;
        session_maxILC           = controller.maxILC;
        session_derivativeFilterK= controller.derivativeFilterK;
    }

    // ─────────────────────────────────────────────────────────────
    void Update()
    {
        if (!isRecording) return;
        recordTimer += Time.deltaTime;
        if (recordTimer >= maxRecordTime) { StopRecording(); return; }

        float interval = 1f / recordingFrequency;
        if (Time.time - lastRecordTime >= interval)
        {
            RecordDataPoint(Time.time - lastRecordTime);
            lastRecordTime = Time.time;
        }
    }

    // ─────────────────────────────────────────────────────────────
    void RecordDataPoint(float dt)
    {
        if (controller == null) return;
        var v = controller.GetVehicleModel();
        if (v == null) return;

        dt = Mathf.Max(dt, 0.001f);

        var p = new DataPoint();
        p.time      = recordTimer;
        p.lapNumber = currentLap;
        p.ilcIndex  = controller.GetILCIndex();

        // ── Состояние машины
        p.vehicleX     = v.x;
        p.vehicleY     = v.y;
        p.heading      = v.theta;
        p.speed        = v.v;
        p.steeringAngle= controller.GetCurrentSteering();

        // ── Расстояние вдоль трека
        Vector2 curPos = new Vector2(v.x, v.y);
        distanceAlongTrack += Vector2.Distance(curPos, prevPosition);
        prevPosition = curPos;
        p.distanceAlongTrack = distanceAlongTrack;

        // ── PID
        p.pTerm         = controller.GetPTerm();
        p.iTerm         = controller.GetITerm();
        p.dTerm         = controller.GetDTerm();
        p.integralError = controller.GetIntegralError();
        p.steeringOutput= controller.GetCurrentSteering();
        p.steeringSaturated = Mathf.Abs(p.steeringOutput) >= session_maxSteering * 0.98f;

        // ── ILC
        p.ilcCorrection = controller.GetILCCorrection();
        float totalCtrl = Mathf.Abs(p.steeringOutput);
        p.ilcShare = totalCtrl > 0.001f
            ? Mathf.Abs(p.ilcCorrection) / totalCtrl * 100f
            : 0f;

        // ── Ошибки
        p.headingError    = controller.GetHeadingError();
        p.crossTrackError = controller.GetCrossTrackError();
        p.absCTE          = Mathf.Abs(p.crossTrackError);

        // ── Производные ошибок
        p.dCTE_dt           = (p.crossTrackError - prevCTE)          / dt;
        p.dHeadingError_dt  = (p.headingError    - prevHeadingError)  / dt;
        prevCTE             = p.crossTrackError;
        prevHeadingError    = p.headingError;

        // ── Трек
        p.curvature         = controller.GetCurrentCurvature();
        p.nearestTrackIndex = controller.GetNearestTrackIndex();

        dataPoints.Add(p);

        // ── Накопление статистики
        sumCTE    += p.crossTrackError;
        sumAbsCTE += p.absCTE;
        sumSqCTE  += p.absCTE * p.absCTE;
        if (p.absCTE > maxCTELap) maxCTELap = p.absCTE;
        if (p.absCTE < minCTELap) minCTELap = p.absCTE;

        sumHE     += Mathf.Abs(p.headingError);
        sumSqHE   += p.headingError * p.headingError;
        sumAbsP   += Mathf.Abs(p.pTerm);
        sumAbsI   += Mathf.Abs(p.iTerm);
        sumAbsD   += Mathf.Abs(p.dTerm);
        sumAbsILC += Mathf.Abs(p.ilcCorrection);
        sumSpeed  += p.speed;

        if (p.steeringSaturated) satCount++;
        if (Mathf.Abs(p.ilcCorrection) > 0.001f) ilcActiveCount++;
        cteCount++;
    }

    // ─────────────────────────────────────────────────────────────
    public void StartRecording()
    {
        dataPoints.Clear();
        lapHistory.Clear();
        recordTimer = lastRecordTime = 0f;
        isRecording = true;
        currentLap  = 0;
        ResetLapAccumulators();
        lapStartTime = 0f;

        var v = controller?.GetVehicleModel();
        prevPosition = v != null ? new Vector2(v.x, v.y) : Vector2.zero;
        distanceAlongTrack = 0f;

        Debug.Log($"DataRecorder: запись начата → {filePath}");
    }

    public void StopRecording()
    {
        isRecording = false;
        FinalizeLapStats(currentLap);
        SaveMainFile();
        SaveLapSummary();
    }

    // ─────────────────────────────────────────────────────────────
    public void OnLapCompleted(int lapNum)
    {
        currentLap = lapNum;
        FinalizeLapStats(lapNum);

        // Сохраняем CSV этого круга
        string lapFile = filePath.Replace(".csv", $"_Lap{lapNum}.csv");
        SaveToFile(lapFile, dataPoints);
        Debug.Log($"DataRecorder: круг {lapNum} сохранён → {lapFile}");

        dataPoints.Clear();
        ResetLapAccumulators();
        lapStartTime = recordTimer;
        distanceAlongTrack = 0f;
    }

    // ─────────────────────────────────────────────────────────────
    void FinalizeLapStats(int lapNum)
    {
        if (cteCount == 0) return;
        float n = cteCount;

        var stats = new LapStats
        {
            lapNum              = lapNum,
            duration            = recordTimer - lapStartTime,
            avgCTE              = sumAbsCTE / n,
            maxCTE              = maxCTELap,
            minCTE              = minCTELap < float.MaxValue ? minCTELap : 0f,
            avgAbsHeadingError  = sumHE   / n,
            avgAbsPTerm         = sumAbsP / n,
            avgAbsITerm         = sumAbsI / n,
            avgAbsDTerm         = sumAbsD / n,
            avgAbsILC           = sumAbsILC / n,
            saturationRatio     = satCount / n,
            ilcActiveRatio      = ilcActiveCount / n,
            avgSpeed            = sumSpeed / n,
            dataPoints          = cteCount,
            rmsHeadingError     = Mathf.Sqrt(sumSqHE / n),
            rmsCTE              = Mathf.Sqrt(sumSqCTE / n)
        };
        lapHistory.Add(stats);
    }

    void ResetLapAccumulators()
    {
        sumCTE = sumAbsCTE = maxCTELap = sumHE = 0f;
        sumAbsP = sumAbsI = sumAbsD = sumAbsILC = sumSpeed = 0f;
        sumSqHE = sumSqCTE = 0f;
        satCount = ilcActiveCount = cteCount = 0;
        minCTELap = float.MaxValue;
    }

    // ─────────────────────────────────────────────────────────────
    void SaveMainFile()
    {
        // Главный файл — все данные сессии объединены
        SaveToFile(filePath, dataPoints);
    }

    void SaveLapSummary()
    {
        if (lapHistory.Count == 0) return;
        string sep = useSemicolon ? ";" : ",";
        string summaryPath = filePath.Replace(".csv", "_LapSummary.csv");

        var sb = new StringBuilder();

        // ── Параметры сессии
        sb.AppendLine("=== SESSION PARAMETERS ===");
        sb.AppendLine($"Kp{sep}{F(session_kp)}");
        sb.AppendLine($"Ki{sep}{F(session_ki)}");
        sb.AppendLine($"Kd{sep}{F(session_kd)}");
        sb.AppendLine($"LookaheadDistance{sep}{F(session_lookahead)}");
        sb.AppendLine($"MaxSteering{sep}{F(session_maxSteering)}");
        sb.AppendLine($"SteeringRate{sep}{F(session_steeringRate)}");
        sb.AppendLine($"TargetSpeed{sep}{F(session_targetSpeed)}");
        sb.AppendLine($"LearningRate{sep}{F(session_learningRate)}");
        sb.AppendLine($"MaxILC{sep}{F(session_maxILC)}");
        sb.AppendLine($"DerivativeFilterK{sep}{F(session_derivativeFilterK)}");
        sb.AppendLine();

        // ── Заголовок таблицы кругов
        sb.AppendLine(string.Join(sep, new[]{
            "Lap","Duration_s","AvgCTE_m","MaxCTE_m","MinCTE_m","RMS_CTE",
            "AvgHE_rad","RMS_HE",
            "AvgP","AvgI","AvgD","AvgILC",
            "SaturationPct","ILC_ActivePct","AvgSpeed_ms","DataPoints"
        }));

        foreach (var s in lapHistory)
            sb.AppendLine(string.Join(sep, new[]{
                s.lapNum.ToString(),
                F(s.duration),
                F(s.avgCTE), F(s.maxCTE), F(s.minCTE), F(s.rmsCTE),
                F(s.avgAbsHeadingError), F(s.rmsHeadingError),
                F(s.avgAbsPTerm), F(s.avgAbsITerm), F(s.avgAbsDTerm), F(s.avgAbsILC),
                F(s.saturationRatio * 100f), F(s.ilcActiveRatio * 100f),
                F(s.avgSpeed), s.dataPoints.ToString()
            }));

        // ── Итог сессии
        if (lapHistory.Count > 1)
        {
            float firstCTE = lapHistory[0].avgCTE;
            float bestCTE  = float.MaxValue;
            int   bestLap  = 0;
            foreach (var s in lapHistory)
                if (s.avgCTE < bestCTE) { bestCTE = s.avgCTE; bestLap = s.lapNum; }

            sb.AppendLine();
            sb.AppendLine($"=== SESSION SUMMARY{sep}===");
            sb.AppendLine($"TotalLaps{sep}{lapHistory.Count}");
            sb.AppendLine($"FirstLap_AvgCTE{sep}{F(firstCTE)}");
            sb.AppendLine($"BestLap{sep}{bestLap}");
            sb.AppendLine($"BestLap_AvgCTE{sep}{F(bestCTE)}");
            sb.AppendLine($"ILC_Improvement_Pct{sep}{F((firstCTE - bestCTE) / firstCTE * 100f)}");
        }

        File.WriteAllText(summaryPath, sb.ToString(), new UTF8Encoding(true));
        Debug.Log($"DataRecorder: сводка сохранена → {summaryPath}");
    }

    // ─────────────────────────────────────────────────────────────
    void SaveToFile(string path, List<DataPoint> pts)
    {
        if (pts.Count == 0) return;
        string sep = useSemicolon ? ";" : ",";
        var sb = new StringBuilder();

        sb.AppendLine(string.Join(sep, new[]{
            "Time","Lap","ILC_Index",
            "P_Term","I_Term","D_Term","Integral","Steering","Saturated","ILC_Correction","ILC_Share_Pct",
            "X","Y","Heading","Speed","SteeringAngle",
            "HeadingError","CTE","AbsCTE","dCTE_dt","dHE_dt",
            "Curvature","TrackIndex","DistAlongTrack"
        }));

        foreach (var pt in pts)
            sb.AppendLine(string.Join(sep, new[]{
                F(pt.time), pt.lapNumber.ToString(), pt.ilcIndex.ToString(),
                F(pt.pTerm), F(pt.iTerm), F(pt.dTerm), F(pt.integralError),
                F(pt.steeringOutput), pt.steeringSaturated ? "1" : "0",
                F(pt.ilcCorrection), F(pt.ilcShare),
                F(pt.vehicleX), F(pt.vehicleY), F(pt.heading), F(pt.speed), F(pt.steeringAngle),
                F(pt.headingError), F(pt.crossTrackError), F(pt.absCTE),
                F(pt.dCTE_dt), F(pt.dHeadingError_dt),
                F(pt.curvature), pt.nearestTrackIndex.ToString(), F(pt.distanceAlongTrack)
            }));

        // Summary в конце каждого файла круга
        if (cteCount > 0)
        {
            sb.AppendLine();
            sb.AppendLine($"=== SUMMARY{sep}===");
            sb.AppendLine($"Duration (s){sep}{F(recordTimer - lapStartTime)}");
            sb.AppendLine($"Max CTE (m){sep}{F(maxCTELap)}");
            sb.AppendLine($"Avg CTE (m){sep}{F(cteCount > 0 ? sumAbsCTE / cteCount : 0)}");
            sb.AppendLine($"RMS CTE (m){sep}{F(Mathf.Sqrt(sumSqCTE / cteCount))}");
            sb.AppendLine($"Saturation%{sep}{F((float)satCount / cteCount * 100f)}");
            sb.AppendLine($"Data Points{sep}{pts.Count}");
        }

        File.WriteAllText(path, sb.ToString(), new UTF8Encoding(true));
    }

    string F(float v) => useDotDecimal
        ? v.ToString("F4", CultureInfo.InvariantCulture)
        : v.ToString("F4", new CultureInfo("ru-RU"));

    void OnApplicationQuit() { if (isRecording) StopRecording(); }
    void OnDestroy()         { if (isRecording) StopRecording(); }

    // ── Getters
    public float GetAverageCTE()    => cteCount > 0 ? sumAbsCTE / cteCount : 0f;
    public float GetMaxCTE()        => maxCTELap;
    public float GetRmsCTE()        => cteCount > 0 ? Mathf.Sqrt(sumSqCTE / cteCount) : 0f;
    public int   GetDataPointCount()=> dataPoints.Count;
    public float GetRecordingTime() => recordTimer;
    public float GetSaturationPct() => cteCount > 0 ? (float)satCount / cteCount * 100f : 0f;
}
