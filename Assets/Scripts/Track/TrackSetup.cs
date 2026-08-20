using UnityEngine;

[RequireComponent(typeof(TrackPath))]
public class TrackSetup : MonoBehaviour
{
    [Header("Track Generation")]
    [Tooltip("Автоматически генерировать трассу при старте если points пустые")]
    public bool generateOnAwake = true;

    [Tooltip("Радиус базовой окружности (м)")]
    public float trackRadius = 30f;

    [Tooltip("Количество опорных точек. 60–80 для плавной кривой.")]
    public int pointCount = 64;

    [Tooltip("Амплитуда случайных отклонений (м). 0 = идеальный круг.")]
    public float noiseAmount = 4f;

    private TrackPath trackPath;

    void Awake()
    {
        trackPath = GetComponent<TrackPath>();
        if (generateOnAwake && (trackPath.points == null || trackPath.points.Length == 0))
            GenerateTrack();
    }

    [ContextMenu("Generate Track")]
    public void GenerateTrack()
    {
        if (trackPath == null) trackPath = GetComponent<TrackPath>();

        Vector2[] pts = new Vector2[pointCount];
        for (int i = 0; i < pointCount; i++)
        {
            float angle  = (float)i / pointCount * Mathf.PI * 2f;
            float radius = trackRadius + Random.Range(-noiseAmount, noiseAmount);
            pts[i] = new Vector2(Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius);
        }

        // Сглаживание (3 прохода)
        for (int pass = 0; pass < 3; pass++)
            for (int i = 0; i < pointCount; i++)
                pts[i] = (pts[(i - 1 + pointCount) % pointCount] + pts[i] * 2 + pts[(i + 1) % pointCount]) / 4f;

        trackPath.points = pts;
        Debug.Log($"[TrackSetup] Сгенерирована трасса: {pointCount} точек, радиус ~{trackRadius} м");
    }

    void OnDrawGizmos()
    {
        if (trackPath == null || trackPath.points == null) return;
        Gizmos.color = Color.cyan;
        for (int i = 0; i < trackPath.points.Length; i++)
        {
            Vector3 p = new Vector3(trackPath.points[i].x, 0, trackPath.points[i].y);
            Vector3 n = new Vector3(trackPath.points[(i + 1) % trackPath.points.Length].x, 0,
                                    trackPath.points[(i + 1) % trackPath.points.Length].y);
            Gizmos.DrawLine(p, n);
        }
    }
}