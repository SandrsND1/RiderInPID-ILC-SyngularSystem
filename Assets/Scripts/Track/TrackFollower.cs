using UnityEngine;

/// <summary>
/// Отслеживает положение машины на трассе
/// Вычисляет поперечную ошибку (CTE) и точку предпросмотра
/// </summary>
public class TrackFollower : MonoBehaviour
{
    [Header("References")]
    public TrackPath  trackPath;
    public CarVisual  carVisual;

    [Header("Lookahead")]
    [Tooltip("Базовая дистанция предпросмотра (м). Перекрывается контроллером.")]
    public float lookaheadDistance = 6f;
    public int   maxSearchSegments = 30;

    [Header("Debug")]
    public bool  showDebugLines   = true;
    public Color lookaheadColor   = Color.green;
    public Color directionColor   = Color.red;

    private SingularVehicleModel vehicle;
    private float   currentCTE;
    private Vector2 targetPoint;
    private float   targetAngle;
    private int     nearestIndex;

    void Start()
    {
        if (trackPath  == null) trackPath  = FindObjectOfType<TrackPath>();
        if (carVisual  == null) carVisual  = FindObjectOfType<CarVisual>();
        if (carVisual  != null) vehicle    = carVisual.GetVehicleModel();

        if (trackPath == null) Debug.LogError("TrackFollower: TrackPath не найден!");
        if (vehicle   == null) Debug.LogError("TrackFollower: модель машины не найдена!");
    }

    void FixedUpdate()
    {
        if (trackPath == null || vehicle == null ||
            trackPath.points == null || trackPath.points.Length < 2) return;

        Vector2 carPos = new Vector2(vehicle.x, vehicle.y);
        nearestIndex   = trackPath.GetNearestPointIndex(carPos);
        targetPoint    = CalculateLookaheadPoint(carPos, nearestIndex, lookaheadDistance);
        currentCTE     = CalculateCrossTrackError(carPos, nearestIndex);

        Vector2 toTarget = (targetPoint - carPos).normalized;
        targetAngle = Mathf.Atan2(toTarget.y, toTarget.x);

        if (showDebugLines)
        {
            float h = 0.5f;
            Debug.DrawLine(new Vector3(carPos.x, h, carPos.y),
                           new Vector3(targetPoint.x, h, targetPoint.y), lookaheadColor);
            Debug.DrawRay(new Vector3(carPos.x, h, carPos.y),
                          new Vector3(toTarget.x, 0, toTarget.y) * 3f, directionColor);
        }
    }

    public Vector2 CalculateLookaheadPoint(Vector2 carPos, int startIdx, float dist)
    {
        float accum = 0f;
        int   idx   = startIdx;

        for (int i = 0; i < maxSearchSegments; i++)
        {
            int     nxt = (idx + 1) % trackPath.points.Length;
            float   seg = Vector2.Distance(trackPath.points[idx], trackPath.points[nxt]);
            if (seg < 0.001f) { idx = nxt; continue; }

            if (accum + seg >= dist)
                return Vector2.Lerp(trackPath.points[idx], trackPath.points[nxt],
                                    Mathf.Clamp01((dist - accum) / seg));
            accum += seg;
            idx    = nxt;
        }
        return trackPath.points[(startIdx + maxSearchSegments) % trackPath.points.Length];
    }

    float CalculateCrossTrackError(Vector2 carPos, int nearIdx)
    {
        int     nxtIdx = (nearIdx + 1) % trackPath.points.Length;
        Vector2 A      = trackPath.points[nearIdx];
        Vector2 B      = trackPath.points[nxtIdx];
        Vector2 AB     = B - A;
        float   segLen = AB.magnitude;

        if (segLen < 0.001f) return Vector2.Distance(carPos, A);

        float   proj      = Mathf.Clamp01(Vector2.Dot(carPos - A, AB) / (segLen * segLen));
        Vector2 closest   = A + proj * AB;
        float   dist      = Vector2.Distance(carPos, closest);
        Vector2 trackNorm = new Vector2(-AB.normalized.y, AB.normalized.x);
        float   side      = Vector2.Dot(carPos - closest, trackNorm);

        return side >= 0 ? dist : -dist;
    }

    public float   GetCurrentCTE()    => currentCTE;
    public Vector2 GetTargetPoint()   => targetPoint;
    public float   GetTargetAngle()   => targetAngle;
    public int     GetNearestIndex()  => nearestIndex;
}