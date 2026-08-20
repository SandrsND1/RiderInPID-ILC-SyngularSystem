using UnityEngine;
public class TrackPath : MonoBehaviour
{
    public Vector2[] points;
    private float[]  angles;

    void Awake() => PrecomputeAngles();

    void PrecomputeAngles()
    {
        if (points == null || points.Length == 0) return;
        angles = new float[points.Length];
        for (int i = 0; i < points.Length; i++)
        {
            Vector2 dir = (points[(i + 1) % points.Length] - points[i]).normalized;
            angles[i] = Mathf.Atan2(dir.y, dir.x);
        }
    }

    public int GetNearestPointIndex(Vector2 pos)
    {
        int   best     = 0;
        float bestDist = float.MaxValue;
        for (int i = 0; i < points.Length; i++)
        {
            float d = Vector2.Distance(pos, points[i]);
            if (d < bestDist) { bestDist = d; best = i; }
        }
        return best;
    }

    public float GetAngleAtIndex(int idx) => angles != null ? angles[idx] : 0f;

    void OnDrawGizmos()
    {
        if (points == null) return;
        Gizmos.color = Color.yellow;
        for (int i = 0; i < points.Length; i++)
        {
            Gizmos.DrawSphere(new Vector3(points[i].x, 0, points[i].y), 0.15f);
            Vector2 next = points[(i + 1) % points.Length];
            Gizmos.DrawLine(
                new Vector3(points[i].x, 0, points[i].y),
                new Vector3(next.x,      0, next.y));
        }
    }
}