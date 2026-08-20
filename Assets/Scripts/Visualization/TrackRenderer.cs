using UnityEngine;
[RequireComponent(typeof(LineRenderer))]
public class TrackRenderer : MonoBehaviour
{
    public TrackPath trackPath;

    [Header("Visual")]
    public float trackWidth      = 1.0f;
    public Color centerLineColor = Color.yellow;
    public Color leftBorderColor = Color.red;
    public Color rightBorderColor= Color.blue;

    [Header("Quality")]
    public int  pointsPerSegment = 20;
    public bool showCenterLine   = true;
    public bool showBorders      = true;
    public bool showGizmos       = true;
    public Color gizmoColor      = Color.white;

    private LineRenderer centerLineRenderer;
    private LineRenderer leftBorderRenderer;
    private LineRenderer rightBorderRenderer;

    void Start()
    {
        if (trackPath == null) trackPath = FindObjectOfType<TrackPath>();
        SetupRenderers();
        GenerateTrackVisuals();
    }

    void SetupRenderers()
    {
        if (showCenterLine)
        {
            centerLineRenderer = GetComponent<LineRenderer>();
            if (centerLineRenderer == null) centerLineRenderer = gameObject.AddComponent<LineRenderer>();
            SetupRenderer(centerLineRenderer, 0.1f, centerLineColor);
        }

        if (showBorders)
        {
            leftBorderRenderer  = CreateChildRenderer("LeftBorder",  0.08f, leftBorderColor);
            rightBorderRenderer = CreateChildRenderer("RightBorder", 0.08f, rightBorderColor);
        }
    }

    LineRenderer CreateChildRenderer(string name, float width, Color color)
    {
        GameObject obj = new GameObject(name);
        obj.transform.parent = transform;
        LineRenderer lr = obj.AddComponent<LineRenderer>();
        SetupRenderer(lr, width, color);
        return lr;
    }

    void SetupRenderer(LineRenderer lr, float width, Color color)
    {
        lr.startWidth = width; lr.endWidth = width;
        lr.material   = new Material(Shader.Find("Sprites/Default"));
        lr.startColor = color; lr.endColor = color;
    }

    void GenerateTrackVisuals()
    {
        if (trackPath == null || trackPath.points == null || trackPath.points.Length < 2) return;

        Vector3[] smooth = GenerateSmoothedTrack();

        if (showCenterLine && centerLineRenderer != null)
        {
            centerLineRenderer.positionCount = smooth.Length;
            centerLineRenderer.SetPositions(smooth);
        }

        if (showBorders)
        {
            Vector3[] left  = new Vector3[smooth.Length];
            Vector3[] right = new Vector3[smooth.Length];

            for (int i = 0; i < smooth.Length; i++)
            {
                Vector2 dir    = GetTrackDir(i, smooth);
                Vector2 norm   = new Vector2(-dir.y, dir.x);
                left[i]  = smooth[i] + new Vector3( norm.x, 0,  norm.y) * trackWidth;
                right[i] = smooth[i] + new Vector3(-norm.x, 0, -norm.y) * trackWidth;
            }

            if (leftBorderRenderer  != null) { leftBorderRenderer.positionCount  = left.Length;  leftBorderRenderer.SetPositions(left);   }
            if (rightBorderRenderer != null) { rightBorderRenderer.positionCount = right.Length; rightBorderRenderer.SetPositions(right); }
        }
    }

    Vector3[] GenerateSmoothedTrack()
    {
        int      total   = trackPath.points.Length * pointsPerSegment;
        Vector3[] smooth = new Vector3[total];

        for (int i = 0; i < trackPath.points.Length; i++)
        {
            Vector2 p0 = trackPath.points[i];
            Vector2 p1 = trackPath.points[(i + 1) % trackPath.points.Length];
            Vector2 p2 = trackPath.points[(i + 2) % trackPath.points.Length];
            Vector2 p3 = trackPath.points[(i + 3) % trackPath.points.Length];

            for (int j = 0; j < pointsPerSegment; j++)
            {
                float   t   = (float)j / pointsPerSegment;
                Vector2 pt  = CatmullRom(p0, p1, p2, p3, t);
                int     idx = i * pointsPerSegment + j;
                if (idx < total) smooth[idx] = new Vector3(pt.x, 0, pt.y);
            }
        }
        return smooth;
    }

    Vector2 CatmullRom(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3, float t)
    {
        float t2 = t * t, t3 = t2 * t;
        return 0.5f * ((2 * p1) + (-p0 + p2) * t +
               (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 +
               (-p0 + 3 * p1 - 3 * p2 + p3) * t3);
    }

    Vector2 GetTrackDir(int idx, Vector3[] pts)
    {
        int     nxt = Mathf.Min(idx + 1, pts.Length - 1);
        Vector2 a   = new Vector2(pts[idx].x, pts[idx].z);
        Vector2 b   = new Vector2(pts[nxt].x, pts[nxt].z);
        return (b - a).normalized;
    }

    void OnDrawGizmos()
    {
        if (!showGizmos || trackPath == null || trackPath.points == null) return;
        Gizmos.color = gizmoColor;
        for (int i = 0; i < trackPath.points.Length; i++)
        {
            Vector3 p = new Vector3(trackPath.points[i].x, 0, trackPath.points[i].y);
            Gizmos.DrawSphere(p, 0.2f);
            Vector3 n = new Vector3(trackPath.points[(i + 1) % trackPath.points.Length].x, 0,
                                    trackPath.points[(i + 1) % trackPath.points.Length].y);
            Gizmos.DrawLine(p, n);
        }
    }
}