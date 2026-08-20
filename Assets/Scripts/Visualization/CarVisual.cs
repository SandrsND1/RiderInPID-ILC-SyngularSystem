using UnityEngine;

public class CarVisual : MonoBehaviour
{
    [Header("Vehicle Setup")]
    public SingularVehicleModel model;
    public float visualUpdateSpeed = 10f;

    [Header("Initial State")]
    public Vector2 startPosition = new Vector2(0, 0);
    [Tooltip("Начальный угол (градусы)")]
    public float startRotationDeg = 0f;

    void Awake()
    {
        if (model == null)
        {
            model         = new SingularVehicleModel();
            model.x       = startPosition.x;
            model.y       = startPosition.y;
            model.theta   = startRotationDeg * Mathf.Deg2Rad;
        }
    }

    void Update()
    {
        if (model == null) return;
        Vector3 targetPos = new Vector3(model.x, 0, model.y);
        Quaternion targetRot = Quaternion.Euler(0, -model.theta * Mathf.Rad2Deg, 0);

        transform.position = Vector3.Lerp(transform.position, targetPos,
                                        visualUpdateSpeed * Time.deltaTime);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRot,
                                            visualUpdateSpeed * Time.deltaTime);
    }

    public SingularVehicleModel GetVehicleModel() => model;
}