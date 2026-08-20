using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    [Header("Target")]
    public Transform  target;
    public CarVisual  carVisual;

    [Header("Position")]
    public float distanceBehind  = 8f;
    public float height          = 5f;
    public float lateralOffset   = 0f;

    [Header("Smoothness")]
    [Range(0.01f, 1f)]  public float positionSmoothness = 0.15f;
    [Range(0.1f,  10f)] public float rotationSmoothness = 5f;

    [Header("Look")]
    public float lookAheadDistance = 10f;
    public float lookAtHeight      = 0.5f;

    private Vector3 currentVelocity;
    private SingularVehicleModel vehicle;

    void Start()
    {
        if (target    == null) { carVisual = FindObjectOfType<CarVisual>(); if (carVisual) target = carVisual.transform; }
        if (carVisual == null)   carVisual  = target?.GetComponent<CarVisual>();
        if (carVisual != null)   vehicle    = carVisual.GetVehicleModel();
    }

    void LateUpdate()
    {
        if (target == null) return;
        Vector3 carFwd = target.forward;
        if (vehicle != null)
            carFwd = new Vector3(Mathf.Cos(vehicle.theta), 0, Mathf.Sin(vehicle.theta));

        Vector3 desired = target.position - carFwd * distanceBehind + Vector3.up * height
                        + Vector3.Cross(Vector3.up, carFwd).normalized * lateralOffset;

        transform.position = Vector3.SmoothDamp(transform.position, desired, ref currentVelocity, positionSmoothness);

        Vector3 lookAt = target.position + carFwd * lookAheadDistance;
        lookAt.y = target.position.y + lookAtHeight;
        transform.rotation = Quaternion.Slerp(transform.rotation,
            Quaternion.LookRotation(lookAt - transform.position),
            rotationSmoothness * Time.deltaTime);
    }

    void OnDrawGizmosSelected()
    {
        if (target == null) return;
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(target.position + target.forward * lookAheadDistance, 0.3f);
        Gizmos.DrawLine(target.position, target.position + target.forward * lookAheadDistance);
    }
}