using UnityEngine;

public class VehicleModelHolder : MonoBehaviour
{
    public SingularVehicleModel model = new SingularVehicleModel();
    
    void Start()
    {
        // Начальная позиция
        model.x = 0;
        model.y = 0;
        model.theta = 0;
    }
}
