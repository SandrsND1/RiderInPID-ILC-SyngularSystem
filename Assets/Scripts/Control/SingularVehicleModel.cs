using UnityEngine;

public class SingularVehicleModel
{
    public float x, y, theta;
    public float delta;   // алгебраическая переменная (угол поворота колёс)
    public float v;

    // Колёсная база. 3 м — стандартный автомобиль.
    // Уменьши до 1.5–2.0 для картинга/маленькой машинки.
    public float L = 2.5f;

    public void Step(float steeringCommand, float dt)
    {
        dt = Mathf.Max(dt, 0.001f);

        // ── Сингулярное уравнение (DAE стабилизация) ──
        // residual = delta - cmd → гасим за ~1 физ. шаг (коэф. 20)
        float residual = delta - steeringCommand;
        delta -= residual * 20f;
        delta = Mathf.Clamp(delta, -0.8f, 0.8f);
        v     = Mathf.Clamp(v, 0.5f, 20f);

        // ── Кинематика велосипедной модели ──
        float dx     = v * Mathf.Cos(theta);
        float dy     = v * Mathf.Sin(theta);
        float dtheta = (v / L) * Mathf.Tan(delta);

        x     += dx     * dt;
        y     += dy     * dt;
        theta += dtheta * dt;

        // Оборачиваем theta в [-π, π] — без скачка производной
        theta = Mathf.Atan2(Mathf.Sin(theta), Mathf.Cos(theta));
    }
}