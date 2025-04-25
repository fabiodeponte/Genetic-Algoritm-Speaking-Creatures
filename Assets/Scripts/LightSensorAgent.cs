using UnityEngine;

public class LightSensorAgent : MonoBehaviour
{
    [Header("Sensor Settings")]
    public Light targetLight;                    // Assign manually or auto-detect by tag
    public float detectionRange = 200f;           // How far the sensor can detect
    public float angleSensitivity = 360;         // Max angle to consider the light "visible"
    public LayerMask obstacleMask;               // LayerMask for raycast (obstructions)

    [Header("Debug")]
    public bool lightDetected;
    public float detectedIntensity;

    private void Start()
    {
        // Optional: find the light by tag if not set
        if (targetLight == null)
        {
            GameObject lightObject = GameObject.FindGameObjectWithTag("TargetLight");
            if (lightObject != null)
            {
                targetLight = lightObject.GetComponent<Light>();
                Debug.LogError("Target Light not assigned or found.");
            }

        }

        if (targetLight == null)
        {
            Debug.LogError("Target Light not assigned or found.");
        }
        Debug.LogError($"Target Light: {targetLight}");
    }

    void Update()
    {
        Debug.LogError($"SenseLight: {SenseLight()}");
        if (SenseLight()==true)
        OnDrawGizmosSelected();

    }

    bool SenseLight()
    {
        if (targetLight == null)
        {
            lightDetected = false;
            //Debug.LogError("Target Light  = false");
            return lightDetected;
        }

        Vector3 toLight = targetLight.transform.position - transform.position;
        float distanceToLight = toLight.magnitude;
        float angle = Vector3.Angle(transform.forward, toLight);
        Debug.LogError($"distanceToLight: {distanceToLight}");
        Debug.LogError($"detectionRange: {detectionRange}");
        Debug.LogError($"angleSensitivity: {angleSensitivity}");
        Debug.LogError($"angle: {angle}");

        // Check if within range
        if (distanceToLight > detectionRange)
        {
            lightDetected = false;
            Debug.LogError($"1 - distanceToLight > detectionRange");
            return lightDetected;
        }

        // Check angle
        angle = Vector3.Angle(transform.forward, toLight);
        if (angle > angleSensitivity)
        {
            lightDetected = false;
            Debug.LogError($"2 - angle > angleSensitivity");
            return lightDetected;
        }

        // Check line of sight (raycast)
        if (Physics.Raycast(transform.position, toLight.normalized, out RaycastHit hit, detectionRange, obstacleMask))
        {
            if (hit.transform != targetLight.transform)
            {
                lightDetected = false;
                Debug.LogError($"3 - hit.transform != targetLight.transform");
                return lightDetected;
            }
        }

        // Calculate light intensity based on distance and angle
        float intensity = targetLight.intensity / (1 + distanceToLight * distanceToLight);
        intensity *= Mathf.Cos(angle * Mathf.Deg2Rad); // angle sensitivity

        detectedIntensity = intensity;
        lightDetected = true;

        Debug.Log($"Light detected! Intensity: {detectedIntensity:F2}");
        return lightDetected;
    }

    private void OnDrawGizmosSelected()
    {
        // Visualize detection area
        Gizmos.color = lightDetected ? Color.green : Color.red;
        Gizmos.DrawWireSphere(transform.position, detectionRange);

        // Show direction to light (if assigned)
        if (targetLight != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(transform.position, targetLight.transform.position);
        }
    }
}
