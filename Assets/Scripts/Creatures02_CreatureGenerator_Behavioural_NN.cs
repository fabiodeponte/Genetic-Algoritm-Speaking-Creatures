using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class Creatures02_CreatureGenerator_Behavioural_NN
{

    // =========== DECLARATION OF VARIABLES AND OBJECTS NEEDED IN THE METHODS ===========

    public GameObject root;     // main body
    public GameObject upper;    // upper part of the legs
    public GameObject lower;    // lower part of the legs
    public HingeJoint hip;      // joint between the main body and the leg
    public HingeJoint knee;     // joint between upper and lower parts of the leg
    private Rigidbody body;     // component needed to add rigidbody to the main body

    // a list with all joints: hips
    public List<HingeJoint> hips = new List<HingeJoint>();

    // a list with all joints: knees
    public List<HingeJoint> knees = new List<HingeJoint>();

    // a list with all upper legs
    public List<GameObject> uppers = new List<GameObject>();

    // a list with all lower legs
    public List<GameObject> lowers = new List<GameObject>();

    private float motorForce;           // force applied to the legs
    private float motorSpeed;           // speed applied to the legs
    private float limitsHipMin;         // lower limit of the range of movement of the the hip (it's a range in grades, like from -30 to +15)
    private float limitsHipMax;         // higher limit of the range of movement of the the hip
    private float limitsKneeMin;        // lower limit of the range of movement of the the knee
    private float limitsKneeMax;        // higher limit of the range of movement of the the knee
    
    private Vector3 positionLeg1;       // three values to define position of the leg1 with respect to main body: basically, where the leg is attached
    private Vector3 positionLeg2;       // three values to define position of the leg2 with respect to main body: basically, where the leg is attached
    private Vector3 positionLeg3;       // three values to define position of the leg3 with respect to main body: basically, where the leg is attached
    private Vector3 positionLeg4;       // three values to define position of the leg4 with respect to main body: basically, where the leg is attached

    private Vector3 bodySize;           // three values to define length, width and depth of the body
    private Vector3 bodyCenterOfMass;   // three values to define the position of the center of gravity with the main body of the creature
    private float bodyMass;
    
    // three values to define length, width and depth of the upper part of the leg
    private float upperLen;
    private float upperWidth;
    private float upperDepth;

    // three values to define length, width and depth of the lower part of the leg
    private float lowerLen;
    private float lowerWidth;
    private float lowerDepth;

    public float distanceTraveled;  // variable for the fitness function: how far did the creature go in the time allowed?
    public Vector3 InitialPosition; // variable to store the initial position of the creature

    // this is used as a boolean: if gravity=1, gravity applies, if it's zero creatures float in the air
    private int gravity;

    // phaseOffset offsets the phase of the sinusoidal function used to drive the creature's joints (legs have a cyclical movement): 
    // it changes the intial position, but not the movement of the legs relative to one another
    //private float phaseOffset;

    public Genome genomeNN;
    private NeuralNetwork nn;

    // the value of the light sensor
    //public float LightSensor_08_NNs Value;

    private LightSensor_08_NNs_new sensor;
    float normalized_distance;
    float normalized_angle;

    public float[] lastOutputsOfTheSingleExperiment;

    public Creatures02_CreatureGenerator_Behavioural_NN(Vector3 spawnPos, List<object> genome)
    {
        gravity = 1;
        //phaseOffset = 0;
        
        InitialPosition = spawnPos;
        motorForce = (float)genome[0];          // [50 | 300]
        motorSpeed = (float)genome[1];          // [30 | 70]
            //Debug.Log($"speed in the second file - genome[1]: {(float)genome[1]}");
            //Debug.Log($"speed in the second file - motorSpeed: {motorSpeed}");
        limitsHipMin = (float)genome[2];        // [-30]
        limitsHipMax = (float)genome[3];        // [30]
        limitsKneeMin = (float)genome[4];       // [-45]
        limitsKneeMax = (float)genome[5];       // [0]
        bodySize = (Vector3)genome[6];          // [each dimension randomly picked between 1 and 2]
        positionLeg1 = (Vector3)genome[7];      // [-0.5f, 0, -0.5f]
        positionLeg2 = (Vector3)genome[8];      // [0.5f, 0, -0.5f]
        positionLeg3 = (Vector3)genome[9];      // [-0.5f, 0,  0.5f]
        positionLeg4 = (Vector3)genome[10];     // [0.5f, 0,  0.5f]
        bodyCenterOfMass = (Vector3)genome[11]; // [each dimension randomly picked between -0.2 and 0.2]
        upperLen = (float)genome[12];           // [0.1 | 1]
        upperWidth = (float)genome[13];         // [0.1 | 1]
        upperDepth = (float)genome[14];         // [0.1 | 1]
            //Debug.Log($"upperLen: {upperLen} - upperWidth: {upperWidth} - upperDepth: {upperDepth}");
            //Debug.Log($"genome[11]: {genome[11]} - genome[12]: {genome[12]} - genome[13]: {genome[13]}");
        lowerLen = (float)genome[15];           // [0.1 | 1]
        lowerWidth = (float)genome[16];         // [0.1 | 1]
        lowerDepth = (float)genome[17];         // [0.1 | 1]
        bodyMass = (float)genome[18];           // [3 | 5]
            //Debug.Log($"lowerLen: {lowerLen} - lowerWidth: {lowerWidth} - lowerDepth: {lowerDepth}");
            //Debug.Log($"genome[14]: {genome[14]} - genome[15]: {genome[15]} - genome[16]: {genome[16]}");

        genomeNN = (Genome)genome[19];          // object
        nn = new NeuralNetwork(genomeNN.ReturnWeights());

        // spawnPos is the initial position where the root body of the creature will be placed when it is created
        CreateBody(spawnPos);
        CreateLegs(); // the position of the legs is then relative to the position of the body

    }

    void CreateBody(Vector3 pos)
    {
        root = GameObject.CreatePrimitive(PrimitiveType.Cube);
        root.name = "CreatureBody";
        root.transform.position = pos;


        // size of the creature
        root.transform.localScale = bodySize;
        body = root.AddComponent<Rigidbody>();
        body.centerOfMass = Vector3.zero;
        body.angularDamping = 0f;
        if (gravity == 1) { body.useGravity = true; } else { body.useGravity = false; }
        
        // weight fo the creature
        body.mass = bodyMass;

        // add the sensor
        sensor = root.AddComponent<LightSensor_08_NNs_new>();
        sensor.detectionRange = GeneralConfigurationParameters.lightSensorDetectionRange;
        sensor.angleSensitivity = GeneralConfigurationParameters.lightSensorAngleSensitivity;
        sensor.RegisterInitialPosition(InitialPosition);


    }

    void CreateLegs()
    {
        Vector3[] offsets = new Vector3[]
        {
            // position of the four legs with respect to the main body
            positionLeg1, 
            positionLeg2,
            positionLeg3, 
            positionLeg4
        };

        for (int i = 0; i < offsets.Length; i++)
        {
            // ============= CREATE THE UPPER PART OF THE LEG
            upper = GameObject.CreatePrimitive(PrimitiveType.Cube);

            // size and shape of the upper part of the leg
            upper.transform.localScale = new Vector3(upperDepth, upperLen, upperWidth);

            // add Rigidbody
            var upperRb = upper.AddComponent<Rigidbody>();
            if (gravity == 1) { upperRb.useGravity = true; } else { upperRb.useGravity = false; }

            // --> set the position

            // move down by half the length of the upper leg
            Vector3 upperPos = root.transform.position + offsets[i] + new Vector3(0, -upperLen / 2, 0);

            // position the upper part of the leg
            upper.transform.position = upperPos;

            // needed to avoid strange movements
            upperRb.angularDamping = 0f;

            // weight of the upper leg = 1
            upperRb.mass=1f;

            // Add the upper leg to the list of uppers
            uppers.Add(upper);

            // ============= CREATE HIPS
            hip = upper.AddComponent<HingeJoint>();
            hip.connectedBody = body;
            hip.axis = Quaternion.Euler(0, 270, 0) * Vector3.forward; // applied rotation to align the legs with the way forward
            hip.anchor = new Vector3(0, 0.5f, 0);
            hip.autoConfigureConnectedAnchor = false;
            hip.connectedAnchor = offsets[i];
            hip.useLimits = true;
            hip.limits = new JointLimits { min = limitsHipMin, max = limitsHipMax }; // 45 degrees both sides
            hip.useMotor = true;
            hips.Add(hip);


            // ============= CREATE THE LOWER PART OF THE LEG
            lower = GameObject.CreatePrimitive(PrimitiveType.Cube);

            // size and shape of the lower part of the leg
            lower.transform.localScale = new Vector3(lowerDepth, lowerLen, lowerWidth);

            // move down by the length of the lower leg
            Vector3 lowerPos = upperPos + new Vector3(0, -lowerLen, 0);
            lower.transform.position = lowerPos;

            // add Rigidbody
            var lowerRb = lower.AddComponent<Rigidbody>();

            // use gravity
            if (gravity == 1) { lowerRb.useGravity = true; } else { lowerRb.useGravity = false; }

            // needed to avoid strange movements
            lowerRb.angularDamping = 0f;

            // weight of the lower leg = 1
            lowerRb.mass=1f;

            // Add the lower leg to the list of lowers
            lowers.Add(lower);

            // ============= CREATE KNEES
            knee = lower.AddComponent<HingeJoint>();
            knee.connectedBody = upperRb;
            knee.axis = Quaternion.Euler(0, 270, 0) * Vector3.forward; // applied rotation to align the legs with the way forward
            knee.anchor = new Vector3(0, 0.5f, 0);
            knee.autoConfigureConnectedAnchor = false;
            knee.connectedAnchor = new Vector3(0, -0.5f, 0);
            knee.useLimits = true;
            knee.limits = new JointLimits { min = limitsKneeMin, max = limitsKneeMax }; // 60 degrees only one side
            knee.useMotor = true;
            knees.Add(knee);
        }
    }


    public void UpdateMotion(float time)
    {

        // Inputs
        float light_dist = sensor.LightDistance();
        normalized_distance = LightSensor_08_NNs_new.NormalizeDistance(light_dist); // normalise at [0,1] - at distance 200 and higher it returns 1, between 0 and 200 it's logarithmic

        float light_angle = sensor.LightAngle();        // this ranges from -180 to 180 degrees - normalise at [0,1]
        normalized_angle = LightSensor_08_NNs_new.NormalizeAngle(light_angle, -180, +180);

        float[] inputs = new float[2];
        inputs[0] = normalized_distance;
        inputs[1] = normalized_angle;

        float[] outputs = nn.FeedForward(inputs); // apply the neural network
        float[] expanded_outputs = outputs;

        // we expand now the output for force and speed from the range [0,1] to the range [min_force, max_force] and [min_speed, max_speed] respectively

        for (int i=0; i < outputs.Length; i=i+2) // force: neurons 0, 2, 4, 6
        {
            expanded_outputs[i] = LightSensor_08_NNs_new.ExpandToRange(outputs[i], Creatures02_PopulationManager_Behavioural_NN.min_force, Creatures02_PopulationManager_Behavioural_NN.max_force);
        }

        for (int i=1; i < outputs.Length; i=i+2) // speed: neurons 1, 3, 5, 7
        {
            expanded_outputs[i] = LightSensor_08_NNs_new.ExpandToRange(outputs[i], Creatures02_PopulationManager_Behavioural_NN.min_speed, Creatures02_PopulationManager_Behavioural_NN.max_speed);
        }

        lastOutputsOfTheSingleExperiment = expanded_outputs;

/*
        for (int i=0; i < expanded_outputs.Length; i++)
        {
            Debug.LogError($"expanded_outputs[{i}]: {expanded_outputs[i]}");
        }
*/

        // 3. Apply the results to the legs
        for (int i = 0; i < hips.Count; i++)
        {
            float motorForce = outputs[i * 2];     // First out: force
            float velocity = outputs[i * 2 + 1]; // Second out: speed

            // apply force and velocity to the hip
            var m = hips[i].motor;
            m.force = motorForce;
            m.targetVelocity = velocity;
            hips[i].motor = m;

            // apply force and (a opposite half-value) velocity to the knee
            var km = knees[i].motor;
            km.force = motorForce;
            km.targetVelocity = -velocity * 0.5f; // goes opposite direction half the speed
            knees[i].motor = km;
        }

        var a = sensor.SenseLight(); // at each update we update the value of the light sensor as well
        distanceTraveled = sensor.distanceTraveledTowardsLight(); // the original distance to the light minus the current distance for the fitness function
    }


    // method to remove the creatures from one generation to the next
    public void RemoveCreatures()
        {
            GameObject.Destroy(root);            // body
            for (int a = 0; a < 4; a++)
            {
                GameObject.Destroy(hips[a]);     // hips
                GameObject.Destroy(knees[a]);    // knees
                GameObject.Destroy(uppers[a]);   // uppers
                GameObject.Destroy(lowers[a]);   // lowers
            }
        }


}

public class LightSensor_08_NNs_new : MonoBehaviour
{
    //[Header("Sensor Settings")]
    public Light targetLight;              // Assign manually or auto-detect by tag
    public float detectionRange;           // How far the sensor can detect
    public float angleSensitivity;         // Max angle to consider the light "visible"
    public LayerMask obstacleMask;         // LayerMask for raycast (obstructions)

    //[Header("Debug")]
    public bool lightDetected;
    public float detectedIntensity;

    public float originalDistanceToLight;
    public Vector3 initialPositionSensor;

    private void Start()
    {
        // Optional: find the light by tag if not set
        if (targetLight == null)
        {
            GameObject lightObject = GameObject.FindGameObjectWithTag("TargetLight");
            if (lightObject != null)
            {
                targetLight = lightObject.GetComponent<Light>();
            }

        }

        if (targetLight == null)
        {
            Debug.LogError("Target Light not assigned or found.");
        }

        targetLight.shadows = LightShadows.None; // the target light does not project shadows to save computational effort
        //Debug.LogError($"Target Light: {targetLight}");


        originalDistanceToLight = (targetLight.transform.position - initialPositionSensor).magnitude;
    }

    public bool SenseLight()
    {
        //Vector3 direction = Vector3.forward;
        //Debug.DrawRay(transform.position, direction * 10, Color.yellow);

        Vector3 origin = transform.position + Vector3.up * 0.5f; // put the light slightly above the body
        Vector3 direction = transform.forward;                   // the direction the look at to detect the light
        Debug.DrawRay(origin, direction * 10, Color.gray);


        if (targetLight == null)
        {
            lightDetected = false;
            //Debug.LogError("Target Light  = false");
            return lightDetected;
        }

        Vector3 toLight = targetLight.transform.position - transform.position;
        float distanceToLight = toLight.magnitude;
        float angle = Vector3.SignedAngle(transform.forward, toLight, Vector3.up);
        // Debug.LogError($"distanceToLight: {distanceToLight} - angle: {angle}");
        // Debug.LogError($"detectionRange: {detectionRange}");
        // Debug.LogError($"angleSensitivity: {angleSensitivity}");
        // Debug.LogError($"angle: {angle}");

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
        float intensity = targetLight.intensity *1000 / (1 + distanceToLight);
        intensity *= Mathf.Cos(angle * Mathf.Deg2Rad); // angle sensitivity

        detectedIntensity = intensity;
        lightDetected = true;

        //Debug.Log($"==================Light detected! angle: {angle}");
        //Debug.Log($"==================Light detected! targetLight.intensity: {targetLight.intensity}");
        //Debug.Log($"==================Light detected! distanceToLight: {distanceToLight}");
        //Debug.Log($"==================Light detected! Intensity: {detectedIntensity}");
        return lightDetected;
    }

    public void RegisterInitialPosition(Vector3 initialPosition)
    {
        initialPositionSensor = initialPosition;
    }

    public float LightDistance()
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f; // put the light slightly above the body
        Vector3 direction = transform.forward;                   // the direction the look at to detect the light
        Vector3 toLight = targetLight.transform.position - transform.position;
        float distanceToLight = toLight.magnitude;
        return distanceToLight;
    }

    public float distanceTraveledTowardsLight()
    {
        float currentDistance = LightDistance();
        float dist_traveled = originalDistanceToLight - currentDistance;
        return dist_traveled;
    }

    public float LightAngle()
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f; // put the light slightly above the body
        Vector3 direction = transform.forward;                   // the direction the look at to detect the light
        Vector3 toLight = targetLight.transform.position - transform.position;
        float angle = Vector3.Angle(transform.forward, toLight);
        return angle;
    }

    public static float NormalizeAngle(float value, float min, float max)
    {
        if (Mathf.Approximately(max, min))
        {
            Debug.LogWarning("Min and Max are equal; returning 0 to avoid division by zero.");
            return 0f;
        }
        return (value - min) / (max - min);
    }

    public static float NormalizeDistance(float distance, float maxDistance = 200f)  // this makes use of a logarithmic scale
    {
        // Clamp distance to [0, maxDistance]
        float clampedDistance = Mathf.Clamp(distance, 0f, maxDistance); // if it gets a distance higher than maxdistance, it truncates at maxdistances and returns 1

        // Compute logarithmic normalization
        float logValue = Mathf.Log10(clampedDistance + 1f);          // Avoid log(0)
        float logMax = Mathf.Log10(maxDistance + 1f);                // Precompute max log

        return logValue / logMax;                                    // Return value in [0,1]
    }

    public static float ExpandToRange(float normalizedValue, float min, float max)
    {
        // Clamp the input to make sure it's within [0, 1]
        float clampedValue = Mathf.Clamp01(normalizedValue);

        // Linearly expand to the desired range
        return clampedValue * (max - min) + min;
    }

    public void OnDrawGizmosSelected_08_NNs()
    {
        Debug.Log("Drawing Gizmos!");
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

public class Genome
{
    public float[][][] genomeNeuralNetworkWeights;

    public float[][][] ReturnWeights()
    {
        return genomeNeuralNetworkWeights;
    }
    
    public void SaveWeights(float[][][] weights)
    {
        genomeNeuralNetworkWeights = weights;
    }

    public void MutateNetworkWeights(float mutationRangeMin = -0.1f, float mutationRangeMax = 0.1f)
    {
        for (int layer = 0; layer < genomeNeuralNetworkWeights.Length; layer++)
        {
            for (int neuron = 0; neuron < genomeNeuralNetworkWeights[layer].Length; neuron++)
            {
                for (int input = 0; input < genomeNeuralNetworkWeights[layer][neuron].Length; input++)
                {
                    genomeNeuralNetworkWeights[layer][neuron][input] += Random.Range(mutationRangeMin, mutationRangeMax);
                    genomeNeuralNetworkWeights[layer][neuron][input] = Mathf.Clamp01(genomeNeuralNetworkWeights[layer][neuron][input]);
                }
            }
        }
    }

}


public class NeuralNetwork
{
    private float[][][] weights; // [layer][neuron][input]

    public NeuralNetwork(float[][][] initialWeights)
    {
        weights = initialWeights;
    }

// THIS NETWORK HAS 
// - Sigmoid activation for all hidden layers
// - ReLU on FORCE outputs 0, 2, 4, 6
// - Tanh on SPEED outputs 1, 3, 5, 7
    public float[] FeedForward(float[] inputs)
    {
        float[] current = inputs;

        for (int layer = 0; layer < weights.Length; layer++)
        {
            int numNeurons = weights[layer].Length;
            float[] next = new float[numNeurons];
            bool isOutputLayer = (layer == weights.Length - 1);

            for (int n = 0; n < numNeurons; n++)
            {
                float sum = 0f;
                for (int i = 0; i < weights[layer][n].Length; i++)
                {
                    sum += current[i] * weights[layer][n][i];
                }

                if (isOutputLayer)
                {
                    // Output layer: use ReLU for even indices (force), Tanh for odd (speed)
                    if (n % 2 == 0)
                        // next[n] = ReLU(sum);
                        next[n] = Tanh(sum);
                    else
                        next[n] = Tanh(sum);
                }
                else
                {
                    // Hidden layers: use Sigmoid
                    next[n] = Tanh(sum);


                }
            }

            current = next;
        }

        return current;
    }


/*
    public float[] FeedForward(float[] inputs)
    {
        float[] current = inputs;

        for (int layer = 0; layer < weights.Length; layer++)
        {
            int numNeurons = weights[layer].Length;
            float[] next = new float[numNeurons];

            for (int n = 0; n < numNeurons; n++)
            {
                float sum = 0f;
                for (int i = 0; i < weights[layer][n].Length; i++)
                {
                    sum += current[i] * weights[layer][n][i];
                }

                next[n] = Tanh(sum);
            }

            current = next;
        }

        return current;
    }
*/

    private float Sigmoid(float x)
    {
        return 1f / (1f + Mathf.Exp(-x));
    }

    private float Tanh(float x)
    {
        float ePos = Mathf.Exp(x);
        float eNeg = Mathf.Exp(-x);
        return (ePos - eNeg) / (ePos + eNeg);
    }

    private float ReLU(float x)
    {
        return Mathf.Max(0f, x);
    }


/*
    public static float[][][] MutateNetworkWeights(float[][][] weights, float mutationRangeMin = -0.1f, float mutationRangeMax = 0.1f)
    {
        for (int layer = 0; layer < weights.Length; layer++)
        {
            for (int neuron = 0; neuron < weights[layer].Length; neuron++)
            {
                for (int input = 0; input < weights[layer][neuron].Length; input++)
                {
                    weights[layer][neuron][input] += Random.Range(mutationRangeMin, mutationRangeMax);
                }
            }
        }
        return weights;
    }
*/
}
