using UnityEngine;
using System.Collections.Generic;

public class CreatureGenerator_08_NNs

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
    // private float phaseOffset;

    // the value of the light sensor
    public float lightSensorValue;

    private LightSensor sensor;

    public NeuralNetwork nn;


    public CreatureGenerator_08_NNs(Vector3 spawnPos, List<object> genome)
    {
        gravity = 1;
        // phaseOffset = 0;
        
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


if (genome == null)
{
    Debug.LogError("Genome list is NULL!");
}
else if (genome.Count <= 19)
{
    Debug.LogError("Genome list is too short: Count = " + genome.Count);
}
else if (genome[19] == null)
{
    Debug.LogError("Genome[19] is NULL!");
}
else
{
    Debug.Log($"Genome[19]: {(Genome)genome[19]}");
    Debug.Log($"layerSizes: {((Genome)genome[19]).layerSizes}");
    Debug.Log($"weights: {((Genome)genome[19]).weights}");
}


Genome g = (Genome)genome[19];

for (int layer = 0; layer < g.weights.Length; layer++) // <--- use Length instead of Count
{
    float[][] matrix = g.weights[layer];
    int rows = matrix.Length; // how many "from" neurons

    Debug.Log($"Layer {layer}: {rows} rows");

    for (int i = 0; i < rows; i++)
    {
        int cols = matrix[i].Length; // how many "to" neurons
        string rowText = $"Row {i}: ";
        for (int j = 0; j < cols; j++)
        {
            rowText += matrix[i][j].ToString("F3") + " ";
        }
        Debug.Log(rowText);
    }
}




        //NeuralNetwork nn01 = new NeuralNetwork((Genome)genome[19]); // creates a neural network with the received genome
        NeuralNetwork nn = new NeuralNetwork((Genome)genome[19]);

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
        sensor = root.AddComponent<LightSensor>();
        sensor.detectionRange = 200f;
        sensor.angleSensitivity = 300f;
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
        float light_angle = sensor.LightAngle();
        float[] inputs = new float[2];
        inputs[0] = light_dist;
        inputs[1] = light_angle;
        distanceTraveled = light_dist;

        // Run the network
        float[] outputs = nn.FeedForward(inputs);
        Debug.Log($"outputs.Length: {outputs.Length}");



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
        //if (sensor.SenseLight()==true) sensor.OnDrawGizmosSelected();
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

public class LightSensor : MonoBehaviour
{
    //[Header("Sensor Settings")]
    public Light targetLight;              // Assign manually or auto-detect by tag
    public float detectionRange;           // How far the sensor can detect
    public float angleSensitivity;         // Max angle to consider the light "visible"
    public LayerMask obstacleMask;         // LayerMask for raycast (obstructions)

    //[Header("Debug")]
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
            }

        }

        if (targetLight == null)
        {
            Debug.LogError("Target Light not assigned or found.");
        }
        //Debug.LogError($"Target Light: {targetLight}");
    }

/*
    void Update()
    {
        Debug.LogError($"SenseLight: {SenseLight()}");
        if (SenseLight()==true)
        OnDrawGizmosSelected();

    }
*/

    public float LightDistance()
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f; // put the light slightly above the body
        Vector3 direction = transform.forward;                   // the direction the look at to detect the light
        Vector3 toLight = targetLight.transform.position - transform.position;
        float distanceToLight = toLight.magnitude;
        return distanceToLight;
    }

    public float LightAngle()
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f; // put the light slightly above the body
        Vector3 direction = transform.forward;                   // the direction the look at to detect the light
        Vector3 toLight = targetLight.transform.position - transform.position;
        float angle = Vector3.Angle(transform.forward, toLight);
        return angle;
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
        float intensity = targetLight.intensity *1000 / (1 + distanceToLight);
        intensity *= Mathf.Cos(angle * Mathf.Deg2Rad); // angle sensitivity

        detectedIntensity = intensity;
        lightDetected = true;

        Debug.Log($"==================Light detected! angle: {angle}");
        Debug.Log($"==================Light detected! targetLight.intensity: {targetLight.intensity}");
        Debug.Log($"==================Light detected! distanceToLight: {distanceToLight}");
        Debug.Log($"==================Light detected! Intensity: {detectedIntensity}");
        return lightDetected;
    }

    public void OnDrawGizmosSelected()
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
    public int[] layerSizes;
    public float[][][] weights;

    public Genome(int[] layerSizes, float[][][] weights)
    {
        this.layerSizes = layerSizes;
        this.weights = weights;
    }

    // Create a new random Genome
    public static Genome CreateRandomGenome(int numInputs, int numOutputs, int minHiddenLayers = 1, int maxHiddenLayers = 3, int minNeuronsPerLayer = 2, int maxNeuronsPerLayer = 8)
    {
        int hiddenLayerCount = UnityEngine.Random.Range(minHiddenLayers, maxHiddenLayers + 1);

        List<int> layers = new List<int>();
        layers.Add(numInputs); // Input layer

        for (int i = 0; i < hiddenLayerCount; i++)
        {
            int neurons = UnityEngine.Random.Range(minNeuronsPerLayer, maxNeuronsPerLayer + 1);
            layers.Add(neurons);
        }

        layers.Add(numOutputs); // Output layer

        int[] layerSizes = layers.ToArray();

        float[][][] weights = new float[layerSizes.Length - 1][][];

        for (int i = 0; i < layerSizes.Length - 1; i++)
        {
            weights[i] = new float[layerSizes[i + 1]][]; // from layer i to i+1

            for (int j = 0; j < layerSizes[i + 1]; j++)
            {
                weights[i][j] = new float[layerSizes[i]];
                for (int k = 0; k < layerSizes[i]; k++)
                {
                    weights[i][j][k] = UnityEngine.Random.Range(-1f, 1f);
                }
            }
        }

        return new Genome(layerSizes, weights);
    }
}

public class NeuralNetwork
{
    public int[] layerSizes;
    public float[][] neurons;
    public float[][][] weights;

    // Constructor from a Genome
    public NeuralNetwork(Genome genome)
    {
        this.layerSizes = (int[])genome.layerSizes.Clone();

        this.neurons = new float[layerSizes.Length][];
        for (int i = 0; i < layerSizes.Length; i++)
        {
            neurons[i] = new float[layerSizes[i]];
        }

        this.weights = new float[genome.weights.Length][][];
        for (int i = 0; i < genome.weights.Length; i++)
        {
            weights[i] = new float[genome.weights[i].Length][];
            for (int j = 0; j < genome.weights[i].Length; j++)
            {
                weights[i][j] = new float[genome.weights[i][j].Length];
                for (int k = 0; k < genome.weights[i][j].Length; k++)
                {
                    weights[i][j][k] = genome.weights[i][j][k];
                }
            }
        }
    }

    // Export NeuralNetwork to Genome
    public Genome ExportToGenome()
    {
        int[] exportedLayerSizes = (int[])layerSizes.Clone();

        float[][][] exportedWeights = new float[weights.Length][][];

        for (int i = 0; i < weights.Length; i++)
        {
            exportedWeights[i] = new float[weights[i].Length][];

            for (int j = 0; j < weights[i].Length; j++)
            {
                exportedWeights[i][j] = new float[weights[i][j].Length];
                for (int k = 0; k < weights[i][j].Length; k++)
                {
                    exportedWeights[i][j][k] = weights[i][j][k];
                }
            }
        }

        return new Genome(exportedLayerSizes, exportedWeights);
    }



public float[] FeedForward(float[] inputs)
{
    if (inputs == null)
    {
        Debug.LogError("FeedForward: inputs is NULL");
        return null;
    }
    
    if (neurons == null)
    {
        Debug.LogError("FeedForward: neurons is NULL");
        return null;
    }

    Debug.Log("FeedForward: neurons is initialized");
    
    if (weights == null)
    {
        Debug.LogError("FeedForward: weights is NULL");
        return null;
    }
    
    Debug.Log("FeedForward: weights is initialized");
    
    if (neurons[0] == null)
    {
        Debug.LogError("FeedForward: neurons[0] is NULL");
        return null;
    }

    if (inputs.Length != neurons[0].Length)
    {
        Debug.LogError($"FeedForward: input length {inputs.Length} does not match input layer size {neurons[0].Length}");
        return null;
    }

    // Log input values
    Debug.Log("FeedForward: inputs = " + string.Join(",", inputs));

    // Set input neurons
    for (int i = 0; i < inputs.Length; i++)
    {
        neurons[0][i] = inputs[i];
    }

    // Log neurons[0] after setting inputs
    Debug.Log("FeedForward: neurons[0] after input assignment = " + string.Join(",", neurons[0]));

    // Feed through layers
    for (int layer = 1; layer < neurons.Length; layer++)
    {
        if (neurons[layer] == null)
        {
            Debug.LogError($"FeedForward: neurons[{layer}] is NULL");
            return null;
        }

        Debug.Log($"FeedForward: Processing layer {layer}");

        for (int neuron = 0; neuron < neurons[layer].Length; neuron++)
        {
            if (weights[layer - 1] == null)
            {
                Debug.LogError($"FeedForward: weights[{layer - 1}] is NULL");
                return null;
            }

            if (weights[layer - 1].Length != neurons[layer - 1].Length)
            {
                Debug.LogError($"FeedForward: weights[{layer - 1}] size mismatch for layer {layer}");
                return null;
            }

            float sum = 0f;

            for (int prevNeuron = 0; prevNeuron < neurons[layer - 1].Length; prevNeuron++)
            {
                if (weights[layer - 1][prevNeuron] == null)
                {
                    Debug.LogError($"FeedForward: weights[{layer - 1}][{prevNeuron}] is NULL");
                    return null;
                }

                sum += neurons[layer - 1][prevNeuron] * weights[layer - 1][prevNeuron][neuron];
            }

            neurons[layer][neuron] = ActivationFunction(sum);
        }
    }

    return neurons[neurons.Length - 1]; // Output layer
}



// ============================ DA QUI ==================================

/*
    // FeedForward
    public float[] FeedForward(float[] inputs)
    {
        if (inputs.Length != neurons[0].Length)
        {
            Debug.LogError("Input size does not match network input size!");
            return null;
        }

        // Assign inputs
        for (int i = 0; i < inputs.Length; i++)
        {
            neurons[0][i] = inputs[i];
        }

        // Propagate through network
        for (int layer = 1; layer < neurons.Length; layer++)
        {
            for (int neuron = 0; neuron < neurons[layer].Length; neuron++)
            {
                float sum = 0f;
                for (int prevNeuron = 0; prevNeuron < neurons[layer - 1].Length; prevNeuron++)
                {
                    sum += neurons[layer - 1][prevNeuron] * weights[layer - 1][neuron][prevNeuron];
                }
                neurons[layer][neuron] = ActivationFunction(sum);
            }
        }

        return neurons[neurons.Length - 1]; // Output layer
    }
*/


    private float ActivationFunction(float x)
    {
        return 1f / (1f + Mathf.Exp(-x)); // Sigmoid activation, range (0,1)
    }
}
