using UnityEngine;
using System.Collections.Generic;

public class Creatures03_PopulationManager_PhysicalEvolve_2Legs
{
    public GameObject root;
    public GameObject upper;
    public GameObject lower;

    public float distanceTraveled;
    private Rigidbody body;

    public HingeJoint knee;
    public HingeJoint hip;

    public Vector3 InitialPosition;

    // a list with all joints: hips
    public List<HingeJoint> hips = new List<HingeJoint>();

    // a list with all joints: knees
    public List<HingeJoint> knees = new List<HingeJoint>();

    // a list with all upper legs
    public List<GameObject> uppers = new List<GameObject>();

    // a list with all lower legs
    public List<GameObject> lowers = new List<GameObject>();

    private float motorForce;
    private float motorSpeed;

    private float limitsHipMin;
    private float limitsHipMax;
    private float limitsKneeMin;
    private float limitsKneeMax;
    private Vector3 bodySize;
    private Vector3 positionLeg1;
    private Vector3 positionLeg2;
    private Vector3 positionArm1; // discarded
    private Vector3 positionArm2; // discarded
    private Vector3 bodyCenterOfMass;
    private float upperLen;
    private float upperWidth;
    private float upperDepth;
    private float lowerLen;
    private float lowerWidth;
    private float lowerDepth;
    private float bodyMass;

    private int gravity = 1;

    // phaseOffset offsets the phase of the sinusoidal function used to drive the creature's joints
    private float phaseOffset = 0;

    public Creatures03_PopulationManager_PhysicalEvolve_2Legs(Vector3 spawnPos, List<object> genome)
    {
        InitialPosition = spawnPos;
        motorForce = (float)genome[0];      // [0, 300]
        motorSpeed = (float)genome[1];      // [0, 200]
        //Debug.Log($"speed in the second file - genome[1]: {(float)genome[1]}");
        //Debug.Log($"speed in the second file - motorSpeed: {motorSpeed}");
        limitsHipMin = (float)genome[2];     // [0, 2π]
        limitsHipMax = (float)genome[3];
        limitsKneeMin = (float)genome[4];
        limitsKneeMax = (float)genome[5];
        bodySize = (Vector3)genome[6];
        positionLeg1 = (Vector3)genome[7];
        positionLeg2 = (Vector3)genome[8];
        positionArm1 = (Vector3)genome[9]; // discarded
        positionArm2 = (Vector3)genome[10]; // discarded
        bodyCenterOfMass = (Vector3)genome[11];
        upperLen = (float)genome[12];
        upperWidth = (float)genome[13];
        upperDepth = (float)genome[14];
        //Debug.Log($"upperLen: {upperLen} - upperWidth: {upperWidth} - upperDepth: {upperDepth}");
        //Debug.Log($"genome[11]: {genome[11]} - genome[12]: {genome[12]} - genome[13]: {genome[13]}");

        lowerLen = (float)genome[15];
        lowerWidth = (float)genome[16];
        lowerDepth = (float)genome[17];
        bodyMass = (float)genome[18];
        //Debug.Log($"lowerLen: {lowerLen} - lowerWidth: {lowerWidth} - lowerDepth: {lowerDepth}");
        //Debug.Log($"genome[14]: {genome[14]} - genome[15]: {genome[15]} - genome[16]: {genome[16]}");

        // spawnPos is the initial position where the root body of the creature will be placed when it is created
        CreateBody(spawnPos);
        CreateLegs();
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
    }

    void CreateLegs()
    {
        Vector3[] offsets = new Vector3[]
        {
            // position of the four legs with respect to the main body
            positionLeg1, 
            positionLeg2
            //positionArm1, 
            //positionArm2
        };

        for (int i = 0; i < offsets.Length; i++)
        {
            // CREATE THE UPPER PART OF THE LEG
            upper = GameObject.CreatePrimitive(PrimitiveType.Cube);
            //upper.transform.parent = root.transform;

            // --> set the size
            //upperLen: the length of the upper leg segment
            //lowerLen: the length of the lower leg segment
            //width: the thickness of each leg segment
            //float upperLen = 0.5f, lowerLen = 0.5f, width = 0.15f;
            
            
            //Debug.Log($"upperLen: {upperLen} - upperWidth: {upperWidth} - upperDepth: {upperDepth}");
            upper.transform.localScale = new Vector3(upperDepth, upperLen, upperWidth);

            // add Rigidbody
            var upperRb = upper.AddComponent<Rigidbody>();
            if (gravity == 1) { upperRb.useGravity = true; } else { upperRb.useGravity = false; }

            // --> set the position

            // move down by half the length of the upper leg
            Vector3 upperPos = root.transform.position + offsets[i] + new Vector3(0, -upperLen / 2, 0);

            // move down by the length of the lower leg
            Vector3 lowerPos = upperPos + new Vector3(0, -lowerLen, 0);

            // position the upper part of the leg
            upper.transform.position = upperPos;

            upperRb.mass=1f;
            upperRb.angularDamping = 0f;

            // Add the upper leg to the list of uppers
            uppers.Add(upper);

            // add hips
            hip = upper.AddComponent<HingeJoint>();
            hip.connectedBody = body;
            hip.axis = Vector3.forward;
            hip.anchor = new Vector3(0, 0.5f, 0);
            hip.autoConfigureConnectedAnchor = false;
            hip.connectedAnchor = offsets[i];
            hip.useLimits = true;
            hip.limits = new JointLimits { min = limitsHipMin, max = limitsHipMax }; // 45 degrees both sides
            hip.useMotor = true;
            hips.Add(hip);


            // CREATE THE LOWER PART OF THE LEG
            lower = GameObject.CreatePrimitive(PrimitiveType.Cube);
            //lower.transform.parent = upper.transform;
            lower.transform.position = lowerPos;
            lower.transform.localScale = new Vector3(lowerDepth, lowerLen, lowerWidth);
            var lowerRb = lower.AddComponent<Rigidbody>();
            if (gravity == 1) { lowerRb.useGravity = true; } else { lowerRb.useGravity = false; }
            lowerRb.angularDamping = 0f;
            lowerRb.mass=1f;
            lowers.Add(lower);

            // add knees
            knee = lower.AddComponent<HingeJoint>();
            knee.connectedBody = upperRb;
            knee.axis = Vector3.forward;
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
        for (int i = 0; i < hips.Count; i++)
        {
            // 0 and 3 go together, 1 and 2 go together
            float phase = (i == 0 || i == 3) ? 0f : Mathf.PI;

            float velocity = Mathf.Sin(time * 2f + phase + phaseOffset) * motorSpeed;
            //Debug.Log($"Velocity: {velocity} - time: {time} - phase: {phase} - phaseOffset: {phaseOffset} - motorSpeed: {motorSpeed}");

            var m = hips[i].motor;
            m.force = motorForce;
            m.targetVelocity = velocity;
            hips[i].motor = m;

            var km = knees[i].motor;
            km.force = motorForce;
            km.targetVelocity = -velocity * 0.5f; // goes opposite direction half the speed
            knees[i].motor = km;
        }

        var dist = root.transform.position - InitialPosition;
        float dist_pytagor = Mathf.Sqrt((dist.x * dist.x) + (dist.z * dist.z));
        //distanceTraveled = dist_pytagor;

        // replace the fitness function, add Y as an important factor: want those who minimize the loss of Y
        // dist.y measures how much the creature rises (at the beginning it's negative)
        distanceTraveled = 100 + (5 * dist.y) + dist_pytagor; // dist.y is given a higher weight to make sure the creature stands
        

        //DEBUG: printout the calculated distance
        /*
        Debug.Log($"Original position (x,y,z): {InitialPosition.x},{InitialPosition.y},{InitialPosition.z}");
        Debug.Log($"Current position (x,y,z): {root.transform.position.x},{root.transform.position.y},{root.transform.position.z}");
        Debug.Log($"Distance (x,y,z): {dist.x},{dist.y},{dist.z}");
        Debug.Log($"Distance: {dist_pytagor}");
        Debug.Log($"Distance traveled by creature: {distanceTraveled}");
        */

    }

    // method to remove the creatures from one generation to the next
    public void RemoveCreatures()
        {
            GameObject.Destroy(root);            // body
            for (int a = 0; a < hips.Count; a++)
            {
                GameObject.Destroy(hips[a]);     // hips
                GameObject.Destroy(knees[a]);    // knees
                GameObject.Destroy(uppers[a]);   // uppers
                GameObject.Destroy(lowers[a]);   // lowers
            }
        }



}
