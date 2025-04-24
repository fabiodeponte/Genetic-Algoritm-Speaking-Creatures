

using UnityEngine;
using System.Collections.Generic;

public class WalkerCreature
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

    private int gravity = 1;

    // phaseOffset offsets the phase of the sinusoidal function used to drive the creature's joints
    private float phaseOffset;

    public WalkerCreature(Vector3 spawnPos, float[] genome)
    {
        InitialPosition = spawnPos;
        motorForce = genome[0];      // [0, 300]
        motorSpeed = genome[1];      // [0, 200]
        phaseOffset = genome[2];     // [0, 2π]

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
        root.transform.localScale = new Vector3(1, 0.4f, 1);
        body = root.AddComponent<Rigidbody>();
        body.centerOfMass = Vector3.zero;
        body.angularDamping = 0f;
        if (gravity == 1) { body.useGravity = true; } else { body.useGravity = false; }
        

        // weight fo the creature
        body.mass = 4f;
    }

    void CreateLegs()
    {
        Vector3[] offsets = new Vector3[]
        {
            // position of the four legs with respect to the main body
            new Vector3(-0.5f, 0, -0.5f), 
            new Vector3(0.5f, 0, -0.5f),
            new Vector3(-0.5f, 0,  0.5f), 
            new Vector3(0.5f, 0,  0.5f)
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
            float upperLen = 0.5f, lowerLen = 0.5f, width = 0.15f;
            upper.transform.localScale = new Vector3(width, upperLen, width);

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
            hip.limits = new JointLimits { min = -30f, max = 30f }; // 45 degrees both sides
            hip.useMotor = true;
            hips.Add(hip);


            // CREATE THE LOWER PART OF THE LEG
            lower = GameObject.CreatePrimitive(PrimitiveType.Cube);
            //lower.transform.parent = upper.transform;
            lower.transform.position = lowerPos;
            lower.transform.localScale = new Vector3(width, lowerLen, width);
            var lowerRb = lower.AddComponent<Rigidbody>();
            if (gravity == 1) { lowerRb.useGravity = true; } else { lowerRb.useGravity = false; }
            lowerRb.angularDamping = 0f;

            lowerRb.mass=1f;

/*
            // Add CapsuleCollider to the lower leg
            var lowerCapsule = lower.AddComponent<CapsuleCollider>(); // Add CapsuleCollider component
            lowerCapsule.direction = 1; // Y-axis
            lowerCapsule.height = lowerLen; // Set the height to match lower leg length
            lowerCapsule.radius = 2 * width; // Set the radius to half of the width of the leg
            lowerCapsule.center = new Vector3(0, 0, 0); // Adjust the center if needed
*/

            // Add the lower leg to the list of lowers
            lowers.Add(lower);



            // add knees
            knee = lower.AddComponent<HingeJoint>();
            knee.connectedBody = upperRb;
            knee.axis = Vector3.forward;
            knee.anchor = new Vector3(0, 0.5f, 0);
            knee.autoConfigureConnectedAnchor = false;
            knee.connectedAnchor = new Vector3(0, -0.5f, 0);
            knee.useLimits = true;
            knee.limits = new JointLimits { min = -45f, max = 0f }; // 60 degrees only one side
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
        distanceTraveled = dist_pytagor;

        //DEBUG: printout the calculated distance
        /*
        Debug.Log($"Original position (x,y,z): {InitialPosition.x},{InitialPosition.y},{InitialPosition.z}");
        Debug.Log($"Current position (x,y,z): {root.transform.position.x},{root.transform.position.y},{root.transform.position.z}");
        Debug.Log($"Distance (x,y,z): {dist.x},{dist.y},{dist.z}");
        Debug.Log($"Distance: {dist_pytagor}");
        Debug.Log($"Distance traveled by creature: {distanceTraveled}");
        */



    }


    public void UpdateMotionExperiment(float time)
    {
        for (int i = 0; i < hips.Count; i++)
        {
            // 0 and 3 go together, 1 and 2 go together
            float phase = (i == 1 || i == 2) ? 0f : Mathf.PI;
            //float velocity = Mathf.Sin(time * 2f + phase + phaseOffset) * motorSpeed;
            float velocity = Mathf.Sin(time * 2f + phase) * motorSpeed;
            //float velocity = 50f;

            var m = hips[i].motor;
            m.force = motorForce;
            m.targetVelocity = velocity;
            hips[i].motor = m;

            var km = knees[i].motor;
            km.force = motorForce;
            //km.targetVelocity = 0f;
            km.targetVelocity = -velocity * 0.5f; // goes opposite direction half the speed
            //knees[i].motor = km;
        }

        var dist = root.transform.position - InitialPosition;
        float dist_pytagor = Mathf.Sqrt((dist.x * dist.x) + (dist.z * dist.z));
        distanceTraveled = dist_pytagor;

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
            for (int a = 0; a < 4; a++)
            {
                GameObject.Destroy(hips[a]);     // hips
                GameObject.Destroy(knees[a]);    // knees
                GameObject.Destroy(uppers[a]);   // uppers
                GameObject.Destroy(lowers[a]);   // lowers
            }
        }
}
