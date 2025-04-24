/* DA FARE:

* Crea una singola creatura con forza 200 e velocità 150 e verifica se cammina bene
* con qualsiasi configurazione le creature non camminano mai bene. Provare a:
    - allargare il range di forza e velocità
    - inserire altri parametri delle creature nel codice genetico

FATTO:
* verifica se la parte di selezione genetica funziona bene

*/

using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class GeneticWalkerSingleCreature : MonoBehaviour
{
    private float[] genome = new float[3];
    public GameObject body;
    private List<WalkerCreature> population = new List<WalkerCreature>();



    void Start()
    {
        GenerateCreatures();
        //CreateWheel();
        //CreateRotatingLeg();
    }

    void Update()
    {
        foreach (var creature in population)
        {
            creature.UpdateMotionExperiment(Time.time);
        }
    }


    void CreateRotatingLeg()
    {
        // Create the hip object
        GameObject hip = new GameObject("Hip");
        hip.transform.position = new Vector3(0, 2, 0);

        Rigidbody hipRb = hip.AddComponent<Rigidbody>();
        hipRb.isKinematic = true; // make it a static anchor point

        // Create the leg
        GameObject leg = GameObject.CreatePrimitive(PrimitiveType.Cube);
        leg.name = "Leg";
        leg.transform.localScale = new Vector3(0.3f, 1.5f, 0.3f); // tall and narrow
        leg.transform.position = hip.transform.position + new Vector3(0, -0.75f, 0); // position below the hip

        Rigidbody legRb = leg.AddComponent<Rigidbody>();
        legRb.mass = 1f;

        // Add hinge joint to the leg
        HingeJoint hinge = leg.AddComponent<HingeJoint>();
        hinge.connectedBody = hipRb;
        hinge.anchor = new Vector3(0, 0.5f, 0); // pivot at top of leg
        hinge.axis = Vector3.forward; // rotate around Z-axis (like a sideways swing)

        // Optional: limit rotation
        JointLimits limits = hinge.limits;
        limits.min = -180f;
        limits.max = 180f;
        hinge.limits = limits;
        hinge.useLimits = false; // set to true if you want range limitation

        // Motor
        JointMotor motor = hinge.motor;
        motor.force = 50f;
        motor.targetVelocity = 90f; // degrees/sec
        motor.freeSpin = false;

        hinge.motor = motor;
        hinge.useMotor = true;
    }




    void CreateWheel()
    {
        // Create the wheel object
        GameObject wheel = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        wheel.name = "Wheel";
        wheel.transform.position = new Vector3(0, 1, 0);  // position it above ground
        wheel.transform.localScale = new Vector3(1, 0.2f, 1); // make it flatter
        wheel.transform.Rotate(0, 0, 90); // rotate so it spins around the x-axis

        // Add Rigidbody for physics
        Rigidbody rb = wheel.AddComponent<Rigidbody>();
        rb.mass = 1f;

        // Create a base to attach the wheel to
        GameObject baseObject = new GameObject("WheelBase");
        baseObject.transform.position = wheel.transform.position;

        Rigidbody baseRb = baseObject.AddComponent<Rigidbody>();
        baseRb.isKinematic = true; // the base doesn't move

        // Add HingeJoint
        HingeJoint hinge = wheel.AddComponent<HingeJoint>();
        hinge.connectedBody = baseRb;
        hinge.axis = Vector3.right; // rotate around x-axis

        // Configure the motor
        JointMotor motor = hinge.motor;
        motor.force = 100f;
        motor.targetVelocity = 360f; // 360 degrees/sec = 1 full spin/sec
        motor.freeSpin = false;

        hinge.motor = motor;
        hinge.useMotor = true;
    }


    void GenerateCreatures()
    {
            // force: 0-300
            // speed: 30-180
            // phase: p-2pi


            genome = new float[] { 50f, 120f, 3.14f };
            var creature = new WalkerCreature(new Vector3(0, 1, 0), genome);
            population.Add(creature);

        /*

            genome = new float[] { 50f, 30f, 0f };
            var creature = new WalkerCreature(new Vector3(0, 2, 0), genome);
            population.Add(creature);

            genome = new float[] { 100f, 30f, 0f };
            creature = new WalkerCreature(new Vector3(5, 2, 0), genome);
            population.Add(creature);

            genome = new float[] { 150f, 30f, 0f };
            creature = new WalkerCreature(new Vector3(10, 2, 0), genome);
            population.Add(creature);

            genome = new float[] { 200f, 30f, 0f };
            creature = new WalkerCreature(new Vector3(15, 2, 0), genome);
            population.Add(creature);

            genome = new float[] { 250f, 30f, 0f };
            creature = new WalkerCreature(new Vector3(20, 2, 0), genome);
            population.Add(creature);
            
            genome = new float[] { 300f, 30f, 0f };
            creature = new WalkerCreature(new Vector3(25, 2, 0), genome);
            population.Add(creature);



            genome = new float[] { 100f, 60f, 0f };
            creature = new WalkerCreature(new Vector3(0, 2, 5), genome);
            population.Add(creature);

            genome = new float[] { 100f, 90f, 0f };
            creature = new WalkerCreature(new Vector3(5, 2, 5), genome);
            population.Add(creature);

            genome = new float[] { 100f, 120f, 0f };
            creature = new WalkerCreature(new Vector3(10, 2, 5), genome);
            population.Add(creature);

            genome = new float[] { 100f, 150f, 0f };
            creature = new WalkerCreature(new Vector3(15, 2, 5), genome);
            population.Add(creature);

            genome = new float[] { 100f, 180f, 0f };
            creature = new WalkerCreature(new Vector3(20, 2, 5), genome);
            population.Add(creature);
            
            genome = new float[] { 100f, 180f, 0f };
            creature = new WalkerCreature(new Vector3(25, 2, 5), genome);
            population.Add(creature);





            genome = new float[] { 100f, 120f, 0f };
            creature = new WalkerCreature(new Vector3(0, 2, 20), genome);
            population.Add(creature);

            genome = new float[] { 100f, 120f, 1f };
            creature = new WalkerCreature(new Vector3(5, 2, 20), genome);
            population.Add(creature);

            genome = new float[] { 100f, 120f, 2f };
            creature = new WalkerCreature(new Vector3(10, 2, 20), genome);
            population.Add(creature);

            genome = new float[] { 100f, 120f, 3.14f };
            creature = new WalkerCreature(new Vector3(15, 2, 20), genome);
            population.Add(creature);
        */

    }



    



}


