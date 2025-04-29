/* =============== DA FARE ADESSO
inserita una luce e i sensori, orientati nel senso di marcia dell'agente
adesso bisogna 
- costruire una nuova scena
- costruire la rete neurale che collega il sensore della luce al movimento delle gambe
- mettere i parametri della rete neurale nel genoma, in modo che possa modificarsi con l'evoluzione
- cambiare la fitness function in modo che tenga conto della distanza dalla luce
*/


using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.SceneManagement;


public class GeneticWalkerEnhanced4LegsWithSensors : MonoBehaviour
{
    
    public int populationSize;
    public float evaluationTime;
    private float timer;

    private List<WalkerCreatureEnhanced4LegsWithSensors> population = new List<WalkerCreatureEnhanced4LegsWithSensors>();
    private List<object> genome = new List<object>();
    private List<List<object>> genomes = new List<List<object>>();
    private int generation = 0;

    public GameObject body;



    private Vector3 bodySize;
    private Vector3 positionLeg1;
    private Vector3 positionLeg2;
    private Vector3 positionLeg3;
    private Vector3 positionLeg4;

    private Vector3 bodyCenterOfMass;

    private float upperLen;
    private float upperWidth;
    private float upperDepth;

    private float lowerLen;
    private float lowerWidth;
    private float lowerDepth;

    private float limitsHipMin;
    private float limitsHipMax;

    private float limitsKneeMin;
    private float limitsKneeMax;

    private float force;
    private float speed;
    private float bodyMass;

    private WalkerCreatureEnhanced4LegsWithSensors creature;

    public Light targetLight;






    void Start()
    {
        // parameters
        populationSize = 20;  // size of the population
        evaluationTime = 30f; // 30 seconds per generation
        Time.timeScale = 10f; // accelerates time x10 

        //create a light
        CreateLight();

        //update the parameters of the light
        Color lightColor = Color.white;           // Light color
        float lightIntensity = 25.0f;              // Light intensity
        float lightRange = 10.0f;
        Vector3 newPosition = new Vector3(-10, 5, -10);
        UpdateLightSettings(lightColor, lightIntensity, lightRange, newPosition);


        // generate the population
        GenerateInitialPopulation();
    }

    void Update()
    {
        timer += Time.deltaTime; // deltaTime is the time passed since past frame
        foreach (var creature in population)
        {
            creature.UpdateMotion(Time.time);  // this makes the creatures move
        }

        // after 30 seconds, we call EvolvePopulation, that selects the best performing creatures and generates the next generation
        if (timer >= evaluationTime)
        {
            EvolvePopulation();
        }
        
    }

    void GenerateInitialPopulation()
    {
        // These are helpful later in dealing with tuple of three values of type Vector3
        float a = 0;
        float b = 0;
        float c = 0;

        for (int i = 0; i < populationSize; i++) // for each creature, we generate a genome, meaning a certain combination of feature values
        {
        force = Random.Range(50f, 300f); // force
        speed = Random.Range(30f, 70f);  // speed

        // size and shape of the central body
        a = Random.Range(1f, 2f);
        b = Random.Range(1f, 2f);
        c = Random.Range(1f, 2f);
        bodySize = new Vector3(a, b, c); // default was: (1, 0.4f, 1);
        
        // position the leg 1 with respect to the central body
        a = Random.Range(-0.6f, -0.4f);
        b = Random.Range(-0.1f, 0.1f);
        c = Random.Range(-0.6f, -0.4f);
        //positionLeg1 = new Vector3(a, b, c); // default was: (-0.5f, 0, -0.5f);
        positionLeg1 = new Vector3(-0.5f, 0, -0.5f);

        // position the leg 2 with respect to the central body
        a = Random.Range(0.4f, 0.6f);
        b = Random.Range(-0.1f, 0.1f);
        c = Random.Range(-0.6f, -0.4f);
        //positionLeg2 = new Vector3(a, b, c); // default was: (0.5f, 0, -0.5f);
        positionLeg2 = new Vector3(0.5f, 0, -0.5f);

        // position the leg 3 with respect to the central body
        a = Random.Range(-0.6f, -0.4f);
        b = Random.Range(-0.1f, 0.1f);
        c = Random.Range(0.4f, 0.6f);
        //positionLeg3 = new Vector3(a, b, c); // default was: (-0.5f, 0,  0.5f);
        positionLeg3 = new Vector3(-0.5f, 0,  0.5f);

        // position the leg 4 with respect to the central body
        a = Random.Range(0.4f, 0.6f);
        b = Random.Range(-0.1f, 0.1f);
        c = Random.Range(0.4f, 0.6f);
        //positionLeg4 = new Vector3(a, b, c); // default was: (0.5f, 0,  0.5f);
        positionLeg4 = new Vector3(0.5f, 0,  0.5f);

        // position of the center of mass of the main body (the evolutionary process can change it to have a stronger balance)
        a = Random.Range(-0.2f, 0.2f);
        b = Random.Range(-0.2f, 0.2f);
        c = Random.Range(-0.2f, 0.2f);
        bodyCenterOfMass = new Vector3(a, b, c); // default was: Vector3.zero;

        // length of the upper part of the leg
        upperLen = Random.Range(0.1f, 1f);   // default was: 0.5

        // width of the upper part of the leg
        upperWidth = Random.Range(0.1f, 1f); // default was: 0.15

        // depth of the upper part of the leg
        upperDepth = Random.Range(0.1f, 1f); // default was: 0.15

        // length of the lower part of the leg
        lowerLen = Random.Range(0.1f, 1f);   // default was: 0.5

        // width of the lower part of the leg
        lowerWidth = Random.Range(0.1f, 1f); // default was: 0.15

        // depth of the lower part of the leg
        lowerDepth = Random.Range(0.1f, 1f); // default was: 0.15

        // the movement allowed of the length within the hip is a range: this is the MIN
        //limitsHipMin = Random.Range(-60f, -10f); // default was: -30f;
        limitsHipMin = -30f;

        // the movement allowed of the length within the hip is a range: this is the MAX
        //limitsHipMax = Random.Range(10f, 60f); // default was: 30f;
        limitsHipMax = 30f;

        // the movement allowed of the length within the knee is a range: this is the MIN
        //limitsKneeMin = Random.Range(-60f, -30f); // default was: -45f;
        limitsKneeMin = -45f;

        // the movement allowed of the length within the knee is a range: this is the MAX
        //limitsKneeMax = Random.Range(-15f, 10f); // default was: 0f;
        limitsKneeMax = 0f;

        // weight of the main body
        bodyMass = Random.Range(3f, 5f); // default was: 4f;



            genome = new List<object>()
            {
                force,                                       // float   [0]
                speed,                                       // float   [1]
                limitsHipMin,                                // float   [2]
                limitsHipMax,                                // float   [3]
                limitsKneeMin,                               // float   [4]
                limitsKneeMax,                               // float   [5]
                bodySize,                                    // Vector3 [6]
                positionLeg1,                                // Vector3 [7]
                positionLeg2,                                // Vector3 [8]
                positionLeg3,                                // Vector3 [9]
                positionLeg4,                                // Vector3 [10]
                bodyCenterOfMass,                            // Vector3 [11]
                upperLen,                                    // float   [12]
                upperWidth,                                  // float   [13]
                upperDepth,                                  // float   [14]
                lowerLen,                                    // float   [15]
                lowerWidth,                                  // float   [16]
                lowerDepth,                                  // float   [18]
                bodyMass
            };

            //Debug.Log($"PRIMO FILE - speed in the first file: {genome[1]}");
            //Debug.Log($"PRIMO FILE - upperLen: {upperLen} - upperWidth: {upperWidth} - upperDepth: {upperDepth}");
            //Debug.Log($"PRIMO FILE - lowerLen: {lowerLen} - lowerWidth: {lowerWidth} - lowerDepth: {lowerDepth}");
            //Debug.Log($"PRIMO FILE - genome[11]: {genome[11]} - genome[12]: {genome[12]} - genome[13]: {genome[13]}");
            //Debug.Log($"PRIMO FILE - genome[14]: {genome[14]} - genome[15]: {genome[15]} - genome[16]: {genome[16]}");

            // add the created genome to the list of genomes of the population
            genomes.Add(genome);

            // generate a creature for each genome in the list
            creature = new WalkerCreatureEnhanced4LegsWithSensors(new Vector3((i % 5) * 10f, 2, (i / 5) * 10f), genome);
            // change to (new Vector3(i * 10f, 2, 0), genome) if you want the creature in a line an not in lines of five elements each

            // add the generated creature to the population list
            population.Add(creature);

        }

        // Ignore collisions between creatures, so that their performance is not affected by random collisions with one another
        for (int i = 0; i < populationSize; i++)
        {
            for (int j = 0; j < populationSize; j++)
            {
                IgnoreCollisionsBetweenCreatures(population[i], population[j]);
            }
        }

        // after the generation of a new population, we reset the timer to start a new 30-second period
        timer = 0f;

    }


    // check the performance of the creatures and create a new generation
    void EvolvePopulation()
    {
        // for each creature of the population, save in a list distance, genome and id
        List<(float, List<object>, int)> scored = new List<(float, List<object>, int)>();
        for (int i = 0; i < population.Count; i++)
        {
            scored.Add((population[i].distanceTraveled, (List<object>)genomes[i], i));
        }

        scored.Sort((a, b) => b.Item1.CompareTo(a.Item1)); // sort the list by distance

        // Show the performance of the best performing creature
        Debug.Log($"Generation: {generation} - Winner: {scored[0].Item3} - Distance: {scored[0].Item1} - GENOME:: " +
          $"force: {scored[0].Item2[0]} | " +
          $"speed: {scored[0].Item2[1]} | " +
          $"limitsHipMin: {scored[0].Item2[2]} | " +
          $"limitsHipMax: {scored[0].Item2[3]} | " +
          $"limitsKneeMin: {scored[0].Item2[4]} | " +
          $"limitsKneeMax: {scored[0].Item2[5]} | " +
          $"bodySize: {scored[0].Item2[6]} | " +
          $"positionLeg1: {scored[0].Item2[7]} | " +
          $"positionLeg2: {scored[0].Item2[8]} | " +
          $"positionLeg3: {scored[0].Item2[9]} | " +
          $"\n" +
          $"positionLeg4: {scored[0].Item2[10]} | " +
          $"bodyCenterOfMass: {scored[0].Item2[11]} | " +
          $"upperLen: {scored[0].Item2[12]} | " +
          $"upperWidth: {scored[0].Item2[13]} | " +
          $"upperDepth: {scored[0].Item2[14]} | " +
          $"lowerLen: {scored[0].Item2[15]} | " +
          $"lowerWidth: {scored[0].Item2[16]} | " +
          $"lowerDepth: {scored[0].Item2[17]} | " +
          $"bodyMass: {scored[0].Item2[18]}");


        //remove all creatures
        for (int i = 0; i < population.Count; i++)
        {
            population[i].RemoveCreatures();
        }

        // clear the lists
        genomes.Clear();
        population.Clear();


        // ELITISM: clone the top performing two and then add another 18 (if the population is 20)
        for (int i = 0; i < 2; i++)
        {
            genomes.Add(scored[i].Item2);
        }


        // GENERATES THE OTHER 18 GENOMES FOR THE NEW POPULATION
        while (genomes.Count < populationSize)
        {
            //Picks a random genome from the top 5 scored creatures for the first parent
            var parent1 = scored[Random.Range(0, 5)].Item2;

            //Picks a random genome from the top 5 scored creatures for the second parent
            var parent2 = scored[Random.Range(0, 5)].Item2;
            
            float percentage;
            float percentage1;
            float percentage2;
            float percentage3;

            // Initializes an empty genome for the child
            List<object> child = new List<object>();
            float new_gene = 0;
            int gene_added = 0;
            float value01 = 0;
            float value02 = 0;
            float value03 = 0;
            Vector3 valueToAdd = new Vector3 (0f, 0f, 0f);

            // GENERATE A NEW GENOME, WITH CROSSOVER AND MUTATION
            for (int g = 0; g < parent1.Count; g++)
            {
                // CROSSOVER: each gene is ramdomly picked from either parent 1 or parent 2
                object child_gene = (Random.value > 0.5f ? parent1[g] : parent2[g]);   // crossover

                // MUTATION: 1 in 5 is also sligthly modified (by +/- 10%)
                if (Random.value < 0.2f) // 20% probability of mutation
                { 
                    if (g < 6 || g > 11) // The genes between before 6 and after 11 are float
                    { 
                        percentage = Random.Range(-10f, 10f) / 100; // percentage between -10% and +10%
                        new_gene = (1 + percentage) * (float)child_gene;
                        child.Add(new_gene);
                        gene_added = 1;     // if there's mutation we add the gene directly here or below
                    }

                    else        // the genes between 6 and 11 are of type Vector3
                    {
                        percentage1 = Random.Range(-10f, 10f) / 100; // percentage between -10% and +10%
                        percentage2 = Random.Range(-10f, 10f) / 100; // percentage between -10% and +10%
                        percentage3 = Random.Range(-10f, 10f) / 100; // percentage between -10% and +10%

                        //Debug.Log($"gene[{g}]: {child_gene}"); 

                        Vector3 geneVector = (Vector3)child_gene;
                        value01 = geneVector.x;
                        value02 = geneVector.y;
                        value03 = geneVector.z;
                        value01 = (1 + percentage1) * (float)value01;
                        value02 = (1 + percentage2) * (float)value02;
                        value03 = (1 + percentage3) * (float)value03;
                        valueToAdd = new Vector3 (value01, value02, value03);
                        child.Add(valueToAdd);
                        gene_added = 1;     // if there's mutation we add the gene directly above or here
                    }

                    
                }

                if (gene_added == 0) child.Add(child_gene); // if there was no mutation we add the gene here
                gene_added = 0; // reset the flag
            }
            genomes.Add(child); // add the entire genome of the child to the list of the genomes of the new population
        }


        // CREATES THE NEW POPULATION WITH THE LIST OF GENOMES
        for (int i = 0; i < populationSize; i++)
        {
            var creature = new WalkerCreatureEnhanced4LegsWithSensors(new Vector3((i % 5) * 10f, 2, (i / 5) * 10f), genomes[i]);
            // change to (new Vector3(i * 10f, 2, 0), genome) if you want the creature in a line an not in lines of five elements each

            // add the generated creature to the population list
            population.Add(creature);
        }

        generation++; // keep track of the number of generations
        timer = 0f;   // reset the timer at each new generation
    }


    // function to color a creature if we want to show particularly one one
    void ColorCreature(WalkerCreatureEnhanced4LegsWithSensors creature)
    {
        Color gold = new Color(1f, 0.84f, 0f); // gold color

        creature.root.GetComponent<Renderer>().material.color = gold;
        creature.upper.GetComponent<Renderer>().material.color = gold;
        creature.lower.GetComponent<Renderer>().material.color = gold;
    }


    // function to avoid collisions between creatures
    void IgnoreCollisionsBetweenCreatures(WalkerCreatureEnhanced4LegsWithSensors a, WalkerCreatureEnhanced4LegsWithSensors b)
    {
        // Get all colliders of creature a
        List<Collider> collidersA = new List<Collider>();
        collidersA.Add(a.root.GetComponent<Collider>());
        foreach (var upper in a.uppers)
            collidersA.Add(upper.GetComponent<Collider>());
        foreach (var lower in a.lowers)
            collidersA.Add(lower.GetComponent<Collider>());

        // Get all colliders of creature b
        List<Collider> collidersB = new List<Collider>();
        collidersB.Add(b.root.GetComponent<Collider>());
        foreach (var upper in b.uppers)
            collidersB.Add(upper.GetComponent<Collider>());
        foreach (var lower in b.lowers)
            collidersB.Add(lower.GetComponent<Collider>());

        // Disable collision between all parts of a and b
        foreach (var colA in collidersA)
        {
            foreach (var colB in collidersB)
            {
                Physics.IgnoreCollision(colA, colB);
            }
        }
    }





    public void CreateLight()
    {
        // Create a new GameObject to hold the Light component
        GameObject lightGameObject = new GameObject("GeneratedLight");

        // Add a Light component to the GameObject
        targetLight = lightGameObject.AddComponent<Light>();
        
        // Set up light properties
        targetLight.type = LightType.Point;        // Point light
        lightGameObject.transform.position = new Vector3(0, 5, 0);  // Position of light
        targetLight.color = Color.white;           // Light color
        targetLight.intensity = 1.0f;              // Light intensity
        targetLight.range = 10.0f;                 // Light range
        targetLight.shadows = LightShadows.Soft;   // Light shadows

        // Tag the GameObject for identification
        lightGameObject.tag = "TargetLight";       // This is for your sensor logic
    }
    
    public void UpdateLightSettings(Color newColor, float newIntensity, float newRange, Vector3 newPosition)
    {
        if (targetLight != null)
        {
            targetLight.color = newColor;
            targetLight.intensity = newIntensity;
            targetLight.range = newRange;
            targetLight.transform.position = newPosition;  // Position of light
        }
        else
        {
            Debug.LogWarning("Target light not set. Cannot update settings.");
        }
    }


}



