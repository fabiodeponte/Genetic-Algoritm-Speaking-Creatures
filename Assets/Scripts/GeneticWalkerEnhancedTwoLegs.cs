using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.SceneManagement;




public class GeneticWalkerEnhancedTwoLegs : MonoBehaviour
{
    
    public int populationSize;
    public float evaluationTime;
    private float timer;

    private List<WalkerCreatureEnhancedTwoLegs> population = new List<WalkerCreatureEnhancedTwoLegs>();
    private List<object> genome = new List<object>();
    private List<List<object>> genomes = new List<List<object>>();
    private int generation = 0;

    public GameObject body;



private Vector3 bodySize;
private Vector3 positionLeg1;
private Vector3 positionLeg2;
private Vector3 positionArm1;
private Vector3 positionArm2;

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


    void Start()
    {
        //body = GameObject.CreatePrimitive(PrimitiveType.Cube);
        //Destroy(body);

        populationSize = 20;
        evaluationTime = 30f;

        //accelerates time x10
        Time.timeScale = 10f; 

        GenerateInitialPopulation();
    }

    void Update()
    {
        timer += Time.deltaTime; // deltaTime is the time passed since past frame
        foreach (var creature in population)
        {
            creature.UpdateMotion(Time.time);
        }

        //Debug.Log($"Timer: {timer}");
        //Debug.Log($"evaluationTime: {evaluationTime}");

        if (timer >= evaluationTime)
        {
            EvolvePopulation();
        }
    }


    void GenerateInitialPopulation()
    {

        float a = 0;
        float b = 0;
        float c = 0;

        for (int i = 0; i < populationSize; i++)
        {


        force = Random.Range(50f, 300f);
        speed = Random.Range(30f, 70f);

        a = Random.Range(1f, 2f);
        b = Random.Range(1f, 2f);
        c = Random.Range(1f, 2f);
        //bodySize = new Vector3(a, b, c); // default was: (1, 0.4f, 1);
        bodySize = new Vector3(0.4f, 1, 1);

        positionLeg1 = new Vector3(0, 0, -0.5f);
        positionLeg2 = new Vector3(0, 0, 0.5f);
        positionArm1 = new Vector3(0, 1f, -0.5f);
        positionArm2 = new Vector3(0, 1f, 0.5f);


        a = Random.Range(-0.2f, 0.2f);
        b = Random.Range(-0.2f, 0.2f);
        c = Random.Range(-0.2f, 0.2f);
        bodyCenterOfMass = new Vector3(a, b, c); // default was: Vector3.zero;

        upperLen = Random.Range(0.1f, 1f);   // default was: 0.5
        //upperLen=0.5f;

        upperWidth = Random.Range(0.1f, 1f); // default was: 0.15
        //upperWidth = 0.15f;

        upperDepth = Random.Range(0.1f, 1f); // default was: 0.15
        //upperDepth = 0.15f;

        lowerLen = Random.Range(0.1f, 1f);   // default was: 0.5
        lowerWidth = Random.Range(0.1f, 1f); // default was: 0.15
        lowerDepth = Random.Range(0.1f, 1f); // default was: 0.15

        //limitsHipMin = Random.Range(-60f, -10f); // default was: -30f;
        limitsHipMin = -30f;

        //limitsHipMax = Random.Range(10f, 60f); // default was: 30f;
        limitsHipMax = 30f;

        //limitsKneeMin = Random.Range(-60f, -30f); // default was: -45f;
        limitsKneeMin = -45f;

        //limitsKneeMax = Random.Range(-15f, 10f); // default was: 0f;
        limitsKneeMax = 0f;

        bodyMass = Random.Range(0.5f, 1.5f); // default was: 4f;



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
                positionArm1,                                // Vector3 [9]
                positionArm2,                                // Vector3 [10]
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

            genomes.Add(genome);

            //var creature = new WalkerCreatureEnhancedTwoLegs(new Vector3(i * 10f, 2, 0), genome);
            var creature = new WalkerCreatureEnhancedTwoLegs(new Vector3((i % 5) * 10f, 2f, (i / 5) * 10f), genome);
            population.Add(creature);
        }
        timer = 0f;


        // Ignore collisions between creatures
        for (int i = 0; i < populationSize; i++)
        {
            for (int j = 0; j < populationSize; j++)
            {
                IgnoreCollisionsBetweenCreatures(population[i], population[j]);
            }
        }


    }

    void EvolvePopulation()
    {

        // Save distance, genome and id creature
        List<(float, List<object>, int)> scored = new List<(float, List<object>, int)>();
        for (int i = 0; i < population.Count; i++)
        {
            scored.Add((population[i].distanceTraveled, (List<object>)genomes[i], i));

            //Debug.Log($"Distance traveled by creature {i}: {population[i].distanceTraveled}");
        }

        scored.Sort((a, b) => b.Item1.CompareTo(a.Item1)); // sort by distance
        ///Debug.Log($"Generation {generation} - Highest distance: {scored[0].Item1} - Second: {scored[1].Item1}");

        // color in gold the fastest creature
        //ColorFastestCreature(population[scored[0].Item3]);

        // Freeze 2 seconds
        //StartCoroutine(FreezeTime(2f));

        //remove creatures of the old generation
        for (int i = 0; i < population.Count; i++)
        {
            population[i].RemoveCreatures();
        }


        genomes.Clear();
        population.Clear();

        // Elitism: keep top 2
        for (int i = 0; i < 2; i++)
        {
            genomes.Add(scored[i].Item2);

        }

        //Debug.Log($"Generation: {generation} - Winner: {scored[0].Item3} - Distance: {scored[0].Item1} - Genome: {scored[0].Item2[0]} | {scored[0].Item2[1]} | {scored[0].Item2[2]}");
        //Debug.Log($"Generation: {generation} - Winner: {scored[0].Item3} - Distance: {scored[0].Item1} - GENOME:: force: {scored[0].Item2[0]} | speed: {scored[0].Item2[1]} | limitsHipMin: {scored[0].Item2[2]} | {scored[0].Item2[3]} | {scored[0].Item2[4]} | {scored[0].Item2[5]} | {scored[0].Item2[6]} | {scored[0].Item2[7]} | {scored[0].Item2[8]} | {scored[0].Item2[9]} | {scored[0].Item2[10]} | {scored[0].Item2[11]} | {scored[0].Item2[12]} | {scored[0].Item2[13]} | {scored[0].Item2[14]} | {scored[0].Item2[15]} | {scored[0].Item2[16]}");
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
          $"positionArm1: {scored[0].Item2[9]} | " +
          $"\n" +
          $"positionArm2: {scored[0].Item2[10]} | " +
          $"bodyCenterOfMass: {scored[0].Item2[11]} | " +
          $"upperLen: {scored[0].Item2[12]} | " +
          $"upperWidth: {scored[0].Item2[13]} | " +
          $"upperDepth: {scored[0].Item2[14]} | " +
          $"lowerLen: {scored[0].Item2[15]} | " +
          $"lowerWidth: {scored[0].Item2[16]} | " +
          $"lowerDepth: {scored[0].Item2[17]} | " +
          $"bodyMass: {scored[0].Item2[18]}");

        // GENERATES THE GENOMES FOR THE NEW POPULATION
        while (genomes.Count < populationSize)
        {

            
            //Picks a random genome from the top 5 scored creatures for the first parent
            var parent1 = scored[Random.Range(0, 5)].Item2;

            //Picks a random genome from the top 5 scored creatures for the second parent
            var parent2 = scored[Random.Range(0, 5)].Item2;
            

            // select the best performing two as parents of the next generation
            //var parent1 = scored[0].Item2; //genome parent 1
            //var parent2 = scored[1].Item2; //genome parent 2
            float percentage;
            float percentage1;
            float percentage2;
            float percentage3;
            //Console.WriteLine("Generation: " + generation + " - Winner: " + scored[0].Item1 + " - Score: " + scored[0].Item2 + " - Genome: " + scored[0].Item3[0] + " | " + scored[0].Item3[1] + " | " + scored[0].Item3[2]);
            
            //Debug.Log($"Generation {generation} - Highest distance: {scored[0].Item1} - Second: {scored[1].Item1}");
            //Debug.Log($"Generation: {generation} - Winner: {scored[0].Item3} - Distance: {scored[0].Item1} - Genome: {scored[0].Item2[0]} | {scored[0].Item2[1]} | {scored[0].Item2[2]}");

            //Initializes an empty genome for the child (with 3 genes: force, speed and phase)
            List<object> child = new List<object>();
            float new_gene = 0;
            int gene_added = 0;
            float value01 = 0;
            float value02 = 0;
            float value03 = 0;
            Vector3 valueToAdd = new Vector3 (0f, 0f, 0f);


            for (int g = 0; g < parent1.Count; g++)
            {
                // CROSSOVER
                // each gene is ramdomly picked from either parent 1 or parent 2
                object child_gene = (Random.value > 0.5f ? parent1[g] : parent2[g]);   // crossover

                //Debug.Log($"gene[{g}]: {child_gene.GetType()}"); 


                // MUTATION
                // 1 in 5 is also sligthly modified (adding a value ranging from -10 to +10)

                if (Random.value < 0.2f)
                { 
                    if (g < 6 || g > 11)
                    { 
                        percentage = Random.Range(-10f, 10f) / 100; // percentage between -10% and +10%
                        new_gene = (1 + percentage) * (float)child_gene;
                        child.Add(new_gene);
                        gene_added = 1;
                    }

                    else
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
                        gene_added = 1;


                    }

                    
                }

                if (gene_added == 0) child.Add(child_gene);
                gene_added = 0;

            }


            genomes.Add(child);
        }


        // CREATES THE NEW POPULATION
        for (int i = 0; i < populationSize; i++)
        {
            //var creature = new WalkerCreatureEnhancedTwoLegs(new Vector3(i * 3f, 2, 0), genomes[i]);
            var creature = new WalkerCreatureEnhancedTwoLegs(new Vector3((i % 5) * 10f, 2f, (i / 5) * 10f), genomes[i]);
            population.Add(creature);
        }

        generation++;
        timer = 0f;


        for (int i = 0; i < populationSize; i++)
        {
            for (int j = 0; j < populationSize; j++)
            {
                IgnoreCollisionsBetweenCreatures(population[i],population[j]);
            }
        }

    }



    void ColorFastestCreature(WalkerCreatureEnhancedTwoLegs creature)
    {
        Color gold = new Color(1f, 0.84f, 0f); // gold color

        creature.root.GetComponent<Renderer>().material.color = gold;
        creature.upper.GetComponent<Renderer>().material.color = gold;
        creature.lower.GetComponent<Renderer>().material.color = gold;
    }






void IgnoreCollisionsBetweenCreatures(WalkerCreatureEnhancedTwoLegs a, WalkerCreatureEnhancedTwoLegs b)
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


}
