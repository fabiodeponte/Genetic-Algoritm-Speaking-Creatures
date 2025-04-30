using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.SceneManagement;


public class Insect : MonoBehaviour
{
    
    public int populationSize;
    public float evaluationTime;
    private float timer;

    private List<Creatures01_CreatureGenerator_PhysicalEvolve> population = new List<Creatures01_CreatureGenerator_PhysicalEvolve>();
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

    private Creatures01_CreatureGenerator_PhysicalEvolve creature;

    void Start()
    {
        // parameters
        populationSize = 1;  // size of the population
        evaluationTime = 300f; // 300 seconds per generation
        Time.timeScale = 1f;

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

        for (int i = 0; i < populationSize; i++) // for each creature, we generate a genome, meaning a certain combination of feature values
        {


            if (i==0) // generation 713        // galoppo
            {
                force = 144.7964f;
                speed = 171.1907f;
                limitsHipMin = -28.37243f;
                limitsHipMax = 59.63264f;
                limitsKneeMin = -72.76945f;
                limitsKneeMax = 0f;
                bodySize = new Vector3 (4.45f, 1.07f, 2.77f);
                positionLeg1 = new Vector3 (-0.72f, 0.00f, -1.07f);
                positionLeg2 = new Vector3 (0.89f, 0.00f, -0.67f);
                positionLeg3 = new Vector3 (-0.79f, 0.00f, 0.31f);
                positionLeg4 = new Vector3 (0.26f, 0.00f, 0.66f);
                bodyCenterOfMass = new Vector3 (-0.04f, -0.18f, 0.06f);
                upperLen = 1.741453f;
                upperWidth = 0.4146942f;
                upperDepth = 1.263755f;
                lowerLen = 0.8547196f;
                lowerWidth = 0.5039697f;
                lowerDepth = 0.1444479f;
                bodyMass = 0.9835867f;
            }

            if (i==1) // generation 853         // sbilenco
            {
                force = 161.5651f;
                speed = 200.8626f;
                limitsHipMin = -30.16429f;
                limitsHipMax = 53.61658f;
                limitsKneeMin = -73.16537f;
                limitsKneeMax = 0f;
                bodySize = new Vector3(6.33f, 0.60f, 3.24f);
                positionLeg1 = new Vector3(-0.80f, 0.00f, -0.99f);
                positionLeg2 = new Vector3(0.75f, 0.00f, -0.94f);
                positionLeg3 = new Vector3(-0.77f, 0.00f, 0.48f);
                positionLeg4 = new Vector3(0.16f, 0.00f, 0.73f);
                bodyCenterOfMass = new Vector3(-0.03f, -0.13f, 0.06f);
                upperLen = 4.029487f;
                upperWidth = 0.50566f;
                upperDepth = 1.51613f;
                lowerLen = 0.8920193f;
                lowerWidth = 0.3546988f;
                lowerDepth = 0.1299367f;
                bodyMass = 0.7242252f;
            }

            if (i==2) // generation 1500           // eplode
            {
                force = 159.8491f;
                speed = 74.08461f;
                limitsHipMin = -14.30097f;
                limitsHipMax = 9.05621f;
                limitsKneeMin = -65.33143f;
                limitsKneeMax = 0f;
                bodySize = new Vector3(13.72f, 0.20f, 1.36f);
                positionLeg1 = new Vector3(-0.54f, 0.00f, -0.72f);
                positionLeg2 = new Vector3(1.96f, 0.00f, -0.64f);
                positionLeg3 = new Vector3(-3.27f, 0.00f, 0.44f);
                positionLeg4 = new Vector3(0.17f, 0.00f, 0.41f);
                bodyCenterOfMass = new Vector3(-0.02f, -0.21f, 0.12f);
                upperLen = 2.210491f;
                upperWidth = 0.3828156f;
                upperDepth = 0.7657632f;
                lowerLen = 0.6603064f;
                lowerWidth = 0.4221073f;
                lowerDepth = 0.1536982f;
                bodyMass = 0.1077927f;
            }

            if (i==3) // generation 2700            // esplode
            {
                force = 58.53562f;
                speed = 22.14372f;
                limitsHipMin = -4.498312f;
                limitsHipMax = 3.02033f;
                limitsKneeMin = -18.21725f;
                limitsKneeMax = 0f;
                bodySize = new Vector3(5.41f, 0.11f, 0.20f);
                positionLeg1 = new Vector3(-0.20f, 0.00f, -0.58f);
                positionLeg2 = new Vector3(10.02f, 0.00f, -0.33f);
                positionLeg3 = new Vector3(-1.29f, 0.00f, 0.13f);
                positionLeg4 = new Vector3(0.69f, 0.00f, 0.05f);
                bodyCenterOfMass = new Vector3(-0.01f, -0.02f, 0.07f);
                upperLen = 0.2677372f;
                upperWidth = 0.1214893f;
                upperDepth = 0.3449826f;
                lowerLen = 0.6414126f;
                lowerWidth = 0.03772715f;
                lowerDepth = 0.03912784f;
                bodyMass = 0.06885628f;
            } 





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
            creature = new Creatures01_CreatureGenerator_PhysicalEvolve(new Vector3((i % 5) * 10f, 3, (i / 5) * 10f), genome);
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
            var creature = new Creatures01_CreatureGenerator_PhysicalEvolve(new Vector3((i % 5) * 10f, 3, (i / 5) * 10f), genomes[i]);
            // change to (new Vector3(i * 10f, 2, 0), genome) if you want the creature in a line an not in lines of five elements each

            // add the generated creature to the population list
            population.Add(creature);
        }

        generation++; // keep track of the number of generations
        timer = 0f;   // reset the timer at each new generation
    }


    // function to color a creature if we want to show particularly one one
    void ColorCreature(Creatures01_CreatureGenerator_PhysicalEvolve creature)
    {
        Color gold = new Color(1f, 0.84f, 0f); // gold color

        creature.root.GetComponent<Renderer>().material.color = gold;
        creature.upper.GetComponent<Renderer>().material.color = gold;
        creature.lower.GetComponent<Renderer>().material.color = gold;
    }


    // function to avoid collisions between creatures
    void IgnoreCollisionsBetweenCreatures(Creatures01_CreatureGenerator_PhysicalEvolve a, Creatures01_CreatureGenerator_PhysicalEvolve b)
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
