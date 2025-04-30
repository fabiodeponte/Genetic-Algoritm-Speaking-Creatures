/*
DA FARE: per qualche motivo non si muovono più. pubblica a video il valore degli output della rete neurale del vincente
alla fine di ogni generazione, così da vedere cosa arriva effettivamente ai motori.

*/


using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.SceneManagement;
using NUnit.Framework.Constraints;


public class GeneralConfigurationParameters
{
    // GENERAL PARAMETERS
    public static int populationSize = 100;     // size of the population
    public static float evaluationTime = 10f;   // 30 seconds per generation
    public static float timeScale = 5f;         // accelerates time x10 

    // RANGE OF FORCE AND SPEED FOR THE LEGS OF THE AGENTS
    public static float min_force = 50f;
    public static float max_force = 300f;
    public static float min_speed = 30f;
    public static float max_speed = 70f;
 
    // LIGHT PARAMETERS
    public static Color lightColor = Color.white;     // Light color
    public static float lightIntensity = 25.0f;       // Light intensity
    public static float lightRange = 10.0f;           // Light Range: this is not relevant - the distance is calculated between the position of the light and of the agent, no matter what the lightRange is
    
    // LIGHT POSITION (RANDOM RANGE) - 2 for Y (slightly above creatures) and from -100 to +100 for X and Z
    public static int lightPosition_range_min_X = -100;
    public static int lightPosition_range_max_X = 100;
    public static int lightPosition_range_min_Y = 2;
    public static int lightPosition_range_max_Y = 2;
    public static int lightPosition_range_min_Z = -100;
    public static int lightPosition_range_max_Z = 100; 

    // LIGHT SENSOR PARAMETERS
    public static float lightSensorDetectionRange = 500f;   // This is the relevant variable to define how far the agent can see the light
    public static float lightSensorAngleSensitivity = 180f; // Sensors see angles from -180 to +180 degrees

    // GENOMES
    public static string physicalInitialGenome = "Duck";
    // Possible alternatives:
    //  - Random: every creature has a random genome
    //  - GallopingSavannah: every creature shares the physical genome of GallopingSavannah
    //  - Bison: every creature shares the physical genome of Bison
    //  - Duck: every creature shares the physical genome of Duck

    // STOP PHYSICAL MUTATION
    public static bool freezePhysicalEvolution = true; // stop mutation of the physical genome (still does crossover)

    // INITIAL DISPOSITION OF THE AGENTS ON THE GROUND
    public static Vector3 ComputePosition(int i)
    {
        return new Vector3((i % 5) * 10f, 2f, (i / 5) * 10f); // in lines of 5, at a distance of 10 from each other
        // change to new Vector3(i * 10f, 2, 0) if you want the creature in a line an not in lines of five elements each
    }

}

public class Creatures02_PopulationManager_Behavioural_NN : MonoBehaviour
{
    
    public int populationSize;
    public float evaluationTime;
    private float timer;

    private List<Creatures02_CreatureGenerator_Behavioural_NN> population = new List<Creatures02_CreatureGenerator_Behavioural_NN>();
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

    private Creatures02_CreatureGenerator_Behavioural_NN creature;

    public Light targetLight;

    public Genome genomeNN;

    public static float min_force = GeneralConfigurationParameters.min_force;
    public static float max_force = GeneralConfigurationParameters.max_force;
    public static float min_speed = GeneralConfigurationParameters.min_speed;
    public static float max_speed = GeneralConfigurationParameters.max_speed;
 
    Color lightColor = GeneralConfigurationParameters.lightColor;
    float lightIntensity = GeneralConfigurationParameters.lightIntensity;
    float lightRange = GeneralConfigurationParameters.lightRange;
    Vector3 newPosition;


    void Start()
    {
        // parameters
        populationSize = GeneralConfigurationParameters.populationSize;  // size of the population
        evaluationTime = GeneralConfigurationParameters.evaluationTime; // 30 seconds per generation
        Time.timeScale = GeneralConfigurationParameters.timeScale; // accelerates time x10 

        //create a light
        newPosition = new Vector3(Random.Range(GeneralConfigurationParameters.lightPosition_range_min_X, GeneralConfigurationParameters.lightPosition_range_max_X), Random.Range(GeneralConfigurationParameters.lightPosition_range_min_Y, GeneralConfigurationParameters.lightPosition_range_max_Y), Random.Range(GeneralConfigurationParameters.lightPosition_range_min_Z, GeneralConfigurationParameters.lightPosition_range_max_Z));
        CreateLight(lightColor, lightIntensity, lightRange, newPosition);

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
        
        // initialize Genomes
        switch (GeneralConfigurationParameters.physicalInitialGenome)
        {
            case "Random":
                GenomeRandomInitialization();
                break;

            case "GallopingSavannah":
                GenomeInitializeGallopingSavannah();
                break;

            case "Bison":
                GenomeInitializeBison();
                break;

            case "Duck":
                GenomeInitializeDuck();
                break;

            default:
                GenomeRandomInitialization();
                break;
        }

        
        for (int i = 0; i < populationSize; i++) // for each creature, we generate a genome, meaning a certain combination of feature values
        {
            //GenomeRandomInitialization(); // generates random values for the variables that are used below


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
                lowerDepth,                                  // float   [17]
                bodyMass,                                    // float   [18]
                genomeNN                                     // float[] [19]
            };

            // add the created genome to the list of genomes of the population
            genomes.Add(genome);

            // generate a creature for each genome in the list
            creature = new Creatures02_CreatureGenerator_Behavioural_NN(GeneralConfigurationParameters.ComputePosition(i), genome);

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


    //Debug.Log($"genome.Count: {genomes.Count}");
    
    //Debug.Log($"Generation: {generation} -

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


        var message = $"";

        // Show the performance of the best performing creature
        var message01 = $"Generation: {generation} - Winner: {scored[0].Item3} - Distance: {scored[0].Item1} \n";

        var message02 = $"PHYSICAL GENOME: " +
          //$"force: {scored[0].Item2[0]} | " +         // force and speed are governed by the NN
          //$"speed: {scored[0].Item2[1]} | " +         // force and speed are governed by the NN
          $"limitsHipMin: {scored[0].Item2[2]} | " +
          $"limitsHipMax: {scored[0].Item2[3]} | " +
          $"limitsKneeMin: {scored[0].Item2[4]} | " +
          $"limitsKneeMax: {scored[0].Item2[5]} | " +
          $"bodySize: {scored[0].Item2[6]} | " +
          $"positionLeg1: {scored[0].Item2[7]} | " +
          $"positionLeg2: {scored[0].Item2[8]} | " +
          $"positionLeg3: {scored[0].Item2[9]} | ";
        message02 += $"\n";
        message02 += $"positionLeg4: {scored[0].Item2[10]} | " +
          $"bodyCenterOfMass: {scored[0].Item2[11]} | " +
          $"upperLen: {scored[0].Item2[12]} | " +
          $"upperWidth: {scored[0].Item2[13]} | " +
          $"upperDepth: {scored[0].Item2[14]} | " +
          $"lowerLen: {scored[0].Item2[15]} | " +
          $"lowerWidth: {scored[0].Item2[16]} | " +
          $"lowerDepth: {scored[0].Item2[17]} | " +
          $"bodyMass: {scored[0].Item2[18]} | ";
          //$"genomeNN: {scored[0].Item2[19]}";         // displayed in detailed below

       Debug.Log(message01);
       Debug.Log(message02);

       var message03 = $"BEHAVIOURAL GENOME - WEIGHTS OF THE NEURAL NETWORK: \n";
       Debug.Log(message03);

        // The weights of the neural network
        float[][][] weights = population[scored[0].Item3].genomeNN.ReturnWeights();
        for (int layer = 0; layer < weights.Length; layer++)
        {
            var message04 = $"Layer {layer}:";
            for (int i = 0; i < weights[layer].Length; i++)
            {
                string rowValues = "";
                for (int j = 0; j < weights[layer][i].Length; j++)
                {
                    rowValues += weights[layer][i][j].ToString("F4") + " ";
                }
                message04 += $" Weights Neuron {i}: {rowValues}";
            }
            Debug.Log(message04);
        }

        var message05 = $"FORCE AND SPEED DETERMINED BY THE NEURAL NETWORK: \n";
        // print out force and speed produced by the neural network
        message05 += $"Force 1: {population[scored[0].Item3].lastOutputsOfTheSingleExperiment[0]} | "+
          $"Speed 1: {population[scored[0].Item3].lastOutputsOfTheSingleExperiment[1]} | "+
          $"Force 2: {population[scored[0].Item3].lastOutputsOfTheSingleExperiment[2]} | "+
          $"Speed 2: {population[scored[0].Item3].lastOutputsOfTheSingleExperiment[3]} | "+
          $"Force 3: {population[scored[0].Item3].lastOutputsOfTheSingleExperiment[4]} | "+
          $"Speed 3: {population[scored[0].Item3].lastOutputsOfTheSingleExperiment[5]} | "+
          $"Force 4: {population[scored[0].Item3].lastOutputsOfTheSingleExperiment[6]} | "+
          $"Speed 4: {population[scored[0].Item3].lastOutputsOfTheSingleExperiment[7]}";




        Debug.Log(message05);
        Debug.Log($"=============================");


/*
        for (int layer = 0; layer < weights.Length; layer++)
        {
            Debug.Log($"Layer {layer}:");
            for (int i = 0; i < weights[layer].Length; i++)
            {
                string rowValues = "";
                for (int j = 0; j < weights[layer][i].Length; j++)
                {
                    rowValues += weights[layer][i][j].ToString("F4") + " ";
                }
                Debug.Log($"  Neuron {i}: {rowValues}");
            }
        }
*/



/*
        for (int i=0; i < population[scored[0].Item3].lastOutputsOfTheSingleExperiment.Length; i++)
        {
            Debug.Log($"lastOutputsOfTheSingleExperiment[{i}]: {population[scored[0].Item3].lastOutputsOfTheSingleExperiment[i]}");
        }
*/

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

            // Debug.LogError($"============================================================================== MUTATION - parent1.Count: {parent1.Count}");

            // GENERATE A NEW GENOME, WITH CROSSOVER AND MUTATION
            for (int g = 0; g < parent1.Count; g++)
            {
                // CROSSOVER: each gene is ramdomly picked from either parent 1 or parent 2
                object child_gene = Random.value > 0.5f ? parent1[g] : parent2[g];   // crossover

                // Debug.LogError($"============================================================================== g: {g} - child_gene.GetType(): {child_gene.GetType()}");

                if (Random.value < 0.5f &&  g == 19) // mutate one in two
                {
                    ((Genome)child_gene).MutateNetworkWeights();                    
                }

                // MUTATION: 1 in 5 is also sligthly modified (by +/- 10%)
                if (Random.value < 0.2f && GeneralConfigurationParameters.freezePhysicalEvolution == false) // 20% probability of mutation
                { 
                    if (g == 19)
                    { 
                        //new_gene = NeuralNetwork.MutateNetworkWeights(child_gene.ReturnWeights(), -0.1f, 0.1f);

                        //((Genome)child_gene).MutateNetworkWeights();

                        ////  **********************
                        /// ////  **********************
                        /// ////  **********************
                        /// ////  **********************
                        /// ////  **********************
                        /// ////  **********************
                        /// ////  **********************
                        /// ////  **********************
                        /// ////  **********************
                        /// ////  **********************
                        /// ////  **********************
                        /// ////  **********************
                        /// ////  **********************
                        /// ////  **********************
                        /// ////  **********************
                        /// ////  **********************
                        /// ////  **********************

                        //float[][][] a = child_gene.ReturnWeights();
                        //child.Add(new_gene);
                        //Debug.LogError($"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ g: {g} - child_gene.GetType(): {child_gene.GetType()}");
                        //percentage1 = Random.Range(-10f, 10f) / 100; // percentage between -10% and +10%
                        // HERE THE MUTATION OF THE GENOME OF THE NEURAL NETWORK
                    }

                    else if (g < 6 || g > 11) // The genes between before 6 and after 11 are float
                    { 
                        percentage = Random.Range(-10f, 10f) / 100; // percentage between -10% and +10%
                        new_gene = (1 + percentage) * (float)child_gene;
                        child.Add(new_gene);
                        gene_added = 1;     // if there's mutation we add the gene directly here or below
                    }

                    else    // the genes between 6 and 11 are of type Vector3                           
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
            var creature = new Creatures02_CreatureGenerator_Behavioural_NN(GeneralConfigurationParameters.ComputePosition(i), genomes[i]);
            // change to (new Vector3(i * 10f, 2, 0), genome) if you want the creature in a line an not in lines of five elements each

            // add the generated creature to the population list
            population.Add(creature);
        }

        generation++; // keep track of the number of generations
        timer = 0f;   // reset the timer at each new generation

        // change light position
        newPosition = new Vector3(Random.Range(GeneralConfigurationParameters.lightPosition_range_min_X, GeneralConfigurationParameters.lightPosition_range_max_X), Random.Range(GeneralConfigurationParameters.lightPosition_range_min_Y, GeneralConfigurationParameters.lightPosition_range_max_Y), Random.Range(GeneralConfigurationParameters.lightPosition_range_min_Z, GeneralConfigurationParameters.lightPosition_range_max_Z));
        UpdateLightSettings(lightColor, lightIntensity, lightRange, newPosition);
    }


    // function to color a creature if we want to show particularly one one
    void ColorCreature(Creatures02_CreatureGenerator_Behavioural_NN creature)
    {
        Color gold = new Color(1f, 0.84f, 0f); // gold color

        creature.root.GetComponent<Renderer>().material.color = gold;
        creature.upper.GetComponent<Renderer>().material.color = gold;
        creature.lower.GetComponent<Renderer>().material.color = gold;
    }


    // function to avoid collisions between creatures
    void IgnoreCollisionsBetweenCreatures(Creatures02_CreatureGenerator_Behavioural_NN a, Creatures02_CreatureGenerator_Behavioural_NN b)
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



    public void GenomeRandomInitialization()
    {

        force = Random.Range(min_force, max_force); // force    // discarded
        speed = Random.Range(min_speed, max_speed); // speed    // discarded

        // These are helpful later in dealing with tuple of three values of type Vector3
        float a = 0;
        float b = 0;
        float c = 0;

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

        // weight of the neural network
        float[][][] weights2save = WeightsInitialization();
        genomeNN = new Genome();
        genomeNN.SaveWeights(weights2save);
    }


    public void GenomeInitializeGallopingSavannah()
    {
                force = 0f; // discarded
                speed = 0f; // discarded
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

                // weight of the neural network
                float[][][] weights2save = WeightsInitialization();
                genomeNN = new Genome();
                genomeNN.SaveWeights(weights2save);
    }

    public void GenomeInitializeBison()
    {
                force = 304.3004f;
                speed = 113.6859f;
                limitsHipMin = -45.15137f;
                limitsHipMax = 37.58433f;
                limitsKneeMin = -67.44538f;
                limitsKneeMax = 0f;
                bodySize = new Vector3 (5.24f, 2.52f, 2.59f);
                positionLeg1 = new Vector3 (-0.63f, 0.00f, -0.73f);
                positionLeg2 = new Vector3 (0.76f, 0.00f, -0.62f);
                positionLeg3 = new Vector3 (-0.65f, 0.00f, 0.92f);
                positionLeg4 = new Vector3 (0.74f, 0.00f, 0.68f);
                bodyCenterOfMass = new Vector3 (0.09f, 0.05f, 0.17f);
                upperLen = 2.790962f;
                upperWidth = 0.7968422f;
                upperDepth = 0.3585646f;
                lowerLen = 0.09821696f;
                lowerWidth = 0.3150967f;
                lowerDepth = 0.7176875f;
                bodyMass = 1.764475f;

                // weight of the neural network
                float[][][] weights2save = WeightsInitialization();
                genomeNN = new Genome();
                genomeNN.SaveWeights(weights2save);
    }


    public void GenomeInitializeDuck()
    {
                force = 207.6744f;
                speed = 93.91657f;
                limitsHipMin = -55.10178f;
                limitsHipMax = 31.81556f;
                limitsKneeMin = -48.17958f;
                limitsKneeMax = 0f;
                bodySize =  new Vector3(2.44f, 2.54f,  1.44f) ;
                positionLeg1 =  new Vector3(-1.40f, 0.00f, -0.36f) ;
                positionLeg2 =  new Vector3(1.01f, 0.00f, -0.56f) ;
                positionLeg3 =  new Vector3(-0.82f, 0.00f, 0.54f) ;
                positionLeg4 =  new Vector3(0.49f, 0.00f, 0.48f) ;
                bodyCenterOfMass =  new Vector3(-0.07f, -0.02f, 0.00f) ;
                upperLen =  2.034411f;
                upperWidth =  0.2498355f;
                upperDepth =  0.3001766f;
                lowerLen =  0.1985095f;
                lowerWidth =  0.826129f;
                lowerDepth =  1.297097f;
                bodyMass =  1.468854f;

                // weight of the neural network
                float[][][] weights2save = WeightsInitialization();
                genomeNN = new Genome();
                genomeNN.SaveWeights(weights2save);
    }




    public float[][][] WeightsInitialization()
    {
        float[][][] weights2return = new float[][][]
            {
                // Input layer to hidden layer (2 → 8)
                new float[][]
                {
                    new float[] { Random.Range(0f, 1f), Random.Range(0f, 1f) },
                    new float[] { Random.Range(0f, 1f), Random.Range(0f, 1f) },
                    new float[] { Random.Range(0f, 1f), Random.Range(0f, 1f) },
                    new float[] { Random.Range(0f, 1f), Random.Range(0f, 1f) },
                    new float[] { Random.Range(0f, 1f), Random.Range(0f, 1f) },
                    new float[] { Random.Range(0f, 1f), Random.Range(0f, 1f) },
                    new float[] { Random.Range(0f, 1f), Random.Range(0f, 1f) },
                    new float[] { Random.Range(0f, 1f), Random.Range(0f, 1f) }
                },

                // Hidden layer to output layer (8 → 8)
                new float[][]
                {
                    new float[] { Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f),
                                Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f) },
                    new float[] { Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f),
                                Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f) },
                    new float[] { Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f),
                                Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f) },
                    new float[] { Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f),
                                Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f) },
                    new float[] { Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f),
                                Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f) },
                    new float[] { Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f),
                                Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f) },
                    new float[] { Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f),
                                Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f) },
                    new float[] { Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f),
                                Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f) }
                }
            };
        return weights2return;
    }


    public void CreateLight(Color newColor, float newIntensity, float newRange, Vector3 newPosition)
    {
        // Create a new GameObject to hold the Light component
        GameObject lightGameObject = new GameObject("GeneratedLight");

        // Add a Light component to the GameObject
        targetLight = lightGameObject.AddComponent<Light>();
        
        // Set up light properties
        targetLight.type = LightType.Point;        // Point light
        lightGameObject.transform.position = newPosition;  // Position of light
        targetLight.color = newColor;           // Light color
        targetLight.intensity = newIntensity;              // Light intensity
        targetLight.range = newRange;                 // Light range
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






