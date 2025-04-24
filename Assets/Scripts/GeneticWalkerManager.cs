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

public class GeneticWalkerManager : MonoBehaviour
{
    public int populationSize;
    public float evaluationTime;
    private float timer;

    private List<WalkerCreature> population = new List<WalkerCreature>();
    private List<float[]> genomes = new List<float[]>();
    private int generation = 0;

    public GameObject body;

    //public Vector3 StartPosition = new Vector3(0, 0, 0);
    // OLD START POSITION: new Vector3(i * 3f, 2, 0)


    void Start()
    {
        //body = GameObject.CreatePrimitive(PrimitiveType.Cube);
        //Destroy(body);

        populationSize = 10;
        evaluationTime = 30f;

        //accelerates time x10
        Time.timeScale = 1f; 

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
        for (int i = 0; i < populationSize; i++)
        {
            float[] genome = new float[]
            {
                Random.Range(50f, 300f),        // force
                Random.Range(30f, 180f),        // speed
                Random.Range(0f, Mathf.PI * 2f) // phase
            };
            genomes.Add(genome);
            var creature = new WalkerCreature(new Vector3(i * 3f, 2, 0), genome);
            population.Add(creature);
        }
        timer = 0f;
/*
        for (int i = 0; i < populationSize; i++)
        {
            for (int j = 0; j < populationSize; j++)
            {
                IgnoreCollisionsBetweenCreatures(population[i], population[j]);
            }
        }
*/
    }

    void EvolvePopulation()
    {

        // Save distance, genome and id creature
        List<(float, float[], int)> scored = new List<(float, float[], int)>();
        for (int i = 0; i < population.Count; i++)
        {
            scored.Add((population[i].distanceTraveled, genomes[i], i));

            //Debug.Log($"Distance traveled by creature {i}: {population[i].distanceTraveled}");
        }

        scored.Sort((a, b) => b.Item1.CompareTo(a.Item1)); // sort by distance
        Debug.Log($"Generation {generation} - Highest distance: {scored[0].Item1} - Second: {scored[1].Item1}");

        // color in gold the fastest creature
        ColorFastestCreature(population[scored[0].Item3]);

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

        // GENERATES THE GENOMES FOR THE NEW POPULATION
        while (genomes.Count < populationSize)
        {

            /*
            //Picks a random genome from the top 5 scored creatures for the first parent
            var parent1 = scored[Random.Range(0, 5)].Item2;

            //Picks a random genome from the top 5 scored creatures for the second parent
            var parent2 = scored[Random.Range(0, 5)].Item2;
            */

            // select the best performing two as parents of the next generation
            var parent1 = scored[0].Item2;
            var parent2 = scored[1].Item2;
            float percentage = 0;
            //Console.WriteLine("Generation: " + generation + " - Winner: " + scored[0].Item1 + " - Score: " + scored[0].Item2 + " - Genome: " + scored[0].Item3[0] + " | " + scored[0].Item3[1] + " | " + scored[0].Item3[2]);
            
            //Debug.Log($"Generation {generation} - Highest distance: {scored[0].Item1} - Second: {scored[1].Item1}");
            Debug.Log($"Generation: {generation} - Winner: {scored[0].Item3} - Distance: {scored[0].Item1} - Genome: {scored[0].Item2[0]} | {scored[0].Item2[1]} | {scored[0].Item2[2]}");

            //Initializes an empty genome for the child (with 3 genes: force, speed and phase)
            float[] child = new float[3];
            for (int g = 0; g < 3; g++)
            {
                // CROSSOVER
                // each gene is ramdomly picked from either parent 1 or parent 2
                child[g] = Random.value > 0.5f ? parent1[g] : parent2[g];   // crossover

                // MUTATION
                // 1 in 5 is also sligthly modified (adding a value ranging from -10 to +10)

                if (Random.value < 0.2f)
                    percentage = Random.Range(-10f, 10f) / 100; // percentage between -10% and +10%
                    child[g] += percentage * child[g];                // mutation

                    if (g==0 && child[g]>300) {child[g]=300;}
                    if (g==1 && child[g]<50) {child[g]=50;}

                    if (g==1 && child[g]>180) {child[g]=180;}
                    if (g==1 && child[g]<30) {child[g]=30;}
                    
                    if (g==2 && child[g]>Mathf.PI * 2f) {child[g]=Mathf.PI * 2f;}
                    if (g==2 && child[g]<0) {child[g]=0;}

            }

            genomes.Add(child);
        }


        // CREATES THE NEW POPULATION
        for (int i = 0; i < populationSize; i++)
        {
            var creature = new WalkerCreature(new Vector3(i * 3f, 2, 0), genomes[i]);
            population.Add(creature);
        }

        generation++;
        timer = 0f;
    }


    void ColorFastestCreature(WalkerCreature creature)
    {
        Color gold = new Color(1f, 0.84f, 0f); // gold color

        creature.root.GetComponent<Renderer>().material.color = gold;
        creature.upper.GetComponent<Renderer>().material.color = gold;
        creature.lower.GetComponent<Renderer>().material.color = gold;
    }

/*
    IEnumerator SetTimeSpeed(float timespeed)
    {
        Time.timeScale = timespeed;
    }


    IEnumerator FreezeTime(float duration)
    {
        yield return new WaitForSecondsRealtime(duration); // ⏱️ wait using real-world time
    }
*/

void IgnoreCollisionsBetweenCreatures(WalkerCreature a, WalkerCreature b)
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
