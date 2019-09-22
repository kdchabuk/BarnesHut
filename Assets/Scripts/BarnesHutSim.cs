// A demo implementation of a Barnes-Hut n-body simulation for the Unity graphics engine
// Created by: K. D. Chabuk
// Reference: Many-body tree methods in physics / Susanne Pfalzner, Paul Gibbon.

using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Jobs;
using Unity.Collections;
using Unity.Mathematics;

namespace BarnesHut
{
    public class BarnesHutSim : MonoBehaviour
    {
        public List<GameObject> gameObjects;
        public GameObject prefab;
        [SerializeField] private bool useJobsSystem = false;
        [SerializeField] private bool useMultiTimeStep = true;
        [SerializeField] private bool reverseTime = false;
        [SerializeField] private bool drawOctants = false;
        [SerializeField] private float theta = 0.1f;
        [SerializeField] private float alphaTimeStep = 0.005f;
        [SerializeField] private double fixedDeltaTime = 0.01d;
        // Gravitational constant using km, seconds, and solar mass
        // Source: https://www.wolframalpha.com/input/?i=gravitational+constant+in+km%5E3%2Fs%5E2%2F%28solar+mass%29
        // private const double g = 1.327e11f; 
        // AU, years, and solar mass
        private const double g = 39.42;
        
        public int NumObjects = 10;
        private List<Particle> objects;
        
        private List<List<int>> interactionLists;
        private OctreeNode root;
        private float minOctantSize;
        
        private double initialEnergy;
        private List<double> t2stepsizes;
        private List<double> t3stepsizes;
        private List<int> stepcounts;
        //private List<data> newObjects;

        public static double G => g;



        void Start()
        {
            Time.fixedDeltaTime = (float)fixedDeltaTime;
            GenerateExampleSystem();
            initialEnergy = TotalEnergy();
            Debug.Log("Initial energy:" + initialEnergy);

            interactionLists = new List<List<int>>(NumObjects);
            for(int i = 0; i < this.NumObjects; i++)
                interactionLists.Add(new List<int>());

            t2stepsizes = new List<double>();
            t3stepsizes = new List<double>();
            stepcounts = new List<int>();
            
            
            gameObjects = new List<GameObject>();
            for (int i = 0; i < objects.Count; i++)
            {
                var position = new Vector3((float)objects[i].position.x, (float)objects[i].position.y, (float)objects[i].position.z);
                //var position =  new float3(objects[i].position);
                var gameObject = Instantiate(prefab, position, Quaternion.identity);
                gameObject.transform.localScale = Vector3.one * (0.2f + Mathf.Sqrt((float)objects[i].mass));
                gameObjects.Add(gameObject);
            }
        }

        void Update()
        {
            for (int i = 0; i < NumObjects; i++)
                gameObjects[i].transform.position = new Vector3(
                    (float)(objects[i].position.x), (float)objects[i].position.y, (float)objects[i].position.z);
            
            if (drawOctants)
            {
                //TODO draw octant grid
            }
        }

        void FixedUpdate()
        {
            if(fixedDeltaTime == 0 || theta <= 0 || alphaTimeStep <= 0)
                return;
            
            float startTime = Time.realtimeSinceStartup;
            
            Step();
            //Debug.Log((Time.realtimeSinceStartup - startTime) * 1000f + "ms");
            Debug.Log("Total energy rel. change:" + math.round(100d * (TotalEnergy() - initialEnergy) / initialEnergy) / 100);

            
            if (!drawOctants)
                root = null; // This allows GC to clean up OctreeNodes
        }

        void GenerateExampleSystem()
        {
            //objects = new List<data>(2 * NumObjects);
            objects = new List<Particle>(32);
            // objects.Add(new Particle(new double3(1d, 0, 0), new double3(0d, 0d, 0.0d), 0.8d));
            // objects.Add(new Particle(new double3(-1d, 0, 0), new double3(0.0d, 0, -math.sqrt(G/2d)), 0.2d));
            objects.Add(new Particle(new double3(0.0d, 0, 0d), new double3(0.0d, 0, 0), 1.0d));
            objects.Add(new Particle(new double3(1e1d, 0d, 0d), new double3(0d, 0d, math.sqrt(G/10d)), 0.001d));
            objects.Add(new Particle(new double3(1.02e1d, 0d, 0d), new double3(0d, 0d, math.sqrt(G/10d) + math.sqrt(G*.001/.2d)), 0.0002d));
            objects.Add(new Particle(new double3(-1e1d, 0d, 0d), new double3(0d, 0d, -math.sqrt(G/10d)), 0.002d));
            objects.Add(new Particle(new double3(0d, 0d, 1.2e1d), new double3(1d, 0d, 0d), 0.0001d));
            
            
            // objects.Add(new Particle(new double3(0d, 0, -20d), new double3(1, 0, 0) * math.sqrt(G/40d), 0.1d));
            // objects.Add(new Particle(new double3(0d, 0, -20d), new double3(0), 0.1d));

            NumObjects = objects.Count;
            
            // Shift gameObjects so COM and total momentum is 0
            var totalmass = 0d;
            var com = new double3(0);
            var velocity = new double3(0);
            for (int i = 0; i < NumObjects; i++)
            {
                totalmass += objects[i].mass;
                com += objects[i].position * objects[i].mass;
                velocity += objects[i].velocity * objects[i].mass;
            }
            com /= totalmass;
            velocity /= totalmass;
            for (int i = 0; i < NumObjects; i++)
            {
                // var x = objects[i];
                // objects[i] = new Particle(x.position - com, x.velocity, x.mass);
                
                objects[i] -= new Particle(com, velocity, 0);
            }
        }

        private OctreeNode BuildTree()
        {
            float sideLength = 0;
            double totalmass = 0;
            double3 com = 0;
            
            foreach (var obj in objects)
            {
                totalmass += obj.mass;
                com += obj.position * obj.mass;
            }
            com /= totalmass;

            foreach (var obj in objects)
            {
                var dist = math.length(obj.position - com) * 2;
                if (dist > sideLength)
                    sideLength = (float)dist;
            }

            root = new OctreeNode (com, sideLength, null);
            for (int i = 0; i < NumObjects; i++)
            {
                root.addAsChild (i, objects[i], objects);
            }
            //minOctantSize = root.MinOctantSize();
            return root;
        }

        static private void ComputeCOM(OctreeNode node, List<Particle> objects)
        {
            if (node.hasChildren())
            {
                // Internal node: set index to reference a new object
                if (node.index < 0)
                {
                    node.index = objects.Count;
                    objects.Add(default(Particle));
                }

                var mass = 0d;
                var com = new double3(0);
                var velocity = new double3(0);
                foreach (var child in node.children)
                {
                    if (child != null)
                    {
                        ComputeCOM(child, objects);
                        //Debug.Log("child index: " + child.index);
                        //Debug.Log("node index: " + node.index);
                        if (objects[child.index].mass > 0)
                        {
                            mass += objects[child.index].mass;
                            com += objects[child.index].position * objects[child.index].mass;
                            velocity += objects[child.index].velocity * objects[child.index].mass;
                        }
                    }
                }
                if (mass > 0)
                {
                    objects[node.index] = new Particle(com / mass, velocity / mass, mass);
                }
            }
        }

        private void Step()
        {
            BuildTree();
            ComputeCOM();
            FindInteractions();
            ComputeStepcounts();

            int f = reverseTime ? -1 : 1;
            if(useMultiTimeStep)
            {
                var tiers = new List<int>();
                for (int i = 0; i < stepcounts.Count; i++)
                {
                    var x = math.ceilpow2(stepcounts[i]);
                    int tier = 0;
                    while (x >> (tier + 1) > 0)
                        tier += 1;
                    tiers.Add(tier);
                }

                
                int maxTier = tiers.Max();
                var tierDicts = new Dictionary<int, Dictionary<int, List<int>>>();
                Dictionary<int, List<int>> d;
                for (int tier = maxTier; tier >= 0; tier--)
                {
                    if (tierDicts.ContainsKey(tier))
                        d = tierDicts[tier];
                    else
                    {
                        d = new Dictionary<int, List<int>>();
                        for (int i = 0; i < tiers.Count; i++)
                            if (tiers[i] == tier)
                            {
                                d.Add(i, interactionLists[i]);
                            }
                        tierDicts.Add(tier, d);
                    }

                    int tierMaxSteps = 1 << tier;
                    
                    for (int stepcount = 0; stepcount < tierMaxSteps; stepcount++)
                    {
                        var newObjects = RK4(f * fixedDeltaTime / tierMaxSteps, d, objects);
                        // Merging across different tiers may be improved
                        // but for now, simply combine immediately.
                        foreach (var kv in newObjects)
                            objects[kv.Key] = kv.Value;
                        ComputeCOM();
                    }
                }

                objects.RemoveRange(NumObjects, objects.Count - NumObjects);
            }
            else
            {
                List<Particle> newObjects = null;    
                newObjects = RK4(f * fixedDeltaTime, interactionLists, objects);
                
                newObjects.RemoveRange(NumObjects, newObjects.Count - NumObjects);
                objects = newObjects;
            }
            if (objects.Count > NumObjects)
                objects.RemoveRange(NumObjects, objects.Count - NumObjects);
        }

        private void rk4stepnum(double stepsize, int stepnum, IEnumerable<int> indexes, List<Particle> k, List<Particle> yn, List<Particle> tempObjects)
        {
            double factor = 0;
            switch(stepnum)
            {
                case 1: factor = 0.5; break;
                case 2: factor = 0.5; break;
                case 3: factor = 1.0; break;
                case 4: break;
                default:
                    Debug.Log("Invalid stepnum");
                    break;
            }
            foreach (int i in indexes)
            {
                //k[i].position *= stepsize;
                //k[i].velocity *= stepsize;
                k[i] *= stepsize;
                if(stepnum < 4)
                {
                    //tempObjects[i].position = yn[i].position + k[i].position * factor;
                    //tempObjects[i].velocity = yn[i].velocity + k[i].velocity * factor;
                    tempObjects[i] = yn[i] + k[i] * factor;
                }
            }
        }

        private void rk4stepnum(double stepsize, int stepnum, List<Particle> k, List<Particle> yn, List<Particle> tempObjects)
        {
            var indexes = Enumerable.Range(0, k.Count);
            rk4stepnum(stepsize, stepnum, indexes, k, yn, tempObjects);
        }

        private void rk4stepnum(double stepsize, int stepnum, Dictionary<int, Particle> k, List<Particle> yn, List<Particle> tempObjects)
        {
            double factor = 0;
            switch(stepnum)
            {
                case 1: factor = 0.5; break;
                case 2: factor = 0.5; break;
                case 3: factor = 1.0; break;
                case 4: break;
                default:
                    Debug.Log("Invalid stepnum");
                    break;
            }
            var keys = new List<int>(k.Keys);
            foreach (int i in keys)
            {
                //k[i].position *= stepsize;
                //k[i].velocity *= stepsize;
                k[i] *= stepsize;
                if(stepnum < 4)
                {
                    //tempObjects[i].position = yn[i].position + k[i].position * factor;
                    //tempObjects[i].velocity = yn[i].velocity + k[i].velocity * factor;
                    tempObjects[i] = yn[i] + k[i] * factor;
                }
            }
        }

        private Dictionary<int, Particle> RK4(double stepsize, Dictionary<int, List<int>> interactionLists, List<Particle> yn)
        {
            // fourth order Runge-Kutta numerical method
            // Reference: Computational physics / Nicholas J Giordano, Hisao Nakanishi - 2nd ed
            
            var tempObjects = new List<Particle>(yn.Count);
            for (int i = 0; i < yn.Count; i++)
                tempObjects.Add(new Particle(yn[i]));

            //k1 = h * f(0, yn);
            var k1 = computeChangeAtIndexes(interactionLists, 0, tempObjects);
            rk4stepnum(stepsize, 1, k1, yn, tempObjects);
            ComputeCOM(root, tempObjects);

            //k2 = h * f(h/2, yn + k1 / 2);
            var k2 = computeChangeAtIndexes(interactionLists, stepsize * 0.5, tempObjects);
            rk4stepnum(stepsize, 2, k2, yn, tempObjects);
            ComputeCOM(root, tempObjects);

            //k3 = h * f(h/2, yn + k2 / 2);
            var k3 = computeChangeAtIndexes(interactionLists, stepsize * 0.5, tempObjects);
            rk4stepnum(stepsize, 3, k3, yn, tempObjects);
            ComputeCOM(root, tempObjects);
            
            //k4 = h * f(h, yn + k3);
            var k4 = computeChangeAtIndexes(interactionLists, stepsize, tempObjects);
            rk4stepnum(stepsize, 4, k4, yn, tempObjects);
            
            var results = new Dictionary<int, Particle>();
            foreach (int i in interactionLists.Keys)
            {
                //yn + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
                results.Add(i, new Particle(
                    yn[i].position + (k1[i].position + 2f * k2[i].position + 2f * k3[i].position + k4[i].position) / 6f,
                    yn[i].velocity + (k1[i].velocity + 2f * k2[i].velocity + 2f * k3[i].velocity + k4[i].velocity) / 6f,
                    yn[i].mass));
            }

            return results;
        }

        private List<Particle> RK4(double stepsize, List<List<int>> interactionLists, List<Particle> yn)
        {
            // fourth order Runge-Kutta numerical method
            // Reference: Computational physics / Nicholas J Giordano, Hisao Nakanishi - 2nd ed
            
            var tempObjects = new List<Particle>(yn.Count);
            for (int i = 0; i < yn.Count; i++)
                tempObjects.Add(new Particle(yn[i]));

            //k1 = h * f(0, yn);
            var k1 = computeChangeAtAll(interactionLists, 0, tempObjects);
            rk4stepnum(stepsize, 1, k1, yn, tempObjects);
            ComputeCOM(root, tempObjects);

            //k2 = h * f(h/2, yn + k1 / 2);
            var k2 = computeChangeAtAll(interactionLists, stepsize * 0.5, tempObjects);
            rk4stepnum(stepsize, 2, k2, yn, tempObjects);
            ComputeCOM(root, tempObjects);

            //k3 = h * f(h/2, yn + k2 / 2);
            var k3 = computeChangeAtAll(interactionLists, stepsize * 0.5, tempObjects);
            rk4stepnum(stepsize, 3, k3, yn, tempObjects);
            ComputeCOM(root, tempObjects);
            
            //k4 = h * f(h, yn + k3);
            var k4 = computeChangeAtAll(interactionLists, stepsize, tempObjects);
            rk4stepnum(stepsize, 4, k4, yn, tempObjects);

            for (int i = 0; i < NumObjects; i++)
            {
                //yn + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
                // tempObjects[i].position = yn[i].position + (k1[i].position + 2f * k2[i].position + 2f * k3[i].position + k4[i].position) / 6f;
                // tempObjects[i].velocity = yn[i].velocity + (k1[i].velocity + 2f * k2[i].velocity + 2f * k3[i].velocity + k4[i].velocity) / 6f;
                tempObjects[i] = new Particle(
                    yn[i].position + (k1[i].position + 2f * k2[i].position + 2f * k3[i].position + k4[i].position) / 6f,
                    yn[i].velocity + (k1[i].velocity + 2f * k2[i].velocity + 2f * k3[i].velocity + k4[i].velocity) / 6f,
                    yn[i].mass
                );
            }

            return tempObjects;
        }

        /* private Dictionary<int, Particle> RK4ParallelJobs(double stepsize, Dictionary<int, List<int>> interactionLists, List<Particle> yn)
        {
            var tempObjects = new NativeArray<Particle>(yn.Count, Allocator.TempJob);
            for (int i = 0; i < yn.Count; i++)
                tempObjects[i] = new Particle(yn[i]);

            //k1 = h * f(0, yn);
            var k1 = computeChangeAtIndexes(interactionLists, 0, tempObjects);
            rk4stepnum(stepsize, 1, k1, yn, tempObjects);
            ComputeCOM(root, tempObjects);

            //k2 = h * f(h/2, yn + k1 / 2);
            var k2 = computeChangeAtIndexes(interactionLists, stepsize * 0.5, tempObjects);
            rk4stepnum(stepsize, 2, k2, yn, tempObjects);
            ComputeCOM(root, tempObjects);

            //k3 = h * f(h/2, yn + k2 / 2);
            var k3 = computeChangeAtIndexes(interactionLists, stepsize * 0.5, tempObjects);
            rk4stepnum(stepsize, 3, k3, yn, tempObjects);
            ComputeCOM(root, tempObjects);
            
            //k4 = h * f(h, yn + k3);
            var k4 = computeChangeAtIndexes(interactionLists, stepsize, tempObjects);
            rk4stepnum(stepsize, 4, k4, yn, tempObjects);
            
            var results = new Dictionary<int, Particle>();
            foreach (int i in interactionLists.Keys)
            {
                //yn + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
                results.Add(i, new Particle(
                    yn[i].position + (k1[i].position + 2f * k2[i].position + 2f * k3[i].position + k4[i].position) / 6f,
                    yn[i].velocity + (k1[i].velocity + 2f * k2[i].velocity + 2f * k3[i].velocity + k4[i].velocity) / 6f,
                    yn[i].mass));
            }

            return results;
        } */

        private void ComputeCOM()
        {
            ComputeCOM(root, objects);
        }

        /* private Dictionary<int, Particle> computeChangeAtIndexes(Dictionary<int, List<int>> interactionLists, double stepsize, NativeArray<Particle> objects)
        {
            var diffs = new Dictionary<int, Particle>(interactionLists.Count);
            foreach (var i in interactionLists.Keys)
            {
                var list = new NativeArray<int>();
                interactionLists[i].CopyTo(list);
                var particle = new Particle();

                //diffs.Add(i, computeChangeAtIndex(i, interactionLists[i], stepsize, objects));
                ComputeChangeJob job = new ComputeChangeJob {
                    stepsize = stepsize,
                    diff = particle,
                    interactionIndexes = list,
                    jobObjects = objects,
                };
                job.Schedule();
            }
            return diffs;
        } */

        private Dictionary<int, Particle> computeChangeAtIndexes(Dictionary<int, List<int>> interactionLists, double stepsize, List<Particle> objects)
        {
            var diffs = new Dictionary<int, Particle>(interactionLists.Count);
            foreach (var i in interactionLists.Keys)
            {
                diffs.Add(i, computeChangeAtIndex(i, interactionLists[i], stepsize, objects));
            }
            return diffs;
        }

        private List<Particle> computeChangeAtAll(List<List<int>> interactionLists, double stepsize, List<Particle> objects)
        {
            var diffs = new List<Particle>(NumObjects);
            for (int i = 0; i < this.NumObjects; i++)
            {
                diffs.Add(computeChangeAtIndex(i, interactionLists[i], stepsize, objects));
            }
            return diffs;
        }

        private Particle computeChangeAtIndex(int index, List<int> interactionIndexes, double stepsize, List<Particle> tempObjects)
        {
            double3 acc = 0;

            foreach(int gravSourceIndex in interactionIndexes)
            {
                var d = tempObjects[gravSourceIndex].position - tempObjects[index].position +
                        (tempObjects[gravSourceIndex].velocity - tempObjects[index].velocity) * stepsize;
                var dmag = math.length(d);
                
                acc += G * tempObjects[gravSourceIndex].mass / (dmag*dmag*dmag) * d;
            }
            Particle dydt = new Particle(tempObjects[index].velocity, acc, 0);
            return dydt;
        }

        private struct ComputeChangeJob : IJobParallelFor
        {
            public NativeArray<Particle> jobObjects;
            public NativeArray<int> interactionIndexes;
            public double stepsize;
            public Particle diff;

            public void Execute(int index)
            {
                double3 acc = 0;

                foreach(int gravSourceIndex in interactionIndexes)
                {
                    var d = jobObjects[gravSourceIndex].position - jobObjects[index].position +
                            (jobObjects[gravSourceIndex].velocity - jobObjects[index].velocity) * stepsize;
                    var dmag = math.length(d);
                    
                    acc += G * jobObjects[gravSourceIndex].mass / (dmag*dmag*dmag) * d;
                }
                diff = new Particle(jobObjects[index].velocity, acc, 0);
            }
        }

        private void FindInteractions()
        {
            foreach (var l in interactionLists)
                l.Clear();
            for(int i = 0; i < this.NumObjects; i++)
            {
                var l = interactionLists[i];
                FindInteractions(i, root, ref l);
                //interactionLists.Add(l);
            }
        }

        private void FindInteractions(int index, OctreeNode node, ref List<int> list)
        {
            double dmag, s = 1;
            if (index == node.index)
                dmag = 0;
            else
            {
                s = node.octantSize * 2;
                var d = objects[index].position - objects[node.index].position;
                dmag = math.length(d);
            }
            
            // dmag > 0 is necessary here
            if(dmag > 0 && (theta * dmag > s || !node.hasChildren()))
                list.Add(node.index);
            else
            {
                foreach (var child in node.children)
                {
                    if (child != null)
                        FindInteractions(index, child, ref list);
                }
            }
        }

        private int NearestNbhrIndex(int index, List<int> interactionList, List<Particle> objects)
        {
            double minDist = double.PositiveInfinity;
            int minIndex = -1;
            foreach(int i in interactionList)
            {
                var dmag = math.lengthsq(objects[index].position - objects[i].position);
                if (dmag < minDist)
                {
                    minDist = dmag;
                    minIndex = i;
                }
            }
            return minIndex;
        }

        private int FastestNbhrIndex(int index, List<int> interactionList, List<Particle> objects)
        {
            double minSpeed = double.PositiveInfinity;
            int minIndex = -1;
            foreach(int i in interactionList)
            {
                var speed = math.lengthsq(objects[index].velocity - objects[i].velocity);
                if (speed < minSpeed)
                {
                    minSpeed = speed;
                    minIndex = i;
                }
            }
            return minIndex;
        }

        private double TotalEnergy()
        {
            double ke = 0, pe = 0;
            for(int i = 0; i < NumObjects; i++)
            {
                ke += 0.5 * objects[i].mass * math.lengthsq(objects[i].velocity);
                for(int j = i + 1; j < NumObjects; j++)
                    pe += -G * objects[i].mass * objects[j].mass / math.length(objects[i].position - objects[j].position);
            }
            return ke + pe;    
        }

        private void ComputeStepcounts()
        {
            t2stepsizes.Clear();
            t3stepsizes.Clear();
            stepcounts.Clear();
            for (int i = 0; i < interactionLists.Count; i++)
            {
                var l = interactionLists[i];
                var nearestNbhrIndex = NearestNbhrIndex(i, l, objects);
                var d = objects[nearestNbhrIndex].position - objects[i].position;
                var fastestNbhrIndex = FastestNbhrIndex(i, l, objects);
                var v = objects[fastestNbhrIndex].velocity - objects[i].velocity;

                // From Pfalzner Gibbon, p. 66, equation 4.2 and 4.3
                var t2time = alphaTimeStep * 2 / (G * objects[nearestNbhrIndex].mass) * math.pow(math.lengthsq(d), 0.75d);
                t2stepsizes.Add(t2time);
                var t3time = alphaTimeStep * math.length(d) / math.length(v);
                t3stepsizes.Add(t3time);

                var time = math.min(t2time, t3time);
                if (fixedDeltaTime / time > 0)
                    stepcounts.Add((int)math.ceil(fixedDeltaTime / time));
                else
                    stepcounts.Add(1);
            }
        }
        //To calculate the net force on a particular body, the nodes of the tree are traversed, starting from the root. If the center of mass of an internal node is sufficiently far from the body, the bodies contained in that part of the tree are treated as a single particle whose position and mass is respectively the center of mass and total mass of the internal node. If the internal node is sufficiently close to the body, the process is repeated for each of its children. 
        //Whether a node is or isn't sufficiently far away from a body, depends on the quotient s / d {\displaystyle s/d} s/d, where s is the width of the region represented by the internal node, and d is the distance between the body and the node's center of mass. The node is sufficiently far away when this ratio is smaller than a threshold value θ. The parameter θ determines the accuracy of the simulation; larger values of θ increase the speed of the simulation but decreases its accuracy. If θ = 0, no internal node is treated as a single body and the algorithm degenerates to a direct-sum algorithm.

        // dx/dt = v(t)
        // dv/dt = -G m2 / d**2 * dhat
        // first apply RK4 to v and dv/dt to get new velocity, v_n+1.

    }
}
