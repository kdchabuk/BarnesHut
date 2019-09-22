using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace BarnesHut
{
    /* public class Particle
    {
        public Vector3d position;
        public Vector3d velocity;
        public double mass;

        public Particle(Vector3d pos, Vector3d vel, double mass)
        {
            position = pos;
            velocity = vel;
            this.mass = mass;
        }
        public Particle(Particle d)
        {
            this.position = d.position;
            this.velocity = d.velocity;
            this.mass = d.mass;
        }
    } */

    public class OctreeNode
    { 
        public float octantSize;
        public double3 center;
        public int index;

        public OctreeNode[] children = new OctreeNode[8];
        private OctreeNode root;
    
        public OctreeNode(double3 center, float octantSize, OctreeNode root)
        {
            this.center = center;
            this.octantSize = octantSize;
            //this.root = root;
            this.index = -1;
        }
    
        public bool bounds(double3 v)
        {
            var d = v - center ;
            return (-octantSize <= d.x && d.x < octantSize &&
                    -octantSize <= d.y && d.y < octantSize &&
                    -octantSize <= d.z && d.z < octantSize);
        }
        public bool addBelowNode (int newIndex, Particle newObj, List<Particle> objects)
        {
            if (! bounds(newObj.position)) 
                return false;
            if (hasChildren())
            {
                return addAsChild(newIndex, newObj, objects);
            }
            else if (index >= 0)
            {
                addAsChild(this.index, objects[this.index], objects);
                this.index = -1;
                return addAsChild(newIndex, newObj, objects);
            }
            else
            {
                this.index = newIndex;
                return true;
            }

        }


        public bool addAsChild (int newIndex, Particle newObj, List<Particle> objects)
        {
            int octant = computeOctant(newObj.position);
            if (children[octant] == null)
            {
                var newCenter = octantSize/2 * octantVector(octant) + center;
                children[octant] = new OctreeNode (newCenter, octantSize/2, this.root);
            }
            return children[octant].addBelowNode (newIndex, newObj, objects);
        }
        private int computeOctant(double3 v)
        {
            //this does not check bounds, just direction
            int octant = 0;
            var d = v - center;

            if (d.x >=0) octant += 1;
            if (d.y >=0) octant += 2;
            if (d.z >=0) octant += 4;
            return octant;
        }
        static private double3 octantVector(int octant)
        {
            return (2 * (new double3(octant & 1, (octant & 2)>>1, (octant & 4)>>2)) - new double3(1));
        }
        public bool hasChildren()
        {
            foreach(var child in children)
                if(child != null)
                    return true;
            return false;
        }

        public float MinOctantSize()
        {
            if (!hasChildren())
                return octantSize;
            else
            {
                float min = octantSize;
                foreach (var child in children)
                {
                    if (child != null)
                    {
                        var s = child.MinOctantSize();
                        if (s < min) min = s;
                    }
                }
                return min;
            }
        }
    }
}
