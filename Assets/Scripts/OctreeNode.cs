using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;

namespace BarnesHut
{
    public class OctreeNode
    { 
        public float octantSize;
        public double3 center;
        public int index;

        public OctreeNode[] children;
        // private OctreeNode root;
        private bool _hasChildren;

        public OctreeNode(double3 center, float octantSize)
        {
            this.center = center;
            this.octantSize = octantSize;
            //this.root = root;
            this.index = -1;
            this._hasChildren = false;
            this.children = new OctreeNode[8];
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
                children[octant] = new OctreeNode (newCenter, octantSize/2);
            }
            _hasChildren = true;
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
            return _hasChildren;
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

        public NativeArray<OctreeStruct> ToNativeArray(int numResults)
        {
            int4x2 childrenInt4x2;
            OctreeNode current;
            var result = new NativeArray<OctreeStruct>(numResults, Allocator.TempJob);
            var s = new Stack<OctreeNode>();
            s.Push(this);

            while (s.Count > 0)
            {
                current = s.Pop();
                
                if (hasChildren())
                {
                    foreach (var child in current.children)
                        if (child != null)
                            s.Push(child);
                    childrenInt4x2 = new int4x2(
                        (current.children[0] != null ? current.children[0].index : -1),
                        (current.children[1] != null ? current.children[1].index : -1),
                        (current.children[2] != null ? current.children[2].index : -1),
                        (current.children[3] != null ? current.children[3].index : -1),
                        (current.children[4] != null ? current.children[4].index : -1),
                        (current.children[5] != null ? current.children[5].index : -1),
                        (current.children[6] != null ? current.children[6].index : -1),
                        (current.children[7] != null ? current.children[7].index : -1)
                    );
                }
                else
                {
                    childrenInt4x2 = -1;
                }
                // Position in result must match position in nativeObjects
                result[current.index] = (new OctreeStruct(current.center, current.octantSize, current.index, childrenInt4x2));
            }
            return result;
        }
    }
}
