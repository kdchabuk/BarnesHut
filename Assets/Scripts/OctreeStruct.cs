using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace BarnesHut
{
    public struct OctreeStruct
    { 
        public float octantSize;
        public double3 center;
        public int index;
        public int4x2 children;
        public bool hasChildren;

        public OctreeStruct(double3 center, float octantSize, int index, int4x2 children)
        {
            this.center = center;
            this.octantSize = octantSize;
            this.index = index;
            var bc = (children >= 0);
            this.hasChildren = math.any(bc.c0) || math.any(bc.c1);
            this.children = children;
        }
    }
}