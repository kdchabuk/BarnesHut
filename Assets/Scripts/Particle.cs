using Unity.Mathematics;

namespace BarnesHut
{
    // use length(), lengthsq(), distancesq(), etc. from Unity.Mathematics
    public struct Particle
    {
        public double3 position ;//{ get {return position;} set { position = value; } }
        public double3 velocity ;//{ get {return velocity;} set { velocity = value; } }
        public double mass ;

        public Particle(double3 pos, double3 vel, double m)
        {
            position = pos;
            velocity = vel;
            mass = m;
        }
        /* public Particle(Particle d)
        {
            this.position = new double3(d.position.x, d.position.y, d.position.z);
            this.velocity = new double3(d.velocity.x, d.velocity.y, d.velocity.z);
            this.mass = d.mass;
        } */
        public Particle(Particle d)
        {
            position = d.position;
            velocity = d.velocity;
            mass = d.mass;
        }

        public static Particle operator +(Particle a, Particle b) {
            return new Particle(a.position + b.position, a.velocity + b.velocity, a.mass + b.mass);
        }

        public static Particle operator -(Particle a, Particle b) {
            return new Particle(a.position - b.position, a.velocity - b.velocity, a.mass - b.mass);
        }

        public static Particle operator *(Particle a, double b) {
            return new Particle(a.position * b, a.velocity * b, a.mass * b);
        }
    }
}