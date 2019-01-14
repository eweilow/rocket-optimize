using OpenTK;

namespace RocketOptimize.Simulation
{
    public struct State
    {
        public float Time;
        public Vector3d Position;
        public Vector3d Velocity;
        public Vector3d Acceleration;
        public Vector3d Gravity;
        public Vector3d Drag;
        public Vector3d Thrust;
        public Atmosphere Atmosphere;
    }
}
