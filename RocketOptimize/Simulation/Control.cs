using System;

namespace RocketOptimize.Simulation
{
    public struct Input
    {
        readonly float AngleFromVertical;
        readonly float ThrustFactor;

        public Input(float angleFromVertical, float thrustFactor)
        {
            AngleFromVertical = angleFromVertical;
            ThrustFactor = thrustFactor;
        }
    }

}
