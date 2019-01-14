using System;

namespace RocketOptimize.Simulation
{
    public struct Input
    {
        readonly double AngleFromVertical;
        readonly double ThrustFactor;

        public Input(double angleFromVertical, double thrustFactor)
        {
            AngleFromVertical = angleFromVertical;
            ThrustFactor = thrustFactor;
        }
    }

}
