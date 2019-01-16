using System;

namespace RocketOptimize.Simulation
{
    public struct AscentSimulationControl
    {
        public double InitialTurnDuration;
        public double TurnDelay;
        public double TurnDuration;
        public double ThrustDuration;
        public double ThrustCurve;
        public double MinThrust;
        public double MaxThrust;

        public double ComputeAngle(double time)
        {
            double initialTurnAngle = 10 * Math.Min(1.0, time / InitialTurnDuration);
            double finalTurnAngle = 80.0 * Math.Min(1.0, Math.Max(0.0, time - TurnDelay) / TurnDuration);
            double degree = initialTurnAngle + finalTurnAngle;
            return degree / 180.0 * Math.PI;
        }

        public double ComputeThrust(double time)
        {
            if (time < ThrustDuration)
            {
                double rampThrust = (MaxThrust - MinThrust);
                double ramp = rampThrust * Math.Min(1.0, time / ThrustCurve);
                return MinThrust + ramp;
            }
            return 0.0;
        }
    }
}
