using System;

namespace RocketOptimize.Simulation.Initialization
{
    public struct Engine
    {
        public int Count;
        public double SeaLevelThrust;
        public double VacuumThrust;

        public double SeaLevelSpecificImpulse;
        public double VacuumSpecificImpulse;

        private static double LinearBoundedInterpolation(
            double time,
            double startTime,
            double endTime,
            double initialValue,
            double finalValue
        )
        {
            double factor = Math.Max(0, Math.Min(1, (time - startTime) / (endTime - startTime)));
            return initialValue + factor * (finalValue - initialValue);
        }

        public double ComputeThrust(double throttle, double pressure)
        {
            return LinearBoundedInterpolation(throttle, 0, 1, 0, LinearBoundedInterpolation(pressure, 0, Constants.EarthSurfacePressure, VacuumThrust, SeaLevelThrust) * Count);
        }

        public double ComputeSpecificImpulse(double throttle, double pressure)
        {
            return LinearBoundedInterpolation(pressure, 0, Constants.EarthSurfacePressure, VacuumSpecificImpulse, SeaLevelSpecificImpulse);
        }

        public double MassFlow(double throttle, double pressure, out double specificImpulse, out double thrust)
        {
            thrust = ComputeThrust(throttle, pressure);
            specificImpulse = ComputeSpecificImpulse(throttle, pressure);
            return thrust / (Constants.G0 * specificImpulse);
        }
    }
}
