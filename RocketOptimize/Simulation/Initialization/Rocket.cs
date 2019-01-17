using System;

namespace RocketOptimize.Simulation.Initialization
{
    public struct Rocket
    {
        public Stage[] Stages;
        public Fairing[] Fairings;
        public Payload[] Payloads;

        public double ComputeStageFuelMass(int stage, double expendedMass)
        {
            for (var i = 0; i < stage; i++)
            {
                if (expendedMass > Stages[i].FuelMass)
                {
                    expendedMass -= Stages[i].FuelMass;
                }
            }

            return Math.Max(0, Math.Min(Stages[stage].FuelMass, Stages[stage].FuelMass - expendedMass));
        }

        public int ComputeActiveStage(double expendedMass, out double fuelMass)
        {
            for (var i = 0; i < Stages.Length; i++)
            {
                fuelMass = ComputeStageFuelMass(i, expendedMass);
                if (fuelMass > 0.0)
                {
                    return i;
                }
            }
            fuelMass = 0.0;
            return Stages.Length - 1;
        }

        public double MassFlow(double expendedMass, double throttle, double pressure, out double specificImpulse, out double thrust)
        {
            var activeStage = ComputeActiveStage(expendedMass, out double fuelMass);
            return Stages[activeStage].Engine.MassFlow(throttle, pressure, out specificImpulse, out thrust);
        }

        public double ComputeMass(double reachedAltitude, double expendedMass)
        {
            double currentMass = 0.0;
            foreach (var payload in Payloads)
            {
                currentMass += payload.Mass;
            }
            foreach (var fairing in Fairings)
            {
                if (fairing.DropAltitude > reachedAltitude)
                {
                    currentMass += fairing.Mass;
                }
            }

            for (var i = 0; i < Stages.Length; i++)
            {
                double fuelMass = ComputeStageFuelMass(i, expendedMass);
                if (fuelMass > 0.0)
                {
                    currentMass += fuelMass + Stages[i].DryMass;
                }
                else if (i + 1 == Stages.Length)
                {
                    currentMass += Stages[i].DryMass; // Last stage doesn't get dropped
                }
            }

            return currentMass;
        }
    }
}
