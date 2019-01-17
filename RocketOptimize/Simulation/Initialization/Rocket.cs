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
                expendedMass -= Stages[i].FuelMass;
            }

            return Math.Max(0, Math.Min(Stages[stage].FuelMass, Stages[stage].FuelMass - expendedMass));
        }

        public int ComputeActiveStage(double expendedMass, out double fuelMass)
        {
            for (var i = 0; i < Stages.Length; i++)
            {
                fuelMass = ComputeStageFuelMass(i, expendedMass);
                if (fuelMass > 5.0)
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
            if (fuelMass > 0)
            {
                return Stages[activeStage].Engine.MassFlow(throttle, pressure, out specificImpulse, out thrust);
            }
            return Stages[activeStage].Engine.MassFlow(0, pressure, out specificImpulse, out thrust);
        }

        public double TotalFuelMass()
        {
            double mass = 0.0;
            foreach(var stage in Stages)
            {
                mass += stage.FuelMass;
            }
            return mass;
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

            var activeStage = ComputeActiveStage(expendedMass, out double fuelMass);
            currentMass += Stages[activeStage].DryMass;
            currentMass += fuelMass;
            for (var i = activeStage + 1; i < Stages.Length; i++)
            {
                currentMass += Stages[i].FuelMass + Stages[i].DryMass;
            }

            return currentMass;
        }
    }
}
