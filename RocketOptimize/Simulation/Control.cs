using RocketOptimize.Simulation.Initialization;
using System;

namespace RocketOptimize.Simulation
{


    public struct AscentSimulationControl
    {
        public double InitialVerticalTime;
        public double KickPitchTime;
        public double KickPitchAngle;
        public double StagingAngle;
        public double MaxAcceleration;
        public Rocket Rocket;

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

        private static double Radian(double degree)
        {
            return degree / 180.0 * Math.PI;
        }

        public double ComputeMass(double reachedAltitude, double expendedMass)
        {
            return Rocket.ComputeMass(reachedAltitude, expendedMass);
        }

        public double ComputeAngle(double time, double expendedMass)
        {
            if (time < KickPitchTime)
            {
                return Radian(LinearBoundedInterpolation(time, InitialVerticalTime, KickPitchTime, 0, KickPitchAngle));
            }
            var activeStage = Rocket.ComputeActiveStage(expendedMass, out double fuelMass);
            //Console.WriteLine("{0} {1}", activeStage, fuelMass);
            if (activeStage == 0)
            {
                double FuelMassFlowFromKickPitch = Rocket.MassFlow(expendedMass, 1.0, Constants.EarthSurfacePressure, out double specificImpulse, out double thrust);
                return Radian(LinearBoundedInterpolation(fuelMass, Rocket.Stages[activeStage].FuelMass - FuelMassFlowFromKickPitch * KickPitchTime, 0, KickPitchAngle, StagingAngle));
            }
            else if (activeStage == 1)
            {
                return Radian(LinearBoundedInterpolation(fuelMass, Rocket.Stages[activeStage].FuelMass, 0, StagingAngle, 90));
            }
            return 90;
        }

        public double ComputeThrust(double time, double throttle, double expendedMass, double pressure, out double massFlow)
        {
            double thrust;
            double specificImpulse;
            massFlow = Rocket.MassFlow(expendedMass, throttle, pressure, out specificImpulse, out thrust);
            return thrust;
        }

        private static double Clamp(double degree, double min, double max)
        {
            return Math.Max(min, Math.Min(max, degree));
        }

        double aggregatedError;
        public void TerminalGuidance(AscentSimulationGoal goal, double timestep, double altitude, double verticalSpeed, double periapsis, double apoapsis, out double pitch, out double thrust, out bool isDone)
        {
            double maxAngle = 15;
            double minAngle = -5;
            double offset = 5;
            isDone = false;

            if (periapsis > 0)
            {
                maxAngle = 30;
                minAngle = -30;
            }
            if (periapsis > -500)
            {
                offset = 0;
            }

            const double kP = 5;
            const double kI = 5;

            double error = -verticalSpeed;
            double P = error * kP;
            aggregatedError += error * timestep;
            double I = aggregatedError * kI;

            double degrees = Clamp(P + I + offset, minAngle, maxAngle);

            var periapsisIsGood = periapsis > goal.Periapsis - 2;
            var apoapsisIsGood = apoapsis > goal.Apoapsis - 2 && apoapsis < goal.Apoapsis + 3;
            var verticalVelocityIsGood = Math.Abs(verticalSpeed) < 15;
            //Console.WriteLine("{0,2:F} km x {1,2:F} km, v: {2,2:F} m/s", periapsis, apoapsis, verticalSpeed);
            if (periapsisIsGood && apoapsisIsGood)
            {
                //Console.WriteLine("CANCELLING THRUST: periapsisIsGood && apoapsisIsGood");
                thrust = 0;
                isDone = true;
            }
            else if(periapsisIsGood && verticalVelocityIsGood)
            {
                //Console.WriteLine("CANCELLING THRUST: periapsisIsGood && verticalVelocityIsGood");
                thrust = 0;
                isDone = true;
            }
            else
            if (periapsis > goal.Periapsis - 10)
            {
                thrust = 0.05;
            }
            else
            if (periapsis > 0)
            {
                thrust = 0.25;
            }
            else
            if (periapsis > -100)
            {
                thrust = 0.5;
            }
            else
            {

                thrust = 1;
            }
            //Console.WriteLine("{0,2:F}/{1,2:F} {2,2:F}/{3,2:F} = {4,2:F}, {5,2:F}", periapsis, goal.Periapsis, apoapsis, goal.Apoapsis, thrust, degrees);

            //double degrees = -verticalSpeed;
            //Console.WriteLine("{0,2:F}/{1,2:F} @ {2,2:F}, {3,2:F} deg, P{4,2:F} I{5,2:F}", altitude, verticalSpeed, P + I + 5, degrees, P, I);
            pitch = Radian(degrees);

        }

        public override string ToString()
        {
            return string.Format(
@"
KickPitchTime = {0},
StagingAngle = {1},
KickPitchAngle = {2},
InitialVerticalTime = {3},
MaxAcceleration = {4}
",
                KickPitchTime,
                StagingAngle,
                KickPitchAngle,
                InitialVerticalTime,
                MaxAcceleration
            );
        }
    }
}
