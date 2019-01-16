using System;

namespace RocketOptimize.Simulation
{
    public struct AscentSimulationControl
    {
        public double InitialVerticalTime;
        public double KickPitchTime;
        public double KickPitchAngle;
        public double StagingAngle;

        public double Stage1Duration;
        public double Stage2Duration;

        public double Stage1InitialAcceleration;
        public double Stage1MaxAcceleration;

        public double Stage2InitialAcceleration;
        public double Stage2MaxAcceleration;

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

        public double ComputeAngle(double time)
        {
            if (time < KickPitchTime)
            {
                return Radian(LinearBoundedInterpolation(time, InitialVerticalTime, KickPitchTime, 0, KickPitchAngle));
            }
            if (time < Stage1Duration)
            {
                return Radian(LinearBoundedInterpolation(time, KickPitchTime, Stage1Duration, KickPitchAngle, StagingAngle));
            }
            return Radian(LinearBoundedInterpolation(time, Stage1Duration, Stage1Duration + Stage2Duration, StagingAngle, 90));
        }

        public double ComputeThrust(double time)
        {
            if (time < Stage1Duration)
            {
                return LinearBoundedInterpolation(time, KickPitchTime, Stage1Duration, Stage1InitialAcceleration, Stage1MaxAcceleration);
            }
            if (time < Stage1Duration + Stage2Duration)
            {
                return LinearBoundedInterpolation(time, Stage1Duration, Stage1Duration + Stage2Duration, Stage2InitialAcceleration, Stage2MaxAcceleration);
            }
            return 0.0;
        }

        private static double Clamp(double degree, double min, double max)
        {
            return Math.Max(min, Math.Min(max, degree));
        }

        double aggregatedError;
        public void TerminalGuidance(AscentSimulationGoal goal, double timestep, double altitude, double verticalSpeed, double periapsis, double apoapsis, out double pitch, out double thrust)
        {
            const double kP = 5;
            const double kI = 5;

            double error = -verticalSpeed;
            double P = error * kP;
            aggregatedError += error * timestep;
            double I = aggregatedError * kI;

            double degrees = Clamp(P + I, -5, 15);

            if (periapsis > goal.Periapsis - 1 || apoapsis > goal.Apoapsis + 1)
            {
                thrust = 0;
            }
            else
            if (periapsis > goal.Periapsis - 25)
            {
                thrust = 0.2;
            }
            else
            {

                thrust = 1;
            }

            //double degrees = -verticalSpeed;
            //Console.WriteLine("{0,2:F}/{1,2:F} @ {2,2:F}, {3,2:F} deg, P{4,2:F} I{5,2:F}", altitude, verticalSpeed, P + I, degrees, P, I);
            pitch = Radian(degrees);

        }

        public override string ToString()
        {
            return string.Format(
@"
KickPitchTime = {0},
StagingAngle = {1},
Stage1Duration = {2},
Stage2Duration = {3},
Stage1InitialAcceleration = {4},
Stage1MaxAcceleration = {5},
Stage2InitialAcceleration = {6},
Stage2MaxAcceleration = {7}
",
                KickPitchTime,
                StagingAngle,
                Stage1Duration,
                Stage2Duration,
                Stage1InitialAcceleration,
                Stage1MaxAcceleration,
                Stage2InitialAcceleration,
                Stage2MaxAcceleration
            );
        }
    }
}
