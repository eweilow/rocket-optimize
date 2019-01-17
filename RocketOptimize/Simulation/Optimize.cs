using System;
using System.Threading.Tasks;

namespace RocketOptimize.Simulation
{
    public class AscentOptimization
    {
        private static double ErrorFunction(double x, double scale = 1)
        {
            return (x * x / (scale * scale));
        }

        public static double ComputeScore(AscentSimulationGoal goal, LookAheadState lookAheadState, AscentSimulationControl guess)
        {
            const double distanceScaling = 10.0;
            double distanceFromApoapsisGoal = lookAheadState.Apoapsis / 1000.0 - goal.Apoapsis;
            double distanceFromPeriapsisGoal = lookAheadState.Periapsis / 1000.0 - goal.Periapsis;

            return ErrorFunction(distanceFromApoapsisGoal, distanceScaling)
                    + ErrorFunction(distanceFromPeriapsisGoal, distanceScaling)
                    + ErrorFunction(lookAheadState.Eccentricity, 0.5);
                    //+ ErrorFunction((guess.ThrustDuration1 + guess.ThrustDuration2) - guess.ThrustCutoff, 1);
        }

        private static void Print(AscentSimulationGoal goal, LookAheadState lookAheadState, State state, AscentSimulationControl guess, double bestScore)
        {
            Console.WriteLine("{0,1:F} {1,1:F}: g {2,1:F}, d {3,1:F} - score {4,3:F}/{5,3:F}",
                (lookAheadState.Periapsis) / 1000.0,
                (lookAheadState.Apoapsis) / 1000.0,
                state.LossesToGravity,
                state.LossesToDrag,
                ComputeScore(goal, lookAheadState, guess),
                bestScore
            );
        }

        public readonly double TimeStep;
        public readonly int MicroSteps;
        public readonly AscentSimulationGoal Goal;
        public readonly AscentSimulationControl InitialGuess;
        public AscentSimulationControl BestGuess;
        public double BestScore;
        public readonly State InitialState;

        public AscentOptimization(AscentSimulationGoal goal, AscentSimulationControl initialGuess, State initialState, double timeStep, int microSteps)
        {
            Goal = goal;
            InitialGuess = initialGuess;
            BestGuess = initialGuess;
            InitialState = initialState;
            TimeStep = timeStep;
            MicroSteps = microSteps;

            LookAheadState lookAheadState;
            State state;
            BestScore = Tasklet(BestGuess, 0, 0.0, false, out BestGuess, out lookAheadState, out state);
        }

        private Random _random = new Random();
        private double Tasklet(AscentSimulationControl currentBestGuess, double currentBestScore, double perturbationScale, bool print, 
            out AscentSimulationControl control, 
            out LookAheadState lookAheadState,
            out State state)
        {
            control = new AscentSimulationControl()
            {
                InitialVerticalTime = currentBestGuess.InitialVerticalTime + (perturbationScale * _random.NextDouble(2)),
                KickPitchTime = currentBestGuess.KickPitchTime + (perturbationScale * _random.NextDouble(4)),
                KickPitchAngle = Math.Max(0, currentBestGuess.KickPitchAngle + (perturbationScale * _random.NextDouble(1))),
                StagingAngle = Math.Max(0, Math.Min(90, currentBestGuess.StagingAngle + (perturbationScale * _random.NextDouble(2)))),
                Stage1Duration = currentBestGuess.Stage1Duration, // + (perturbationScale * _random.NextDouble(3)),
                Stage2Duration = currentBestGuess.Stage2Duration, // + (perturbationScale * _random.NextDouble(3)),
                Stage1InitialAcceleration = currentBestGuess.Stage1InitialAcceleration,
                Stage1MaxAcceleration = currentBestGuess.Stage1MaxAcceleration,
                Stage2InitialAcceleration = currentBestGuess.Stage2InitialAcceleration,
                Stage2MaxAcceleration = currentBestGuess.Stage2MaxAcceleration
                //InitialTurnDuration = currentBestGuess.InitialTurnDuration + (perturbationScale *  _random.NextDouble(0.2)),
                //MaxThrust1 = currentBestGuess.MaxThrust1,
                //MinThrust1 = currentBestGuess.MinThrust1,
                //MaxThrust2 = currentBestGuess.MaxThrust2,
                //MinThrust2 = currentBestGuess.MinThrust2,
                //ThrustCutoff = 1000, // Math.Max(0, currentBestGuess.ThrustCutoff + (perturbationScale * _random.NextDouble(5))),
                ////ThrustCurve = Math.Max(0, currentBestGuess.ThrustCurve + (perturbationScale * _random.NextDouble(5))),
                //ThrustDuration1 = Math.Max(0, currentBestGuess.ThrustDuration1 + (perturbationScale * _random.NextDouble(5))),
                //ThrustDuration2 = Math.Max(0, currentBestGuess.ThrustDuration2 + (perturbationScale * _random.NextDouble(5))),
                //TurnDelay = currentBestGuess.TurnDelay, //Math.Max(0, currentBestGuess.TurnDelay + (perturbationScale * _random.NextDouble(1))),
                //TurnDuration = Math.Max(0, currentBestGuess.TurnDuration + (perturbationScale * _random.NextDouble(1))),
            };

            var simulation = new AscentSimulation(Goal, control, InitialState);
            while (simulation.CurrentState.Time < control.Stage1Duration + control.Stage2Duration)
            {
                simulation.FastTick(TimeStep, MicroSteps);
                if (simulation.LookAheadState.Apoapsis / 1000.0 > (Goal.Apoapsis + 15) || simulation.LookAheadState.Periapsis / 1000.0 > (Goal.Periapsis + 15))
                {
                    control.Stage2Duration = Math.Max(0, simulation.CurrentState.Time - control.Stage1Duration);
                    break;
                }
            }
            if(print)
            {
                Print(Goal, simulation.LookAheadState, simulation.CurrentState, control, currentBestScore);
            }
            lookAheadState = simulation.LookAheadState;
            state = simulation.CurrentState;
            return ComputeScore(Goal, simulation.LookAheadState, control);
        }

        public double Run(double perturbationScale = 1.0)
        {
            var tasks = new Task<(double, AscentSimulationControl, LookAheadState, State)>[8];
            for (int i = 0; i < tasks.Length; i++)
            {
                tasks[i] = new Task<(double, AscentSimulationControl, LookAheadState, State)>(() =>
                {
                    AscentSimulationControl newControl;
                    LookAheadState lookAheadState;
                    State state;
                    double newScore = Tasklet(BestGuess, BestScore, perturbationScale, false, out newControl, out lookAheadState, out state);
                    return (newScore, newControl, lookAheadState, state);
                });
                tasks[i].Start();
            }
            for (int i = 0; i < tasks.Length; i++)
            {
                tasks[i].Wait();
                if (tasks[i].Result.Item1 < BestScore)
                {
                    BestScore = tasks[i].Result.Item1;
                    BestGuess = tasks[i].Result.Item2;
                    Print(Goal, tasks[i].Result.Item3, tasks[i].Result.Item4, BestGuess, BestScore);
                    Console.WriteLine(BestGuess);

                }
            }

            return BestScore;
            /*
            CurrentGuess.Score = CurrentScore;
            Print();
            if (forceScore || CurrentGuess.Score < BestGuess.Score)
            {
                BestGuess = CurrentGuess;
                Console.WriteLine("New best!");
            }

            Random random = new Random();
            Reset(new AscentSimulationControl()
            {
                InitialTurnDuration = BestGuess.InitialTurnDuration + random.NextDouble(-0.1, 0.1),
                MaxThrust = BestGuess.MaxThrust,
                MinThrust = BestGuess.MinThrust,
                ThrustCurve = Math.Max(0, BestGuess.ThrustCurve + random.NextDouble(-0.1, 0.1)),
                ThrustDuration = Math.Max(0, BestGuess.ThrustDuration + random.NextDouble(-5, 5)),
                TurnDelay = Math.Max(0, BestGuess.TurnDelay + random.NextDouble(-1, 1)),
                TurnDuration = Math.Max(0, BestGuess.TurnDuration + random.NextDouble(-0.1, 0.1)),
            });
            */
        }
    }

    public static class RandomExtensions
    {
        public static double NextDouble(
            this Random random,
            double radius)
        {
            return random.NextDouble(-radius, radius);
        }

        public static double NextDouble(
            this Random random,
            double minValue,
            double maxValue)
        {
            return random.NextDouble() * (maxValue - minValue) + minValue;
        }
    }
}
