using System;

namespace RocketOptimize.Simulation
{
    public class AscentOptimization
    {
        private static double Gaussian(double x, double scale = 1)
        {
            return (x * x / (scale * scale));
        }

        public readonly double TimeStep;
        public readonly int MicroSteps;
        public readonly AscentSimulationGoal Goal;
        public readonly AscentSimulationControl InitialGuess;
        public AscentSimulationControl BestGuess;
        public AscentSimulationControl CurrentGuess;
        public readonly State InitialState;
        private AscentSimulation _simulation;
        private int _iterationsRun;

        public double CurrentScore
        {
            get
            {
                const double distanceScaling = 10.0;
                double distanceFromApoapsisGoal = _simulation.LookAheadState.Apoapsis / 1000.0 - Goal.Apoapsis;
                double distanceFromPeriapsisGoal = _simulation.LookAheadState.Periapsis / 1000.0 - Goal.Periapsis;

                return Gaussian(distanceFromApoapsisGoal, distanceScaling)
                     + Gaussian(distanceFromPeriapsisGoal, distanceScaling)
                     + Gaussian(_simulation.LookAheadState.Eccentricity, 0.5);
                     //+ Gaussian(_simulation.CurrentState.LossesToGravity, 1500.0);
            }
        }

        public AscentOptimization(AscentSimulationGoal goal, AscentSimulationControl initialGuess, State initialState, double timeStep, int microSteps)
        {
            Goal = goal;
            InitialGuess = initialGuess;
            BestGuess = initialGuess;
            InitialState = initialState;
            TimeStep = timeStep;
            MicroSteps = microSteps;

            Reset(initialGuess);
            //Print();
        }

        private void Reset(AscentSimulationControl currentGuess)
        {
            CurrentGuess = currentGuess;
            _simulation = new AscentSimulation(
                CurrentGuess,
                InitialState
            );
            _iterationsRun = 0;
        }

        private void Print()
        {
            Console.WriteLine("{0,1:F} {1,1:F}: g {2,1:F}, d {3,1:F} - score {4,3:F}/{5,3:F}", 
                (_simulation.LookAheadState.Periapsis) / 1000.0, 
                (_simulation.LookAheadState.Apoapsis) / 1000.0,
                _simulation.CurrentState.LossesToGravity,
                _simulation.CurrentState.LossesToDrag,
                CurrentGuess.Score,
                BestGuess.Score
            );
        }

        public void Run(bool forceScore = false)
        {
            while(_simulation.CurrentState.Time < CurrentGuess.ThrustDuration)
            {
                _simulation.FastTick(TimeStep, MicroSteps);
                _iterationsRun += MicroSteps;
                if(_simulation.LookAheadState.Apoapsis / 1000.0 > Goal.Apoapsis)
                {
                    CurrentGuess.ThrustDuration = _simulation.CurrentState.Time;
                    break;
                }
                if (_simulation.LookAheadState.Periapsis / 1000.0 > Goal.Periapsis)
                {
                    CurrentGuess.ThrustDuration = _simulation.CurrentState.Time;
                    break;
                }
            }
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
        }
    }

    public static class RandomExtensions
    {
        public static double NextDouble(
            this Random random,
            double minValue,
            double maxValue)
        {
            return random.NextDouble() * (maxValue - minValue) + minValue;
        }
    }
}
