#define FORCE_DEBUG
#define USE_LOOKAHEAD_FOR_SCALING
#define ROTATING_PLANET
#define OPTIMIZE_TRAJECTORY

using RocketOptimize.Simulation;
using System;

namespace RocketOptimize.App
{
    class Program
    {
        static void Main(string[] args)
        {
            var goal = new AscentSimulationGoal()
            {
                MaxAcceleration = 50,
                Apoapsis = 180,
                Periapsis = 180
            };

            var input = new AscentSimulationControl()
            {
                InitialVerticalTime = 10,
                KickPitchTime = 55,
                KickPitchAngle = 4,
                StagingAngle = 68.7,

                Stage1Duration = 160,
                Stage2Duration = 400,

                Stage1InitialAcceleration = 13,
                Stage1MaxAcceleration = 40,

                Stage2InitialAcceleration = 15,
                Stage2MaxAcceleration = 30
            };

            var initialState = SimulationRenderer.CreateInitialState(5.0, Math.PI / 2);
#if ROTATING_PLANET
            initialState.Velocity.X += Constants.EarthSurfaceVelocity;
#endif

#if OPTIMIZE_TRAJECTORY
            var timeStepsAndSettings = new (double, double, double, int)[]
            {
                (1, 1.0, 1.0, 10000),
                (0.1, 0.5, 0.5, 10000),
                //(0.05, 0.1, 0.1, 10000),
            };

            var bestGuess = input;

            foreach(var tuple in timeStepsAndSettings)
            {
                Console.WriteLine(tuple);
                var optimization = new AscentOptimization(goal, bestGuess, initialState, tuple.Item2, 1);
                for(var i = 0; i < tuple.Item4; i++)
                {
                    double radius = tuple.Item3;
                    double r = radius * (5 - (i % 5)) / 5.0;
                    if (optimization.Run(r) < tuple.Item1)
                    {
                        break;
                    }
                }
                bestGuess = optimization.BestGuess;
            }
#endif

            var simulation = new AscentSimulation(
                goal,
                bestGuess,
                initialState,
#if ROTATING_PLANET
                Constants.EarthSurfaceVelocity
#else
                0.0
#endif
            );
            using (var window = new SimulationRenderer(simulation))
            {
                window.Rate = 15;
                window.MicroStepping = 10;
                window.Start(60.0);
            }
        }
    }
}
