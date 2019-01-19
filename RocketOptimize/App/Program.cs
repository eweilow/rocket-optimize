//#define ROTATING_PLANET
#define OPTIMIZE_TRAJECTORY

using RocketOptimize.Simulation;
using RocketOptimize.Simulation.Initialization;
using System;

namespace RocketOptimize.App
{
    class Program
    {
        static void Main(string[] args)
        {
            var goal = new AscentSimulationGoal()
            {
                Apoapsis = 220,
                Periapsis = 220
            };

            var firstStageMerlins = new Engine()
            {
                Count = 9,
                SeaLevelSpecificImpulse = 282,
                VacuumSpecificImpulse = 311,
                SeaLevelThrust = 845000,
                VacuumThrust = 914000
            };

            var firstStage = new Stage()
            {
                Engine = firstStageMerlins,
                DryMass = 22200,
                FuelMass = 433100 - 22200,
            };

            var secondStageMerlin = new Engine()
            {
                Count = 1,
                SeaLevelSpecificImpulse = 282,
                VacuumSpecificImpulse = 348,
                SeaLevelThrust = 1200000, //845000,
                VacuumThrust = 1200000, //934000
            };

            var secondStage = new Stage()
            {
                Engine = secondStageMerlin,
                DryMass = 4000,
                FuelMass = 111500 - 4000,
            };

            var fairing = new Fairing()
            {
                Mass = 1700,
                DropAltitude = 100000
            };

            var payload = new Payload()
            {
                Mass = 10000
            };

            var rocket = new Rocket()
            {
                Fairings = new[] { fairing },
                Payloads = new[] { payload },
                Stages = new[] { firstStage, secondStage }
            };

            /*
            for(var f = 0; f < 600000; f+= 1000)
            {
                Console.WriteLine("{0}: {1} {2,2:F} mf: {3,2:F}, Isp {4,2:F}, F {5,2:F}, Fm {6,2:F}", 
                    f, 
                    rocket.ComputeActiveStage(f, out double fuelMass), 
                    rocket.ComputeMass(0, f), 
                    rocket.MassFlow(f, 1.0, 50000, out double a, out double b),a,b,
                    fuelMass
                );
            }
            Console.ReadLine();
            */

            var input = new AscentSimulationControl()
            {
                Rocket = rocket,
                KickPitchTime = 52.5390915729753,
                StagingAngle = 73.9264028953977,
                KickPitchAngle = 4,
                InitialVerticalTime = 5.92364164880646,
                MaxAcceleration = 54.984388588776
            };

            var initialState = SimulationRenderer.CreateInitialState(1, Math.PI / 2);
#if ROTATING_PLANET
            initialState.Velocity.X += Constants.EarthSurfaceVelocity;
#endif

#if OPTIMIZE_TRAJECTORY
            var timeStepsAndSettings = new (double, double, double, int)[]
            {
                (10, 1.0, 30.0, 10000),
                (5, 1.0, 15.0, 10000),
                (5, 0.5, 8.0, 10000),
                (2, 0.5, 4.0, 10000),
                (1, 0.2, 2.0, 10000),
                (0.2, 0.1, 1, 1000),
                //(0.1, 0.1, 0.5, 10000),
                //(0.1, 0.01, 0.5, 10000),
                //(0.1, 0.001, 0.1, 10000),
            };

            var bestGuess = input;
            double bestScore = -1;

            foreach (var tuple in timeStepsAndSettings)
            {
                Console.WriteLine(tuple);
                var optimization = new AscentOptimization(
#if ROTATING_PLANET
                Constants.EarthSurfaceVelocity,
#else
                0.0,
# endif
                goal, bestGuess, initialState, tuple.Item2, 1, bestScore);
                for (var i = 0; i < tuple.Item4; i++)
                {
                    double radius = tuple.Item3;
                    double r = radius * (5 - (i % 5)) / 5.0;
                    double score = optimization.Run(r);
                    if (score < tuple.Item1)
                    {
                        break;
                    }
                }
                bestScore = optimization.BestScore;
                bestGuess = optimization.BestGuess;
            }
#endif

            var simulation = new AscentSimulation(
                goal,
#if OPTIMIZE_TRAJECTORY
                bestGuess,
#else
                input,
#endif
                initialState,
#if ROTATING_PLANET
                Constants.EarthSurfaceVelocity
#else
                0.0
#endif
            );
            using (var window = new SimulationRenderer(simulation))
            {
                window.Rate = 50;
                window.MicroStepping = 10;
                window.Start(60.0);
            }
        }
    }
}
