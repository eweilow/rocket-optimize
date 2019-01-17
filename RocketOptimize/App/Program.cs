#define FORCE_DEBUG
#define USE_LOOKAHEAD_FOR_SCALING
#define ROTATING_PLANET
//#define OPTIMIZE_TRAJECTORY

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

                Stage1Duration = 155,
                Stage2Duration = 340,

                Stage1InitialAcceleration = 13,
                Stage1MaxAcceleration = 40,

                Stage2InitialAcceleration = 8,
                Stage2MaxAcceleration = 30
            };

            var initialState = SimulationRenderer.CreateInitialState(5.0, Math.PI / 2);
#if ROTATING_PLANET
            initialState.Velocity.X += Constants.EarthSurfaceVelocity;
#endif

#if OPTIMIZE_TRAJECTORY
            var optimization = new AscentOptimization(goal, input, initialState, 0.5, 1);
            for(var i = 0; i < 10000; i++)
            {
                double radius = 1.0;
                double r = radius * (5 - (i % 5)) / 5.0;
                optimization.Run(r);
            }
#endif

            var simulation = new AscentSimulation(
                goal,
                input, //optimization.BestGuess,
                initialState,
#if ROTATING_PLANET
                Constants.EarthSurfaceVelocity
#else
                0.0
#endif
            );
            using (var window = new SimulationRenderer(simulation))
            {
                window.Rate = 5;
                window.MicroStepping = 4;
                window.Start(30.0);
            }
        }
    }
}
