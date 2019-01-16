#define FORCE_DEBUG
#define USE_LOOKAHEAD_FOR_SCALING

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
                Apoapsis = 170,
                Periapsis = 170
            };

            var input = new AscentSimulationControl()
            {
                TurnDelay = 60.0,
                InitialTurnDuration = 15.0,
                TurnDuration = 110.0,
                ThrustDuration = 333,
                ThrustCurve = 620.0,
                MinThrust = 15.0,
                MaxThrust = 65
            };

            var initialState = SimulationRenderer.CreateInitialState(5.0, Math.PI / 2);

            var optimization = new AscentOptimization(goal, input, initialState, 0.1, 1);
            optimization.Run(true); //First score
            for(var i = 0; i < 150; i++)
            {
                optimization.Run();
            }

            var simulation = new AscentSimulation(
                optimization.BestGuess,
                initialState
            );
            using (var window = new SimulationRenderer(simulation))
            {
                window.Rate = 250;
                window.MicroStepping = 5;
                window.Start(60.0);
            }
        }
    }
}
