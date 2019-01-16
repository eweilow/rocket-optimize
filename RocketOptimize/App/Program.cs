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
                //TurnDelay = 30.0, //60.0,
                //InitialTurnDuration = 20, //15.0,
                //TurnDuration = 80.0, //210.0,
                //ThrustDuration = 343, //400,
                //ThrustCutoff = 253, //400,
                //TurnDelay = 50,
                //InitialTurnDuration = 30,
                //TurnDuration = 90,
                //ThrustDuration1 = 133.89,
                //ThrustDuration2 = 200.89,
                //ThrustCutoff = 315.50,
                //MinThrust1 = 12.0,
                //MaxThrust1 = 40,
                //MinThrust2 = 25,
                //MaxThrust2 = 45
            };

            var initialState = SimulationRenderer.CreateInitialState(5.0, Math.PI / 2);

            /*
            var optimization = new AscentOptimization(goal, input, initialState, 0.5, 1);
            for(var i = 0; i < 10000; i++)
            {
                double radius = 1.0;
                double r = radius * (5 - (i % 5)) / 5.0;
                optimization.Run(r);
            }*/

            var simulation = new AscentSimulation(
                goal,
                input, //optimization.BestGuess,
                initialState
            );
            using (var window = new SimulationRenderer(simulation))
            {
                window.Rate = 150;
                window.MicroStepping = 5;
                window.Start(60.0);
            }
        }
    }
}
