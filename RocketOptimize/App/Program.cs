#define FORCE_DEBUG
#define USE_LOOKAHEAD_FOR_SCALING

using OpenTK;
using OpenTK.Graphics.OpenGL;
using RocketOptimize.App.Render;
using RocketOptimize.Simulation;
using System;


namespace RocketOptimize.App
{
    class Program
    {
        static void Main(string[] args)
        {
            var initialState = SimulationRenderer.CreateInitialState(55.0, Math.PI / 2.01);
            using (var window = new SimulationRenderer(initialState))
            {
                window.Rate = 250;
                window.MicroStepping = 5;
                window.Start(60.0);
            }
        }
    }
}
