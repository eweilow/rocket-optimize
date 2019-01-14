using RocketOptimize.Simulation;
using RocketOptimize.Simulation.Integrators;
using System;
using System.Numerics;

namespace RocketOptimize.App
{
    class Program
    {
        static void Main(string[] args)
        {
            var controlVectors = new Input[]{
                new Input(90f, 1f),
                new Input(80f, 1f),
                new Input(70f, 1f),
                new Input(60f, 1f),
                new Input(50f, 1f),
                new Input(40f, 1f),
                new Input(30f, 1f),
                new Input(20f, 1f),
                new Input(10f, 1f),
                new Input(0f,  1f)
            };

            var simulation = new AscentSimulation();
            simulation.Run(controlVectors);

            var theta = 0.25f;
            var v0 = 555f;
            var currentState = new State()
            {
                Position = new Vector3()
                {
                    X = 0,
                    Y = 0,
                    Z = 0
                },
                Velocity = new Vector3()
                {
                    X = v0 * (float)Math.Cos(theta),
                    Y = v0 * (float)Math.Sin(theta),
                    Z = 0
                }
            };

            var rk4 = new RK4();


            while (currentState.Position.Y >= -0.1f)
            {
                rk4.Integrate(0.0166666f, ref currentState, out currentState, (State state) =>
                {
                    return Vector3.UnitY * (-9.81f);
                });
            }
            Console.WriteLine("{0} ({1,2:F} {2,2:F}) {3,2:F} ({4,2:F})", currentState.Time, currentState.Position.X, currentState.Position.Y, currentState.Velocity.X, currentState.Velocity.Y);
            Console.WriteLine("{0} ({1,2:F} {2,2:F}) {3,2:F} ({4,2:F})",
                currentState.Time,
                v0 * Math.Cos(theta) * currentState.Time,
                v0 * Math.Sin(theta) * currentState.Time - 0.5 * 9.81 * currentState.Time * currentState.Time,
                v0 * Math.Cos(theta),
                v0 * Math.Sin(theta) - 9.81 * currentState.Time
            );
            Console.WriteLine("{0} ({1,2:F} {2,2:F}) {3,2:F} ({4,2:F})",
                currentState.Time,
                v0 * Math.Cos(theta) * currentState.Time - currentState.Position.X,
                v0 * Math.Sin(theta) * currentState.Time - 0.5 * 9.81 * currentState.Time * currentState.Time - currentState.Position.Y,
                v0 * Math.Cos(theta) - currentState.Velocity.X,
                v0 * Math.Sin(theta) - 9.81 * currentState.Time - currentState.Velocity.Y
            );
            Console.ReadLine();
        }
    }
}
