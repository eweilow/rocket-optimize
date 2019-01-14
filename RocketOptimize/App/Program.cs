using OpenTK.Graphics.OpenGL;
using RocketOptimize.App.Render;
using RocketOptimize.Simulation;
using RocketOptimize.Simulation.Integrators;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;

namespace RocketOptimize.App
{
    class Program : Window
    {
        static State InitializeState(float v0, float theta)
        {
            return new State()
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
        }

        State currentState = InitializeState(555f, 0.45f);
        RK4 rk4 = new RK4();

        List<State> states = new List<State>();

        public int Rate = 1;
        public int MicroStepping = 1;

        public override void DidResize(int width, int height)
        {
            Camera.Resize(5000, 5000);
            Camera.SetProjectionOrthographic(-1f, 1f);
        }

        public override void UpdateTick(float updateTime)
        {
            var watch = new Stopwatch();
            watch.Start();
            for (int i = 0; i < Rate; i++)
            {
                for (int j = 0; j < MicroStepping; j++)
                {
                    rk4.Integrate(updateTime / MicroStepping, ref currentState, out currentState, (State state) =>
                    {
                        if (state.Position.Y < -0.1f)
                        {
                            return -state.Velocity * 10f;
                        }
                        return Vector3.UnitY * (-9.81f) - 0.001f * state.Velocity * state.Velocity.Length();
                    });
                }
                states.Add(currentState);
            }
            watch.Stop();
            double ticks = watch.ElapsedTicks;
            double seconds = ticks / Stopwatch.Frequency;

            Title = string.Format("{0,2:F}", seconds * 1000);

            Camera.SetPosition(new OpenTK.Vector3() { X = currentState.Position.X, Y = currentState.Position.Y, Z = currentState.Position.Z });
            Camera.SetLookat(new OpenTK.Vector3() { X = currentState.Position.X, Y = currentState.Position.Y, Z = currentState.Position.Z - 1 });
        }

        public override void RenderTick(float updateTime)
        {
            // Console.WriteLine("{0} ({1,2:F} {2,2:F}) {3,2:F} ({4,2:F})", 1f / updateTime, currentState.Position.X, currentState.Position.Y, currentState.Velocity.X, currentState.Velocity.Y);

            GL.LineWidth(4f);
            GL.Begin(PrimitiveType.Lines);
            GL.Color3(0f, 1f, 0f);
            GL.Vertex3(-5000f, 0f, 0f);
            GL.Vertex3(5000f, 0f, 0f);
            GL.End();

            GL.LineWidth(1f);
            GL.Begin(PrimitiveType.Lines);
            for (float f = 1f; f < 100f; f += 1f)
            {
                GL.Color3(0.1f, 0.1f, 0.1f);
                GL.Vertex3(-5000f, f*1000f, 0f);
                GL.Vertex3(5000f, f*1000f, 0f);
            }
            GL.End();

            GL.Begin(PrimitiveType.LineStrip);
            foreach (var state in states)
            {
                GL.Color3(1f, 1f, 1f);
                GL.Vertex3(state.Position.X, state.Position.Y, state.Position.Z);
            }
            GL.End();
        }


        static void Main(string[] args)
        {
            using (var window = new Program())
            {
                window.Rate = 1;
                window.MicroStepping = 100;
                window.Start(60.0);
            }
        }
    }
}
