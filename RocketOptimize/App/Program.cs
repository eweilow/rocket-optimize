using OpenTK.Graphics.OpenGL;
using RocketOptimize.App.Render;
using RocketOptimize.Simulation;
using System;
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

        readonly AscentSimulation Simulation = new AscentSimulation(
            new Input[]{
                new Input(90f, 1f)
            },
            InitializeState(555f, 1f)
        );

        public int Rate = 1;
        public int MicroStepping = 1;

        public override void DidResize(int width, int height)
        {
            Camera.Resize(5000, 5000);
            Camera.SetProjectionOrthographic(-1f, 1f);
        }

        public override void UpdateTick(float updateTime)
        {
            double timeSpent = Simulation.Tick(updateTime, Rate, MicroStepping);
            Title = string.Format("{0,2:F}", timeSpent * 1000);

            State currentState = Simulation.CurrentState;

            //Camera.SetPosition(new OpenTK.Vector3() { X = currentState.Position.X, Y = currentState.Position.Y, Z = currentState.Position.Z });
            //Camera.SetLookat(new OpenTK.Vector3() { X = currentState.Position.X, Y = currentState.Position.Y, Z = currentState.Position.Z - 1 });
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
                GL.Vertex3(-5000f, f * 1000f, 0f);
                GL.Vertex3(5000f, f * 1000f, 0f);
            }
            GL.End();

            GL.Begin(PrimitiveType.LineStrip);
            foreach (var state in Simulation.States)
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
                window.Rate = 10;
                window.MicroStepping = 100;
                window.Start(60.0);
            }
        }
    }
}
