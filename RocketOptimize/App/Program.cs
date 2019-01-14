using OpenTK;
using OpenTK.Graphics.OpenGL;
using RocketOptimize.App.Render;
using RocketOptimize.Simulation;
using System;

namespace RocketOptimize.App
{
    class Program : Window<SmoothOrthoCamera>
    {
        static State InitializeState(double v0, double theta)
        {
            return new State()
            {
                Position = new Vector3d()
                {
                    X = 0,
                    Y = Constants.EarthRadius,
                    Z = 0
                },
                Velocity = new Vector3d()
                {
                    X = v0 * Constants.Scaling * (float)Math.Cos(theta),
                    Y = v0 * Constants.Scaling * (float)Math.Sin(theta),
                    Z = 0
                }
            };
        }

        private State _initialState = InitializeState(555.0, 1.0);

        readonly AscentSimulation Simulation;

        public Program() : base()
        {
            Simulation = new AscentSimulation(
                new Input[]{
                    new Input(90f, 1f)
                },
                _initialState
            );
        }

        public int Rate = 1;
        public int MicroStepping = 1;

        private void CenterCameraOnTrajectory()
        {
            double minX = double.MaxValue;
            double maxX = double.MinValue;
            double minY = double.MaxValue;
            double maxY = double.MinValue;

            foreach (var state in Simulation.States)
            {
                minX = Math.Min(minX, state.Position.X);
                maxX = Math.Max(maxX, state.Position.X);
                minY = Math.Min(minY, state.Position.Y);
                maxY = Math.Max(maxY, state.Position.Y);
            }

            Camera.CenterOn(minX, maxX, minY, maxY, 2.5, 10000);
        }

        public override SmoothOrthoCamera CreateCamera()
        {
            return new SmoothOrthoCamera(_initialState.Position.X, _initialState.Position.Y, 1000);
        }

        public override void DidResize(int width, int height)
        {
            CenterCameraOnTrajectory();
        }

        public override void UpdateTick(float updateTime)
        {
            double timeSpent = Simulation.Tick(updateTime, Rate, MicroStepping);
            Title = string.Format("{0,2:F}", timeSpent * 1000);

            State currentState = Simulation.CurrentState;
            CenterCameraOnTrajectory();
            Camera.Update();
        }

        public override void RenderTick(float updateTime)
        {
            GL.LineWidth(4f);
            GL.Begin(PrimitiveType.Lines);
            GL.Color3(0f, 1f, 0f);
            GL.Vertex3(-5000.0, Constants.EarthRadius, 0.0);
            GL.Vertex3(5000.0, Constants.EarthRadius, 0.0);
            GL.End();

            GL.LineWidth(1f);
            GL.Begin(PrimitiveType.Lines);
            for (double f = 1; f < 1000; f += 1)
            {
                GL.Color3(0.1, 0.1, 0.1);
                GL.Vertex3(-5000, Constants.EarthRadius + f * 1000.0, 0);
                GL.Vertex3(5000, Constants.EarthRadius + f * 1000, 0);
            }
            GL.End();

            GL.Begin(PrimitiveType.LineStrip);
            foreach (var state in Simulation.States)
            {
                GL.Color3(1.0, 1.0, 1.0);
                GL.Vertex3(state.Position);
            }
            GL.End();
        }


        static void Main(string[] args)
        {
            using (var window = new Program())
            {
                window.Rate = 5;
                window.MicroStepping = 100;
                window.Start(60.0);
            }
        }
    }
}
