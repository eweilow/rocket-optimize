using OpenTK;
using OpenTK.Graphics.OpenGL;
using RocketOptimize.App.Render;
using RocketOptimize.Simulation;
using System;
using System.Diagnostics;

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

        private State _initialState = InitializeState(55.0, Math.PI / 2.01);

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
            //double minX = -Constants.EarthRadius;
            //double maxX = Constants.EarthRadius;
            //double minY = -Constants.EarthRadius;
            //double maxY = Constants.EarthRadius;

            foreach (var state in Simulation.States)
            {
                minX = Math.Min(minX, state.Position.X);
                maxX = Math.Max(maxX, state.Position.X);
                minY = Math.Min(minY, state.Position.Y);
                maxY = Math.Max(maxY, state.Position.Y);
            }

            foreach (var state in Simulation.Lookahead)
            {
                minX = Math.Min(minX, state.Position.X);
                maxX = Math.Max(maxX, state.Position.X);
                minY = Math.Min(minY, state.Position.Y);
                maxY = Math.Max(maxY, state.Position.Y);
            }

            //Camera.CenterOn(minX, maxX, minY, maxY, 2.5, 12000000);
            Camera.CenterOn(minX, maxX, minY, maxY, 1.5, 10000);
        }

        public override SmoothOrthoCamera CreateCamera()
        {
            return new SmoothOrthoCamera(_initialState.Position.X, _initialState.Position.Y, 1000);
        }

        public override void DidResize(int width, int height)
        {
            CenterCameraOnTrajectory();
        }

        DateTime start = DateTime.UtcNow;

        public override void UpdateTick(float updateTime)
        {
            if ((DateTime.UtcNow - start).TotalMilliseconds > 10000)
            {
                double timeSpent = Simulation.Tick(updateTime, Rate, MicroStepping);

                State currentState = Simulation.CurrentState;

                Title = string.Format("{0} - time: {3,0:F}s - altitude: {1,2:F} km - pressure: {2,2:F} kPa - velocity: {4,2:F} km/s - thrust: {5,2:F} m/s^2 - dens: {6,2:F} kg/m^3",
                    Math.Round(1 / 60.0 / (timeSpent)),
                    (currentState.Position.Length - Constants.EarthRadius) / 1000.0,
                    currentState.Atmosphere.Pressure / 1000.0,
                    currentState.Time,
                    currentState.Velocity.Length / 1000.0,
                    currentState.Thrust.Length,
                    currentState.Atmosphere.Density
                );
            }
            CenterCameraOnTrajectory();
            Camera.Update();
        }

        private void DrawCircle(double radius, float R, float G, float B, float lineWidth = 1f)
        {
            GL.LineWidth(lineWidth);
            GL.Color3(R, G, B);
            GL.Begin(PrimitiveType.LineLoop);
            for (var theta = 0.0; theta < Math.PI * 2.0; theta += 0.05)
            {
                GL.Vertex3(Math.Sin(theta) * radius, Math.Cos(theta) * radius, 0);
            }
            GL.End();
        }

        public override void RenderTick(float updateTime)
        {

            for (int exponent = 0; exponent < 3; exponent++)
            {
                double baseNumber = Math.Pow(10.0, exponent);
                for (double f = 1; f < 10; f += 1)
                {
                    double number = f * baseNumber;
                    DrawCircle(Constants.EarthRadius + number * 1000.0, 0.1f, 0.1f, 0.1f);
                }
            }

            DrawCircle(Constants.EarthRadius, 0f, 1f, 0f, 4f);

            GL.LineWidth(1f);
            GL.Begin(PrimitiveType.LineStrip);
            foreach (var state in Simulation.States)
            {
                GL.Color3(1.0, 1.0, 1.0);
                GL.Vertex3(state.Position);
            }
            GL.End();

            var last = Simulation.States[Simulation.States.Count - 1];

            if(last.Position.Length > Constants.EarthRadius + 10000)
            {
                GL.LineWidth(1f);
                GL.Begin(PrimitiveType.LineStrip);
                foreach (var state in Simulation.Lookahead)
                {
                    GL.Color3(0.5, 0.5, 0.5);
                    GL.Vertex3(state.Position);
                }
                GL.End();
            }

            /*GL.LineWidth(1f);
            GL.Begin(PrimitiveType.Lines);

            GL.Color3(1.0, 0.0, 0.0);
            GL.Vertex3(last.Position);
            GL.Vertex3(last.Position + last.Acceleration.Normalized() * Camera.Size * 0.2);

            GL.Color3(1.0, 1.0, 0.0);
            GL.Vertex3(last.Position);
            GL.Vertex3(last.Position + last.Gravity.Normalized() * Camera.Size * 0.12);

            GL.Color3(0.0, 1.0, 1.0);
            GL.Vertex3(last.Position);
            GL.Vertex3(last.Position + last.Drag.Normalized() * Camera.Size * 0.12);

            GL.Color3(1.0, 0.0, 1.0);
            GL.Vertex3(last.Position);
            GL.Vertex3(last.Position + last.Thrust.Normalized() * Camera.Size * 0.12);

            GL.Color3(0.0, 0.0, 1.0);
            GL.Vertex3(last.Position);
            GL.Vertex3(last.Position + last.Velocity.Normalized() * Camera.Size * 0.05);
            GL.End();*/
        }


        static void Main(string[] args)
        {
            using (var window = new Program())
            {
                window.Rate = 5;
                window.MicroStepping = 10;
                window.Start(60.0);
            }
        }
    }
}
