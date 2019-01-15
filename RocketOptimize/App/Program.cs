//#define ORBIT_DEBUG
#define USE_LOOKAHEAD_FOR_SCALING

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
#if ORBIT_DEBUG
            return new State()
            {
                Position = new Vector3d()
                {
                    X = 5000,
                    Y = Constants.EarthRadius + 15000,
                    Z = 0
                },
                Velocity = new Vector3d()
                {
                    X = 500,
                    Y = 2500,
                    Z = 0
                }
            };
#else
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
#endif
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
#if !ORBIT_DEBUG
            double minX = double.MaxValue;
            double maxX = double.MinValue;
            double minY = double.MaxValue;
            double maxY = double.MinValue;
#else
            double minX = -Constants.EarthRadius;
            double maxX = Constants.EarthRadius;
            double minY = -Constants.EarthRadius;
            double maxY = Constants.EarthRadius;
#endif

            foreach (var state in Simulation.States)
            {
                if(double.IsNaN(state.Position.X) || double.IsNaN(state.Position.Y))
                {
                    continue;
                }
                minX = Math.Min(minX, state.Position.X);
                maxX = Math.Max(maxX, state.Position.X);
                minY = Math.Min(minY, state.Position.Y);
                maxY = Math.Max(maxY, state.Position.Y);
            }

#if USE_LOOKAHEAD_FOR_SCALING
            foreach (var state in Simulation.LookAheadState.FuturePositions)
            {
                if (double.IsNaN(state.X) || double.IsNaN(state.Y))
                {
                    continue;
                }
                minX = Math.Min(minX, state.X);
                maxX = Math.Max(maxX, state.X);
                minY = Math.Min(minY, state.Y);
                maxY = Math.Max(maxY, state.Y);
            }
#endif
            //Console.WriteLine("{0,2:F} {1,2:F} {2,2:F} {3,2:F}", minX, maxX, minY, maxY);
            Camera.CenterOn(minX, maxX, minY, maxY, 1.8, 10000);
        }

        public override SmoothOrthoCamera CreateCamera()
        {
            Console.WriteLine(_initialState.Position);
            return new SmoothOrthoCamera(_initialState.Position.X, _initialState.Position.Y, 10000);
        }

        public override void DidResize(int width, int height)
        {
            CenterCameraOnTrajectory();
        }

        DateTime start = DateTime.UtcNow;

        public override void UpdateTick(float updateTime)
        {
            if ((DateTime.UtcNow - start).TotalMilliseconds > 500)
            {
                double timeSpent = Simulation.Tick(updateTime, Rate, MicroStepping);

                State currentState = Simulation.CurrentState;

                Title = string.Format("{0} - time: {3,0:F}s - altitude: {1,2:F} km - pressure: {2,2:F} kPa - velocity: {4,2:F} km/s - thrust: {5,2:F} m/s^2 - dens: {6,2:F} kg/m^3 - grav: {7,2:F} m/s - drag: {8,2:F} m/s",
                    Math.Round(1 / 60.0 / (timeSpent)),
                    (currentState.Position.Length - Constants.EarthRadius) / 1000.0,
                    currentState.Atmosphere.Pressure / 1000.0,
                    currentState.Time,
                    currentState.Velocity.Length / 1000.0,
                    currentState.Thrust.Length,
                    currentState.Atmosphere.Density,
                    currentState.LossesToGravity,
                    currentState.LossesToDrag
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
            for (var theta = 0.0; theta < Math.PI * 2.0; theta += 0.02)
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

            var last = Simulation.States[Simulation.States.Count - 1];
            //if (last.Position.Length > Constants.EarthRadius + 10000)
            //{
                var positions = Simulation.LookAheadState.FuturePositions;
                GL.LineWidth(2f);
                GL.Begin(PrimitiveType.LineStrip);
                for (int i = 0; i < positions.Length; i++)
                {
                    GL.Color3(1.0 - 0.5 * i / positions.Length, 0.0, 0.0);
                    GL.Vertex3(positions[i]);
                }
                GL.End();
            //}

            GL.LineWidth(1f);
            GL.Begin(PrimitiveType.LineStrip);
            foreach (var state in Simulation.States)
            {
                GL.Color3(1.0, 1.0, 1.0);
                GL.Vertex3(state.Position);
            }
            GL.End();

            /*if (last.Position.Length > Constants.EarthRadius + 10000)
            {
                GL.LineWidth(1f);
                GL.Begin(PrimitiveType.LineStrip);
                foreach (var state in Simulation.Lookahead)
                {
                    GL.Color3(0.5, 0.5, 0.5);
                    GL.Vertex3(state.Position);
                }
                GL.End();
            }*/



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
                window.MicroStepping = 100;
                window.Start(60.0);
            }
        }
    }
}
