//#define ORBIT_DEBUG
//#define FORCE_DEBUG
//#define USE_LOOKAHEAD_FOR_SCALING

using OpenTK;
using OpenTK.Graphics.OpenGL;
using RocketOptimize.App.Render;
using RocketOptimize.Simulation;
using System;

namespace RocketOptimize.App
{
    public class SimulationRenderer : Window<SmoothOrthoCamera>
    {
        public static State CreateInitialState(double v0, double theta)
        {
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


        public int Rate = 1;
        public int MicroStepping = 1;

        private State _initialState;
        private readonly AscentSimulation _simulation;
        private readonly Timer _startTimer;

        public SimulationRenderer(AscentSimulation simulation) : base()
        {
            _simulation = simulation;
            _initialState = simulation.InitialState;
            _startTimer = new Timer(1000);
            VSync = VSyncMode.On;
        }


        private void CenterCameraOnTrajectory()
        {
            Camera.ResetCenteringContext();

            foreach (var state in _simulation.States)
            {
                Camera.UsePointForCentering(state.Position);
            }

#if USE_LOOKAHEAD_FOR_SCALING
            foreach (var point in _simulation.LookAheadState.FuturePositions)
            {
                Camera.UsePointForCentering(point);
            }
#endif
            Camera.FinishCenteringContext();
        }

        public override SmoothOrthoCamera CreateCamera()
        {
            return new SmoothOrthoCamera(_initialState.Position.X, _initialState.Position.Y, 0.1, 1.8, 10000);
        }

        public override void DidResize(int width, int height)
        {
            CenterCameraOnTrajectory();
        }

        public override void UpdateTick(float updateTime)
        {
            if (_startTimer.IsDone() && _simulation.CurrentState.IsDone != true)
            {
                _simulation.Tick(updateTime, Rate, MicroStepping);
            }
            if(_simulation.isTerminalGuidanceTriggered)
            {
                Rate = 10;
            }
            State currentState = _simulation.CurrentState;
            Title = string.Format("{0,2:F} u {9,2:F} f - t: {3,0:F}s - r: {1,2:F} km | {10,1:F} x {11,1:F} km  - P: {2,2:F} kPa - v: {4,2:F} km/s - thrust: {5,2:F} m/s^2 - rho: {6,2:F} kg/m^3 - g: {7,2:F} m/s - d: {8,2:F} m/s - mf {12,2:F} m {13,2:F}",
                _updateMeasurer.CurrentRate,
                (currentState.Position.Length - Constants.EarthRadius) / 1000.0,
                currentState.Atmosphere.Pressure / 1000.0,
                currentState.Time,
                currentState.Velocity.Length / 1000.0,
                currentState.Thrust.Length,
                currentState.Atmosphere.Density,
                currentState.LossesToGravity,
                currentState.LossesToDrag,
                _renderMeasurer.CurrentRate,
                _simulation.LookAheadState.Apoapsis / 1000.0,
                _simulation.LookAheadState.Periapsis / 1000.0,
                currentState.MassFlow,
                _simulation.ControlInput.Rocket.TotalFuelMass() - currentState.ExpendedMass
            );
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

            var last = _simulation.States[_simulation.States.Count - 1];
            if (last.Position.Length > Constants.EarthRadius + 1000)
            {
                var positions = _simulation.LookAheadState.FuturePositions;
                GL.LineWidth(2f);
                GL.Begin(PrimitiveType.LineStrip);
                for (int i = 0; i < positions.Length; i++)
                {
                    GL.Color3(1.0 - 0.5 * i / positions.Length, 0.0, 0.0);
                    GL.Vertex3(positions[i]);
                }
                GL.End();
            }

            GL.LineWidth(1f);
            GL.Begin(PrimitiveType.LineStrip);
            foreach (var state in _simulation.States)
            {
                GL.Color3(1.0, 1.0, 1.0);
                GL.Vertex3(state.Position);
            }
            GL.End();

#if FORCE_DEBUG
            GL.LineWidth(2f);
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
            GL.End();
#endif
        }


    }
}
