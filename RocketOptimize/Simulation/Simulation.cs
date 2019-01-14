using RocketOptimize.Simulation.Integrators;
using System.Collections.Generic;
using System.Diagnostics;
using OpenTK;
using System;

namespace RocketOptimize.Simulation
{
    public class AscentSimulation
    {
        public readonly Input[] ControlInput;
        public readonly List<State> States = new List<State>();
        private State _currentState;
        private State _lastState;
        private IIntegrator _integrator = new RK4();

        public State CurrentState
        {
            get
            {
                return _currentState;
            }
        }

        public AscentSimulation(
            Input[] controlInput,
            State initialState
        )
        {
            ControlInput = controlInput;
            States.Add(initialState);
            _lastState = initialState;
            _currentState = initialState;
        }

        private Vector3d ComputeCurrentAcceleration(ref State state)
        {
            double radius = state.Position.Length;
            if (radius < Constants.EarthRadius - 0.1)
            {
                return -state.Velocity * 10;
            }
            Vector3d heading = state.Velocity.Normalized();

            double velocitySquared = state.Velocity.LengthSquared;

            state.Atmosphere = Models.AtmosphereLerp.Get(radius - Constants.EarthRadius);
            state.Gravity = Models.Gravity(Constants.EarthGravitationalConstant, state.Position, Vector3d.Zero);
            state.Drag = -heading * velocitySquared * state.Atmosphere.Density / 1000.0;


            Vector3d vertical = state.Position.Normalized();
            Vector3d horizontal = Vector3d.Cross(vertical, Vector3d.UnitZ).Normalized();

            double angle = 90.0 * Math.Min(1.0, state.Time / 350.0);
            double radian = angle / 180.0 * Math.PI;

            Vector3d thrustDirection = Math.Cos(radian) * vertical + Math.Sin(radian) * horizontal;
            state.Thrust = thrustDirection * 25.0;

            return state.Acceleration = state.Gravity + state.Drag + state.Thrust;
        }

        public double Tick(float updateTime, int rate, int microSteps)
        {
            var watch = new Stopwatch();
            watch.Start();
            for (int i = 0; i < rate; i++)
            {
                for (int j = 0; j < microSteps; j++)
                {
                    _integrator.Integrate(updateTime / microSteps, ref _currentState, out _currentState, ComputeCurrentAcceleration);
                }
                if ((_lastState.Position - _currentState.Position).LengthSquared > 100.0)
                {
                    ComputeCurrentAcceleration(ref _currentState);
                    States.Add(_currentState);
                    _lastState = _currentState;
                }
            }
            watch.Stop();
            double ticks = watch.ElapsedTicks;
            double seconds = ticks / Stopwatch.Frequency;

            ComputeCurrentAcceleration(ref _currentState);
            States.Add(_currentState);
            _lastState = _currentState;

            return seconds;
        }
    }
}
