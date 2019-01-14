using OpenTK;
using RocketOptimize.Simulation.Integrators;
using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace RocketOptimize.Simulation
{
    public class AscentSimulation
    {
        public readonly Input[] ControlInput;
        public readonly List<State> States = new List<State>();
        public readonly State[] Lookahead = new State[600];
        public readonly double LookaheadTimestep = 1;
        public readonly int LookaheadRate = 10;

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

            for(var i = 0; i < Lookahead.Length; i++)
            {
                Lookahead[i] = initialState;
            }
        }

        private Vector3d ComputeCurrentNaturalAcceleration(ref State state, bool clamp)
        {
            double radius = state.Position.Length;
            if (clamp && radius < Constants.EarthRadius - 0.1)
            {
                return -state.Velocity;
            }
            Vector3d heading = state.Velocity.Normalized();

            const double CoefficientOfDrag = 0.3;
            const double Radius = 3.66 / 2;
            const double Area = Radius * Radius * Math.PI;
            const double Mass = 500000;

            double velocitySquared = state.Velocity.LengthSquared;
            state.Atmosphere = Models.AtmosphereLerp.Get(radius - Constants.EarthRadius);
            state.Gravity = Models.Gravity(Constants.EarthGravitationalConstant, state.Position, Vector3d.Zero);
            state.Drag = -heading * velocitySquared * CoefficientOfDrag * Area * state.Atmosphere.Density / (2.0 * Mass);

            return state.Acceleration = state.Gravity + state.Drag;
        }

        private Vector3d ComputeCurrentNaturalAcceleration(ref State state)
        {
            return ComputeCurrentNaturalAcceleration(ref state, true);
        }

        private Vector3d ComputeCurrentAcceleration(ref State state)
        {
            double radius = state.Position.Length;
            if (radius < Constants.EarthRadius - 0.1)
            {
                return -state.Velocity;
            }

            Vector3d vertical = state.Position.Normalized();
            Vector3d horizontal = Vector3d.Cross(vertical, Vector3d.UnitZ).Normalized();

            const double turnDelay = 60.0;
            const double initialTurnDuration = 15.0;
            const double turnDuration = 110.0;
            const double thrustDuration = 333;
            const double thrustCurve = 620.0;
            const double minThrust = 15.0;
            const double maxThrust = 65;

            double angle = 10 * Math.Min(1.0, state.Time / initialTurnDuration) + 80.0 * Math.Min(1.0, Math.Max(0.0, state.Time - turnDelay) / turnDuration);
            double thrust = state.Time < thrustDuration ? minThrust + (maxThrust - minThrust) * (Math.Min(1.0, state.Time / thrustCurve)) : 0.0;
            double radian = angle / 180.0 * Math.PI;

            Vector3d thrustDirection = Math.Cos(radian) * vertical + Math.Sin(radian) * horizontal;
            state.Thrust = thrustDirection * thrust;

            return state.Acceleration = ComputeCurrentNaturalAcceleration(ref state, false) + state.Thrust;
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
                    double radius = _currentState.Position.Length;
                    if (radius < Constants.EarthRadius - 0.1)
                    {
                        _currentState.Position = _currentState.Position.Normalized() * (Constants.EarthRadius - 0.1);
                        _currentState.Velocity = Vector3d.Zero;
                    }
                }
                if ((_lastState.Position - _currentState.Position).LengthSquared > 100.0)
                {
                    ComputeCurrentAcceleration(ref _currentState);
                    States.Add(_currentState);
                    _lastState = _currentState;
                }
            }

            ComputeCurrentAcceleration(ref _currentState);
            States.Add(_currentState);
            _lastState = _currentState;

            Lookahead[0] = _currentState;
            bool exited = false;
            for (var i = 1; i < Lookahead.Length; i++)
            {
                Lookahead[i] = Lookahead[i - 1];
                if(exited)
                {
                    continue;
                }
                for (var j = 0; j < LookaheadRate; j++)
                {
                    _integrator.Integrate(LookaheadTimestep, ref Lookahead[i], out Lookahead[i], ComputeCurrentNaturalAcceleration);
                    double radius = Lookahead[i].Position.Length;
                    if (radius < Constants.EarthRadius + 10000.0)
                    {
                        Lookahead[i].Position = Lookahead[i].Position.Normalized() * (Constants.EarthRadius - 0.1);
                        Lookahead[i].Velocity = Vector3d.Zero;
                        exited = true;
                        break;
                    }
                }
            }

            watch.Stop();
            double ticks = watch.ElapsedTicks;
            double seconds = ticks / Stopwatch.Frequency;
            return seconds;
        }
    }
}
