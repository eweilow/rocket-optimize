#define FAST_LOOKAHEAD

using OpenTK;
using RocketOptimize.Simulation.Integrators;
using System;
using System.Collections.Generic;

namespace RocketOptimize.Simulation
{
    public class AscentSimulation
    {
        public readonly List<State> States = new List<State>();

#if FAST_LOOKAHEAD
        public LookAheadState LookAheadState = new LookAheadState(15000);
#else
        public LookAheadState LookAheadState = new LookAheadState(450);
        public double LookAheadTime = 500.0; // Approximately slightly more than 1 ISS orbit duration
        public int LookAheadMicroSteps = 1; // Microsteps per saved step
#endif

        public readonly AscentSimulationControl ControlInput;
        public readonly State InitialState;
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
            AscentSimulationControl controlInput,
            State initialState
        )
        {
            ControlInput = controlInput;
            States.Add(initialState);
            _lastState = initialState;
            _currentState = initialState;
            InitialState = initialState;

            for (var i = 0; i < LookAheadState.FuturePositions.Length; i++)
            {
                LookAheadState.FuturePositions[i] = initialState.Position;
            }
        }

        private Vector3d ComputeCurrentNaturalAcceleration(ref State state, bool clamp)
        {
            double radius = state.Position.Length;
            if (clamp && radius < Constants.EarthRadius - 0.1)
            {
                return -state.Velocity;
            }
            double velocity = state.Velocity.Length;
            Vector3d heading = state.Velocity / velocity;


            double velocitySquared = state.Velocity.LengthSquared;
            state.Atmosphere = Models.AtmosphereLerp.Get(radius - Constants.EarthRadius);
            state.Gravity = Models.Gravity(Constants.EarthGravitationalConstant, state.Position, Vector3d.Zero);

            double MachNumber = velocity / 1000.0;
            double CoefficientOfDrag;
            if(MachNumber < 1)
            {
                CoefficientOfDrag = 0.3 + MachNumber;
            } else
            {
                CoefficientOfDrag = 1.3 * Math.Exp(-MachNumber);
            }

            const double Radius = 5 / 2;
            const double Area = Radius * Radius * Math.PI;
            double Mass = 500000 - 450000 * Math.Min(1.0, state.Time/300.0);

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

            double angle = ControlInput.ComputeAngle(state.Time);
            double thrust = ControlInput.ComputeThrust(state.Time);

            Vector3d thrustDirection = Math.Cos(angle) * vertical + Math.Sin(angle) * horizontal;
            state.Thrust = thrustDirection * thrust;

            return state.Acceleration = ComputeCurrentNaturalAcceleration(ref state, false) + state.Thrust;
        }

        public void FastTick(double updateTime, int microSteps)
        {
            for (int j = 0; j < microSteps; j++)
            {
                _integrator.Integrate(updateTime / microSteps, ref _currentState, out _currentState, ComputeCurrentAcceleration);
            }
            LookAhead.CalculateOrbit(ref LookAheadState, _currentState);
        }

        public void Tick(double updateTime, int rate, int microSteps)
        {
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

#if FAST_LOOKAHEAD
            LookAhead.CalculateOrbit(ref LookAheadState, _currentState);
#else
            if (_currentState.Atmosphere.Pressure < 1000.0)
            {
                LookAhead.CalculateOrbit(ref LookAheadState, _currentState);
            }
            else
            {

                var positions = LookAheadState.FuturePositions;
                var timestep = LookAheadTime / (LookAheadMicroSteps * positions.Length);
                bool exited = false;
                var lookAheadCurrentState = _currentState;
                positions[0] = _currentState.Position;
                for (var i = 1; i < positions.Length; i++)
                {
                    positions[i] = positions[i - 1];
                    if (exited)
                    {
                        continue;
                    }
                    for (var j = 0; j < LookAheadMicroSteps; j++)
                    {
                        _integrator.Integrate(timestep, ref lookAheadCurrentState, out lookAheadCurrentState, ComputeCurrentNaturalAcceleration);
                        double radius = lookAheadCurrentState.Position.Length;
                        if (radius < Constants.EarthRadius)
                        {
                            positions[i] = lookAheadCurrentState.Position;
                            LookAhead.Intersect(positions, i, out positions[i]);
                            exited = true;
                            break;
                        }
                    }
                    if(!exited)
                    {
                        positions[i] = lookAheadCurrentState.Position;
                    }
                }
            }
#endif
        }
    }
}
