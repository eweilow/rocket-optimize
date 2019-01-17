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

        public readonly AscentSimulationGoal Goal;
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

        public double WindSurfaceVelocity;

        public AscentSimulation(
            AscentSimulationGoal goal,
            AscentSimulationControl controlInput,
            State initialState,
            double windSurfaceVelocity = 0.0
        )
        {
            Goal = goal;
            ControlInput = controlInput;
            States.Add(initialState);
            _lastState = initialState;
            _currentState = initialState;
            InitialState = initialState;
            WindSurfaceVelocity = windSurfaceVelocity;

            for (var i = 0; i < LookAheadState.FuturePositions.Length; i++)
            {
                LookAheadState.FuturePositions[i] = initialState.Position;
            }
        }

        private double _currentRadius;
        private double _currentAltitude;

        private Vector3d _vertical;
        private Vector3d _horizontal;

        private void PrecomputeState(ref State state)
        {
            _currentRadius = state.Position.Length;
            _currentAltitude = _currentRadius - Constants.EarthRadius;
            _vertical = state.Position / _currentRadius;
            _horizontal = Vector3d.Cross(_vertical, Vector3d.UnitZ).Normalized(); // vertical and Z should be perpendicular;
        }

        private Vector3d ComputeCurrentNaturalAcceleration(ref State state, double mass, bool clamp)
        {
            if (clamp && _currentAltitude < 0.1)
            {
                return -state.Velocity;
            }

            state.Atmosphere = Models.AtmosphereLerp.Get(_currentRadius - Constants.EarthRadius);
            state.Gravity = Models.Gravity(Constants.EarthGravitationalConstant, state.Position, Vector3d.Zero);

            double angularVelocity = WindSurfaceVelocity / Constants.EarthRadius;

            Vector3d windVelocityVector = _horizontal * angularVelocity * _currentRadius;
            Vector3d windRelativeVelocity = state.Velocity - windVelocityVector;
            double velocity = windRelativeVelocity.Length;
            double velocitySquared = velocity * velocity;

            double MachNumber = velocity / 1000.0;
            double CoefficientOfDrag;
            if (MachNumber < 1)
            {
                CoefficientOfDrag = 0.3 + MachNumber;
            }
            else
            {
                CoefficientOfDrag = 1.3 * Math.Exp(-MachNumber);
            }

            const double Radius = 5 / 2;
            const double Area = Radius * Radius * Math.PI;

            state.Drag = -windRelativeVelocity.Normalized() * velocitySquared * CoefficientOfDrag * Area * state.Atmosphere.Density / (2.0 * mass);
            //Console.WriteLine("{0} {1}", state.Gravity, state.Drag);
            return state.Acceleration = state.Gravity + state.Drag;
        }

        private Vector3d ComputeCurrentNaturalAcceleration(ref State state)
        {
            return ComputeCurrentNaturalAcceleration(ref state, ControlInput.ComputeMass(state.ReachedAltitude, state.ExpendedMass), true);
        }

        public bool isTerminalGuidanceTriggered = false;
        private Vector3d terminalGuidanceThrustDirection;
        private double terminalGuidanceThrustValue;
        private void TerminalGuidanceThrust(double dt, ref State state, double radius)
        {
            if(isTerminalGuidanceTriggered == false)
            {
                return;
            }

            PrecomputeState(ref state);
            
            ControlInput.TerminalGuidance(Goal, dt,
                (state.Position.Length - Constants.EarthRadius) / 1000,
                Vector3d.Dot(state.Velocity, _vertical),
                LookAheadState.Periapsis / 1000,
                LookAheadState.Apoapsis / 1000,
                out double pitch,
                out double thrust,
                out bool isDone
            );
            if(isDone)
            {
                _currentState.IsDone = true;
            }

            double thrustValue = ControlInput.ComputeThrust(state.Time, thrust, state.ExpendedMass, state.Atmosphere.Pressure, out state.MassFlow);

            terminalGuidanceThrustDirection = Math.Cos(Math.PI/2 - pitch) * _vertical + Math.Sin(Math.PI / 2 - pitch) * _horizontal;
            terminalGuidanceThrustValue = thrustValue;

            //Console.WriteLine("{0,2:F} {1,2:F} = {2,2:F}", pitch, thrust, thrustValue);
        }

        private Vector3d ComputeCurrentAcceleration(double dt, ref State state)
        {
            PrecomputeState(ref state);

            double mass = ControlInput.ComputeMass(state.ReachedAltitude, state.ExpendedMass);
            var naturalAcceleration = ComputeCurrentNaturalAcceleration(ref state, mass, false);


            if (_currentAltitude < 0.1)
            {
                return -state.Velocity;
            }

            double verticalVelocity = Vector3d.Dot(state.Velocity, _vertical);
            var initiallyWasGuidanceTriggered = isTerminalGuidanceTriggered;

            // Console.WriteLine("{0} {1,2:F} {2,2:F}", isTerminalGuidanceTriggered, LookAheadState.Apoapsis / 1000, Goal.Apoapsis);
            if (isTerminalGuidanceTriggered ||
                (_currentAltitude/1000 > Goal.Apoapsis - 55) || (_currentAltitude/100 > 100 && Math.Abs(verticalVelocity) < 50)
            // || LookAheadState.Apoapsis / 1000 > Goal.Apoapsis
            )
            {
                isTerminalGuidanceTriggered = true;
                if (initiallyWasGuidanceTriggered)
                {
                    state.Thrust = terminalGuidanceThrustDirection * Math.Min(ControlInput.MaxAcceleration, terminalGuidanceThrustValue / mass);
                }
                else
                {
                    //Console.WriteLine("Terminal guidance triggered");
                }
            }

            if(!initiallyWasGuidanceTriggered)
            {
                double angle = ControlInput.ComputeAngle(state.Time, state.ExpendedMass);
                double thrustValue = ControlInput.ComputeThrust(state.Time, 1.0, state.ExpendedMass, state.Atmosphere.Pressure, out state.MassFlow);

                Vector3d thrustDirection = Math.Cos(angle) * _vertical + Math.Sin(angle) * _horizontal;
                state.Thrust = thrustDirection * Math.Min(ControlInput.MaxAcceleration, thrustValue / mass);
            }


            state.Acceleration = naturalAcceleration + state.Thrust;

            return state.Acceleration;
        }

        public void FastTick(double updateTime, int microSteps)
        {
            for (int j = 0; j < microSteps; j++)
            {
                TerminalGuidanceThrust(updateTime / microSteps, ref _currentState, _currentState.Position.Length);
                _integrator.Integrate(updateTime / microSteps, ref _currentState, out _currentState, ComputeCurrentAcceleration);
                _currentState.ReachedAltitude = Math.Max(_currentState.ReachedAltitude, _currentAltitude);
                if(_currentAltitude < -0.1)
                {
                    _currentState.IsDone = true;
                }
            }
            LookAhead.CalculateOrbit(ref LookAheadState, _currentState);
        }

        public void Tick(double updateTime, int rate, int microSteps)
        {
            for (int i = 0; i < rate; i++)
            {
                for (int j = 0; j < microSteps; j++)
                {
                    TerminalGuidanceThrust(updateTime / microSteps, ref _currentState, _currentState.Position.Length);
                    _integrator.Integrate(updateTime / microSteps, ref _currentState, out _currentState, ComputeCurrentAcceleration);
                    _currentState.ReachedAltitude = Math.Max(_currentState.ReachedAltitude, _currentAltitude);
                    
                    double radius = _currentState.Position.Length;
                    if (radius < Constants.EarthRadius - 0.1)
                    {
                        _currentState.Position = _currentState.Position.Normalized() * (Constants.EarthRadius - 0.1);
                        _currentState.Velocity = Vector3d.Zero;
                        _currentState.IsDone = true;
                    }
                }
                if ((_lastState.Position - _currentState.Position).LengthSquared > 100.0)
                {
                    ComputeCurrentAcceleration(updateTime, ref _currentState);
                    States.Add(_currentState);
                    _lastState = _currentState;
                }
            }

            ComputeCurrentAcceleration(updateTime, ref _currentState);
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
