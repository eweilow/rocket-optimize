using RocketOptimize.Simulation.Integrators;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;

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

        private Vector3 ComputeCurrentAcceleration(State state)
        {
            if (state.Position.Y < -0.1f)
            {
                return -state.Velocity * 10f;
            }
            return Vector3.UnitY * (-9.81f) - 0.001f * state.Velocity * state.Velocity.Length();
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
                    if ((_lastState.Position - _currentState.Position).LengthSquared() > 10f)
                    {
                        States.Add(_currentState);
                        _lastState = _currentState;
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
