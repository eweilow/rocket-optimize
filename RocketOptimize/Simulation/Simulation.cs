using RocketOptimize.Simulation.Integrators;
using System.Collections.Generic;
using System.Diagnostics;
using OpenTK;

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

        private Vector3d ComputeCurrentAcceleration(State state)
        {
            if (state.Position.Y < Constants.EarthRadius - 0.1)
            {
                return -state.Velocity * 10;
            }
            return Vector3d.UnitY * (-9.81) - 0.001 * state.Velocity * state.Velocity.Length;
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
                    if ((_lastState.Position - _currentState.Position).LengthSquared > 10.0)
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
