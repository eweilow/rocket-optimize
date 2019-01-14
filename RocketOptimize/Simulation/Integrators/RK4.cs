using System;
using OpenTK;

namespace RocketOptimize.Simulation.Integrators
{
    public class RK4 : IIntegrator
    {
        public RK4()
        {
        }

        public void Integrate(double timeStep, ref State currentState, out State newState, AccelerationCalculator calculateAcceleration)
        {
            var a1 = calculateAcceleration(ref currentState) * timeStep;
            var b1 = currentState.Velocity * timeStep;

            var state2 = new State()
            {
                Time = currentState.Time + timeStep / 2.0,
                Position = currentState.Position + b1 / 2.0,
                Velocity = currentState.Velocity + a1 / 2.0
            };
            var a2 = calculateAcceleration(ref state2) * timeStep;
            var b2 = state2.Velocity * timeStep;

            var state3 = new State()
            {
                Time = currentState.Time + timeStep / 2.0,
                Position = currentState.Position + b2 / 2.0,
                Velocity = currentState.Velocity + a2 / 2.0
            };
            var a3 = calculateAcceleration(ref state3) * timeStep;
            var b3 = state3.Velocity * timeStep;

            var state4 = new State()
            {
                Time = currentState.Time + timeStep,
                Position = currentState.Position + b3,
                Velocity = currentState.Velocity + a3
            };
            var a4 = calculateAcceleration(ref state3) * timeStep;
            var b4 = state3.Velocity * timeStep;

            newState = new State()
            {
                Time = currentState.Time + timeStep,
                Position = currentState.Position + 1.0 / 6.0 * b1 + 1.0 / 3.0 * b2 + 1.0 / 3.0 * b3 + 1.0 / 6.0 * b4,
                Velocity = currentState.Velocity + 1.0 / 6.0 * a1 + 1.0 / 3.0 * a2 + 1.0 / 3.0 * a3 + 1.0 / 6.0 * a4,
                Acceleration = currentState.Acceleration,
                Gravity = currentState.Gravity,
                Drag = currentState.Drag,
                Thrust = currentState.Thrust
            };
        }
    }
}
