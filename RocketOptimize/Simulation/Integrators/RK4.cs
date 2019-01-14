using System;
using OpenTK;

namespace RocketOptimize.Simulation.Integrators
{
    public class RK4 : IIntegrator
    {
        public RK4()
        {
        }

        public void Integrate(float timeStep, ref State currentState, out State newState, AccelerationCalculator calculateAcceleration)
        {
            var a1 = calculateAcceleration(ref currentState) * timeStep;
            var b1 = currentState.Velocity * timeStep;

            var state2 = new State()
            {
                Time = currentState.Time + timeStep / 2f,
                Position = currentState.Position + b1 / 2f,
                Velocity = currentState.Velocity + a1 / 2f
            };
            var a2 = calculateAcceleration(ref state2) * timeStep;
            var b2 = state2.Velocity * timeStep;

            var state3 = new State()
            {
                Time = currentState.Time + timeStep / 2f,
                Position = currentState.Position + b2 / 2f,
                Velocity = currentState.Velocity + a2 / 2f
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
                Position = currentState.Position + 1f / 6f * b1 + 1f / 3f * b2 + 1f / 3f * b3 + 1f / 6f * b4,
                Velocity = currentState.Velocity + 1f / 6f * a1 + 1f / 3f * a2 + 1f / 3f * a3 + 1f / 6f * a4,
                Acceleration = currentState.Acceleration,
                Gravity = currentState.Gravity,
                Drag = currentState.Drag,
                Thrust = currentState.Thrust
            };
        }
    }
}
