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
            var d1 = Vector3d.Dot(currentState.Drag, currentState.Velocity.Normalized()) * timeStep;
            var g1 = Vector3d.Dot(currentState.Gravity, currentState.Velocity.Normalized()) * timeStep;

            var state2 = new State()
            {
                Time = currentState.Time + timeStep / 2.0,
                Position = currentState.Position + b1 / 2.0,
                Velocity = currentState.Velocity + a1 / 2.0
            };
            var a2 = calculateAcceleration(ref state2) * timeStep;
            var b2 = state2.Velocity * timeStep;
            var d2 = Vector3d.Dot(currentState.Drag, currentState.Velocity.Normalized()) * timeStep;
            var g2 = Vector3d.Dot(currentState.Gravity, currentState.Velocity.Normalized()) * timeStep;

            var state3 = new State()
            {
                Time = currentState.Time + timeStep / 2.0,
                Position = currentState.Position + b2 / 2.0,
                Velocity = currentState.Velocity + a2 / 2.0
            };
            var a3 = calculateAcceleration(ref state3) * timeStep;
            var b3 = state3.Velocity * timeStep;
            var d3 = Vector3d.Dot(currentState.Drag, currentState.Velocity.Normalized()) * timeStep;
            var g3 = Vector3d.Dot(currentState.Gravity, currentState.Velocity.Normalized()) * timeStep;

            var state4 = new State()
            {
                Time = currentState.Time + timeStep,
                Position = currentState.Position + b3,
                Velocity = currentState.Velocity + a3
            };
            var a4 = calculateAcceleration(ref state3) * timeStep;
            var b4 = state3.Velocity * timeStep;
            var d4 = Vector3d.Dot(currentState.Drag, currentState.Velocity.Normalized()) * timeStep;
            var g4 = Vector3d.Dot(currentState.Gravity, currentState.Velocity.Normalized()) * timeStep;

            newState = new State()
            {
                Time = currentState.Time + timeStep,
                Position = currentState.Position + 1.0 / 6.0 * b1 + 1.0 / 3.0 * b2 + 1.0 / 3.0 * b3 + 1.0 / 6.0 * b4,
                Velocity = currentState.Velocity + 1.0 / 6.0 * a1 + 1.0 / 3.0 * a2 + 1.0 / 3.0 * a3 + 1.0 / 6.0 * a4,
                LossesToDrag = currentState.LossesToDrag + 1.0 / 6.0 * d1 + 1.0 / 3.0 * d2 + 1.0 / 3.0 * d3 + 1.0 / 6.0 * d4,
                LossesToGravity = currentState.LossesToGravity + 1.0 / 6.0 * g1 + 1.0 / 3.0 * g2 + 1.0 / 3.0 * g3 + 1.0 / 6.0 * g4,
                Acceleration = currentState.Acceleration,
                Gravity = currentState.Gravity,
                Drag = currentState.Drag,
                Thrust = currentState.Thrust
            };
        }
    }
}
