﻿using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Numerics;

namespace RocketOptimize.Simulation.Integrators.Tests
{
    [TestClass()]
    public class RK4Tests
    {
        [TestMethod()]
        public void IntegrateTest()
        {
            const float Gravity = -9.81f;
            const float theta = 0.25f;
            const float v0 = 500f;

            var currentState = new State()
            {
                Position = new Vector3()
                {
                    X = 0,
                    Y = 0,
                    Z = 0
                },
                Velocity = new Vector3()
                {
                    X = v0 * (float)Math.Cos(theta),
                    Y = v0 * (float)Math.Sin(theta),
                    Z = 0
                }
            };

            const float timeStep = 0.01f;


            const float allowedDeviationPosition = 0.8f;
            const float allowedDeviationVelocity = 0.02f;

            Vector3 deltaPosition = Vector3.Zero;
            Vector3 deltaVelocity = Vector3.Zero;

            var rk4 = new RK4();
            while (currentState.Position.Y >= -0.1f)
            {
                rk4.Integrate(timeStep, ref currentState, out currentState, (State state) =>
                {
                    return Vector3.UnitY * Gravity;
                });

                Vector3 analyticalPosition = new Vector3()
                {
                    X = (float)(v0 * Math.Cos(theta) * currentState.Time),
                    Y = (float)(v0 * Math.Sin(theta) * currentState.Time - 0.5 * 9.81 * currentState.Time * currentState.Time),
                    Z = 0f
                };

                Vector3 analyticalVelocity = new Vector3()
                {
                    X = (float)(v0 * Math.Cos(theta)),
                    Y = (float)(v0 * Math.Sin(theta) - 9.81 * currentState.Time),
                    Z = 0f
                };

                deltaPosition = analyticalPosition - currentState.Position;
                deltaVelocity = analyticalVelocity - currentState.Velocity;

                if (deltaPosition.Length() > allowedDeviationPosition)
                {
                    Assert.Fail(string.Format("Position has gone out of error range at time {1,2:F}: {0,8:F}", deltaPosition.Length(), currentState.Time));
                }
                if (deltaVelocity.Length() > allowedDeviationVelocity)
                {
                    Assert.Fail(string.Format("Velocity has gone out of error range at time {1,2:F}: {0,8:F}", deltaVelocity.Length(), currentState.Time));
                }
            }
            Console.WriteLine(string.Format("Final deviation: position: {0,8:F}, velocity: {1,8:F}", deltaPosition.Length(), deltaVelocity.Length()));
        }
    }
}