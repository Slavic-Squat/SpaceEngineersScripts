using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        MyCommandLine commandLine = new MyCommandLine();
        Dictionary<string, Action<string>> commandDictionary = new Dictionary<string, Action<string>>();
        MoleRover moleRover;
        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update1;

            moleRover = new MoleRover(this);
            commandDictionary["SetSpeedLimit"] = moleRover.SetSpeedLimit;
            commandDictionary["IncreaseSpeedLimit"] = moleRover.IncreaseSpeedLimit;
            commandDictionary["DecreaseSpeedLimit"] = moleRover.DecreaseSpeedLimit;
            commandDictionary["SetPropulsionPercentage"] = moleRover.SetPropulsionPercentage;
            commandDictionary["IncreasePropulsionPercentage"] = moleRover.IncreasePropulsionPercentage;
            commandDictionary["DecreasePropulsionPercentage"] = moleRover.DecreasePropulsionPercentage;
        }

        public void Save()
        {

        }

        public void Main(string argument, UpdateType updateSource)
        {
            if (commandLine.TryParse(argument))
            {
                string commandName = commandLine.Argument(0);
                string commandArgument = commandLine.Argument(1);
                Action<string> command;

                if (commandName != null && commandArgument != null)
                {
                    if (commandDictionary.TryGetValue(commandName, out command))
                    {
                        command(commandArgument);
                    }
                }
            }

            moleRover.Run();
        }

        public class MoleRover
        {
            Program program;

            IMyShipController controller;
            IMyTextSurfaceProvider controllerScreens;

            DrillArm drillArmLeft;
            DrillArm drillArmRight;
            DriveWheels wheels;

            public MoleRover(Program program)
            {
                this.program = program;
                controller = (IMyShipController)program.GridTerminalSystem.GetBlockWithName("Mole Controller");
                controllerScreens = (IMyTextSurfaceProvider)controller;

                //drillArmLeft = new DrillArm(1, program, true, controller, 0.05, 1);
                //drillArmRight = new DrillArm(2, program, false, controller, 0.05, 1);
                wheels = new DriveWheels(1, program, controller);
            }

            public void Run()
            {
                //drillArmLeft.Run();
                //drillArmRight.Run();
                wheels.Run();
            }

            public void SetSpeedLimit(string speedLimitString)
            {
                float speedLimit;
                float.TryParse(speedLimitString, out speedLimit);
                wheels.SetSpeedLimit(speedLimit);
            }

            public void IncreaseSpeedLimit(string adjustmentString)
            {
                float adjustment;
                float.TryParse(adjustmentString, out adjustment);
                wheels.IncreaseSpeedLimit(adjustment);
            }

            public void DecreaseSpeedLimit(string adjustmentString)
            {
                float adjustment;
                float.TryParse(adjustmentString, out adjustment);
                wheels.DecreaseSpeedLimit(adjustment);
            }

            public void SetPropulsionPercentage(string propulsionPercentageString)
            {
                float propulsionPercentage;
                float.TryParse(propulsionPercentageString, out propulsionPercentage);
                wheels.SetPropulsionPercentage(propulsionPercentage);
            }

            public void IncreasePropulsionPercentage(string adjustmentString)
            {
                float adjustment;
                float.TryParse(adjustmentString, out adjustment);
                wheels.IncreasePropulsionPercentage(adjustment);
            }

            public void DecreasePropulsionPercentage(string adjustmentString)
            {
                float adjustment;
                float.TryParse(adjustmentString, out adjustment);
                wheels.DecreasePropulsionPercentage(adjustment);
            }
        }
        public class DriveWheels
        {
            int ID;
            Program program;

            IMyShipController controller;

            List<IMyMotorSuspension> wheels = new List<IMyMotorSuspension>();

            double propulsionPercentage = 0;

            double speedLimit = 0;

            double forwardVelocity;
            double maxTurningVelocity = 50;
            double velocityTurningRatio;

            public DriveWheels(int ID, Program program, IMyShipController controller)
            {
                this.ID = ID;
                this.program = program;
                this.controller = controller;

                program.GridTerminalSystem.GetBlockGroupWithName($"Drive Wheels {1}").GetBlocksOfType(wheels);
            }

            public void Run()
            {

                forwardVelocity = Vector3D.TransformNormal(controller.GetShipVelocities().LinearVelocity, MatrixD.Transpose(controller.WorldMatrix)).Z;

                velocityTurningRatio = Math.Min(Math.Abs(forwardVelocity) / maxTurningVelocity, 1);

                foreach (IMyMotorSuspension wheel in wheels)
                {
                    wheel.SteeringOverride = (float)(controller.MoveIndicator.X * (1 - velocityTurningRatio));
                    wheel.PropulsionOverride = (float)(controller.MoveIndicator.Z * propulsionPercentage);

                }
            }

            public void SetSpeedLimit(float speedLimit)
            {
                foreach (IMyMotorSuspension wheel in wheels)
                {
                    wheel.SetValue("Speed Limit", speedLimit);
                }
            }

            public void IncreaseSpeedLimit(float adjustment)
            {
                speedLimit += adjustment;

                SetSpeedLimit((float)speedLimit);
            }

            public void DecreaseSpeedLimit(float adjustment)
            {
                speedLimit -= adjustment;

                SetSpeedLimit((float)speedLimit);
            }

            public void SetPropulsionPercentage(float propulsionPercentage)
            {
                this.propulsionPercentage = propulsionPercentage;
            }

            public void IncreasePropulsionPercentage(float adjustmnet)
            {
                this.propulsionPercentage += adjustmnet;
            }

            public void DecreasePropulsionPercentage(float adjustmnet)
            {
                this.propulsionPercentage -= adjustmnet;
            }
        }
        public class DrillArm
        {
            int ID;
            Program program;
            bool invertedRoll;
            IMyShipController controller;
            IMyMotorAdvancedStator baseRotor;
            bool baseRotorInverted;
            List<IMyPistonBase> pistons = new List<IMyPistonBase>();

            double desiredExtension = 0;
            double currentExtension;
            double maxExtension;

            double baseRotorTargetAngle;
            double baseRotorAngle;

            double sensitivity;
            double speed;

            public DrillArm(int ID, Program program, bool invertedRoll, IMyShipController controller, double sensitivity, double speed)
            {
                this.ID = ID;
                this.program = program;
                this.invertedRoll = invertedRoll;
                this.controller = controller;
                this.sensitivity = sensitivity;
                this.speed = speed;

                program.GridTerminalSystem.GetBlockGroupWithName($"Drill Arm Pistons {ID}").GetBlocksOfType(pistons);
                baseRotor = (IMyMotorAdvancedStator)program.GridTerminalSystem.GetBlockWithName($"Drill Arm Base Rotor {ID}");

                foreach (IMyPistonBase piston in pistons)
                {
                    currentExtension += piston.CurrentPosition;
                    maxExtension += piston.HighestPosition;
                }

                desiredExtension = currentExtension;

                baseRotorInverted = baseRotor.CustomData.Contains("Inverted");

                baseRotorAngle = baseRotorInverted ? 2 * Math.PI - baseRotor.Angle : baseRotor.Angle;
            }

            public void Run()
            {
                currentExtension = 0;

                foreach (IMyPistonBase piston in pistons)
                {
                    currentExtension += piston.CurrentPosition;
                }

                if (controller.MoveIndicator.Y != 0)
                {
                    desiredExtension += 0.25 * sensitivity * controller.MoveIndicator.Y;
                    desiredExtension = desiredExtension > maxExtension ? maxExtension : desiredExtension;
                    desiredExtension = desiredExtension < 0 ? 0 : desiredExtension;
                }

                double extensionError = desiredExtension - currentExtension;

                foreach (IMyPistonBase piston in pistons)
                {
                    if (Math.Abs(extensionError) > (0.01 * maxExtension))
                    {
                        if (desiredExtension < currentExtension)
                        {
                            piston.Velocity = (float)Math.Abs(1.0 * speed * extensionError);
                            if (piston.Status != PistonStatus.Retracting)
                            {
                                piston.Retract();
                            }
                        }
                        else if (desiredExtension > currentExtension)
                        {
                            piston.Velocity = (float)Math.Abs(1.0 * speed * extensionError);
                            if (piston.Status != PistonStatus.Extending)
                            {
                                piston.Extend();
                            }
                        }
                    }
                    else
                    {
                        piston.Velocity = 0;
                    }
                }

                if (controller.RotationIndicator.Y != 0)
                {
                    baseRotorTargetAngle += 0.1 * sensitivity * controller.RotationIndicator.Y;
                }

                if (controller.RollIndicator != 0)
                {
                    baseRotorTargetAngle += invertedRoll ? -0.1 * sensitivity * controller.RollIndicator : 0.1 * sensitivity * controller.RollIndicator;
                }

                baseRotorAngle = baseRotorInverted ? 2 * Math.PI - baseRotor.Angle : baseRotor.Angle;

                double baseRotorAngleError = baseRotorTargetAngle - baseRotorAngle;
                baseRotorAngleError = Math.Abs(baseRotorAngleError) > Math.PI ? (2 * Math.PI - Math.Abs(baseRotorAngleError)) * -Math.Sign(baseRotorAngleError) : baseRotorAngleError;

                baseRotor.TargetVelocityRad = baseRotorInverted ? -(float)(speed * baseRotorAngleError) : (float)(speed * baseRotorAngleError);
            }
        }

        public class DrillHead
        {

        }

        public class PIDControl
        {
            public float Kp { get; }
            public float Ki { get; }
            public float Kd { get; }

            public float integralValue = 0;
            public float priorValue = 0;

            public PIDControl(float Kp, float Ki, float Kd)
            {
                this.Kp = Kp;
                this.Ki = Ki;
                this.Kd = Kd;
            }

            public float Run(float input, float timeDelta)
            {
                if (timeDelta == 0 || input == 0)
                {
                    return input;
                }

                float differencial = (input - priorValue) / timeDelta;
                priorValue = input;
                float result = Kp * input + Ki * integralValue + Kd * differencial;

                return result;
            }

            public virtual void GetIntegral(float input, float timeDelta)
            {
                integralValue += (input * timeDelta);
            }

        }

        public class ClampedIntegralPIDControl : PIDControl
        {
            float lowerLimit;
            float upperLimit;
            public ClampedIntegralPIDControl(float Kp, float Ki, float Kd, float lowerLimit, float upperLimit) : base(Kp, Ki, Kd)
            {
                this.lowerLimit = lowerLimit;
                this.upperLimit = upperLimit;
            }

            public override void GetIntegral(float input, float timeDelta)
            {
                base.GetIntegral(input, timeDelta);
                integralValue = Math.Max(lowerLimit, Math.Min(upperLimit, integralValue));
            }
        }
    }
}
