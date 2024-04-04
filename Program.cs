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
        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update1;
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
        }

        public class CraneArm
        {
            int ID;
            Program program;
            double segmentOneBaseLength;
            double segmentTwoBaseLength;
            double maxArmLength;
            double minArmLength;
            double currentArmLength;
            IMyShipController controller;
            List<IMyPistonBase> pistons = new List<IMyPistonBase>();
            IMyMotorAdvancedStator baseRotor;
            bool baseRotorInverted;
            IMyMotorAdvancedStator segmentOneRotor;
            bool segmentOneRotorInverted;
            IMyMotorAdvancedStator segmentTwoRotor;
            bool segmentTwoRotorInverted;
            IMyMotorAdvancedStator endEffectorHinge;

            double ZCoord = 0;
            double YCoord = 0;
            double XCoord = 0;

            double desiredExtension = 0;
            double currentExtension;
            double maxExtension;

            double baseRotorAngle;
            double segmentOneRotorAngle;
            double segmentTwoRotorAngle;

            double sensitivity;
            double speed;

            public CraneArm(Program program, int ID, double segmentOneBaseLength, double segmentTwoBaseLength, IMyShipController controller, double sensitivity, double speed)
            {
                this.ID = ID;
                this.segmentOneBaseLength = segmentOneBaseLength;
                this.segmentTwoBaseLength = segmentTwoBaseLength;
                this.controller = controller;

                program.GridTerminalSystem.GetBlockGroupWithName($"Crane Pistons {ID}").GetBlocksOfType(pistons);
                baseRotor = (IMyMotorAdvancedStator)program.GridTerminalSystem.GetBlockWithName($"Crane Base Rotor {ID}");
                segmentOneRotor = (IMyMotorAdvancedStator)program.GridTerminalSystem.GetBlockWithName($"Crane Segment One Rotor {ID}");
                segmentTwoRotor = (IMyMotorAdvancedStator)program.GridTerminalSystem.GetBlockWithName($"Crane Segment Two Rotor {ID}");
                endEffectorHinge = (IMyMotorAdvancedStator)program.GridTerminalSystem.GetBlockWithName($"End Effector Hinge {ID}");

                this.sensitivity = sensitivity;
                this.speed = speed;

                foreach (IMyPistonBase piston in pistons)
                {
                    currentExtension += piston.CurrentPosition;
                    maxExtension += piston.HighestPosition;
                }

                desiredExtension = currentExtension;

                double segmentOneLength = segmentOneBaseLength;
                double segmentTwoLength = segmentTwoBaseLength + currentExtension;

                baseRotorInverted = baseRotor.CustomData.Contains("Inverted");
                segmentOneRotorInverted = segmentOneRotor.CustomData.Contains("Inverted");
                segmentTwoRotorInverted = segmentTwoRotor.CustomData.Contains("Inverted");

                baseRotorAngle = baseRotorInverted ? 2 * Math.PI - baseRotor.Angle : baseRotor.Angle;
                segmentOneRotorAngle = segmentOneRotorInverted ? 2 * Math.PI - segmentOneRotor.Angle : segmentOneRotor.Angle;
                segmentTwoRotorAngle = segmentTwoRotorInverted ? 2 * Math.PI - segmentTwoRotor.Angle : segmentTwoRotor.Angle;

                XCoord = -segmentOneLength * Math.Sin(baseRotorAngle) * Math.Cos(segmentOneRotorAngle) - segmentTwoLength * Math.Cos(segmentOneRotorAngle + segmentTwoRotorAngle) * Math.Sin(baseRotorAngle);
                YCoord = segmentTwoLength * Math.Sin(segmentOneRotorAngle + segmentTwoRotorAngle) + segmentOneLength * Math.Sin(segmentOneRotorAngle);
                ZCoord = -segmentOneLength * Math.Cos(baseRotorAngle) * Math.Cos(segmentOneRotorAngle) - segmentTwoLength * Math.Cos(segmentOneRotorAngle + segmentTwoRotorAngle) * Math.Cos(baseRotorAngle);

                currentArmLength = Math.Sqrt(ZCoord * ZCoord + YCoord * YCoord + XCoord * XCoord);
            }

            public void Run()
            {
                currentExtension = 0;

                foreach (IMyPistonBase piston in pistons)
                {
                    currentExtension += piston.CurrentPosition;
                }

                double segmentOneLength = segmentOneBaseLength;
                double segmentTwoLength = segmentTwoBaseLength + currentExtension;

                maxArmLength = segmentOneLength + segmentTwoLength;
                minArmLength = Math.Abs(segmentTwoLength - segmentOneLength) + 1;

                if (controller.MoveIndicator.Z != 0)
                {
                    ZCoord += sensitivity * controller.MoveIndicator.Z;
                    currentArmLength = Math.Sqrt(ZCoord * ZCoord + YCoord * YCoord + XCoord * XCoord);
                    if (currentArmLength > maxArmLength)
                    {
                        currentArmLength = maxArmLength;
                        ZCoord = Math.Sqrt(maxArmLength * maxArmLength - YCoord * YCoord - XCoord * XCoord) * Math.Sign(ZCoord);
                    }
                    else if (currentArmLength < minArmLength)
                    {
                        currentArmLength = minArmLength;
                        ZCoord = Math.Sqrt(minArmLength * minArmLength - YCoord * YCoord - XCoord * XCoord) * Math.Sign(ZCoord);
                    }
                }
                
                if (controller.MoveIndicator.Y != 0)
                {
                    YCoord += sensitivity * controller.MoveIndicator.Y;
                    currentArmLength = Math.Sqrt(ZCoord * ZCoord + YCoord * YCoord + XCoord * XCoord);
                    if (currentArmLength > maxArmLength)
                    {
                        currentArmLength = maxArmLength;
                        YCoord = Math.Sqrt(maxArmLength * maxArmLength - ZCoord * ZCoord - XCoord * XCoord) * Math.Sign(YCoord);
                    }
                    else if (currentArmLength < minArmLength)
                    {
                        currentArmLength = minArmLength;
                        YCoord = Math.Sqrt(minArmLength * minArmLength - ZCoord * ZCoord - XCoord * XCoord) * Math.Sign(YCoord);
                    }
                }

                if (controller.MoveIndicator.X != 0)
                {
                    XCoord += sensitivity * controller.MoveIndicator.X;
                    currentArmLength = Math.Sqrt(ZCoord * ZCoord + YCoord * YCoord + XCoord * XCoord);
                    if (currentArmLength > maxArmLength)
                    {
                        currentArmLength = maxArmLength;
                        XCoord = Math.Sqrt(maxArmLength * maxArmLength - YCoord * YCoord - ZCoord * ZCoord) * Math.Sign(XCoord);
                    }
                    else if (currentArmLength < minArmLength)
                    {
                        currentArmLength = minArmLength;
                        XCoord = Math.Sqrt(minArmLength * minArmLength - YCoord * YCoord - ZCoord * ZCoord) * Math.Sign(XCoord);
                    }
                }

                if (controller.RollIndicator != 0)
                {
                    desiredExtension += 0.25 * sensitivity * controller.RollIndicator;
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

                currentArmLength = Math.Sqrt(ZCoord * ZCoord + YCoord * YCoord + XCoord * XCoord);

                if (currentArmLength > maxArmLength || currentArmLength < minArmLength)
                {
                    XCoord = -segmentOneLength * Math.Sin(baseRotorAngle) * Math.Cos(segmentOneRotorAngle) - segmentTwoLength * Math.Cos(segmentOneRotorAngle + segmentTwoRotorAngle) * Math.Sin(baseRotorAngle);
                    YCoord = segmentTwoLength * Math.Sin(segmentOneRotorAngle + segmentTwoRotorAngle) + segmentOneLength * Math.Sin(segmentOneRotorAngle);
                    ZCoord = -segmentOneLength * Math.Cos(baseRotorAngle) * Math.Cos(segmentOneRotorAngle) - segmentTwoLength * Math.Cos(segmentOneRotorAngle + segmentTwoRotorAngle) * Math.Cos(baseRotorAngle);

                    currentArmLength = Math.Sqrt(ZCoord * ZCoord + YCoord * YCoord + XCoord * XCoord);

                }

                if (controller.RotationIndicator.X != 0)
                {
                    endEffectorHinge.TargetVelocityRad = (float)(0.1 * speed * controller.RotationIndicator.X);
                }
                else if (controller.RotationIndicator.X == 0)
                {
                    endEffectorHinge.TargetVelocityRad = 0;
                }

                baseRotorAngle = baseRotorInverted ? 2 * Math.PI - baseRotor.Angle : baseRotor.Angle;
                segmentOneRotorAngle = segmentOneRotorInverted ? 2 * Math.PI - segmentOneRotor.Angle : segmentOneRotor.Angle;
                segmentTwoRotorAngle = segmentTwoRotorInverted ? 2 * Math.PI - segmentTwoRotor.Angle : segmentTwoRotor.Angle;

                double baseRotorTargetAngle = Math.PI + Math.Atan2(XCoord, ZCoord);

                double segmentOneRotorTargetAngle = Math.Acos((currentArmLength * currentArmLength + segmentOneLength * segmentOneLength - segmentTwoLength * segmentTwoLength) / (2 * currentArmLength * segmentOneLength)) + Math.Atan2(YCoord, Math.Sqrt(XCoord * XCoord + ZCoord * ZCoord));
                double segmentTwoRotorTargetAngle = Math.PI + Math.Acos((segmentTwoLength * segmentTwoLength + segmentOneLength * segmentOneLength - currentArmLength * currentArmLength) / (2 * segmentTwoLength * segmentOneLength));

                double baseRotorAngleError = baseRotorTargetAngle - baseRotorAngle;
                baseRotorAngleError = Math.Abs(baseRotorAngleError) > Math.PI ? ( 2 * Math.PI - Math.Abs(baseRotorAngleError)) * -Math.Sign(baseRotorAngleError) : baseRotorAngleError;
                double segmentOneRotorAngleError = segmentOneRotorTargetAngle - segmentOneRotorAngle;
                segmentOneRotorAngleError = Math.Abs(segmentOneRotorAngleError) > Math.PI ? (2 * Math.PI - Math.Abs(segmentOneRotorAngleError)) * -Math.Sign(segmentOneRotorAngleError) : segmentOneRotorAngleError;

                double segmentTwoRotorAngleError = segmentTwoRotorTargetAngle - segmentTwoRotorAngle;
                segmentTwoRotorAngleError = Math.Abs(segmentTwoRotorAngleError) > Math.PI ? (2 * Math.PI - Math.Abs(segmentTwoRotorAngleError)) * -Math.Sign(segmentTwoRotorAngleError) : segmentTwoRotorAngleError;

                if (!double.IsNaN(segmentOneRotorAngleError) && !double.IsNaN(segmentTwoRotorAngleError))
                {
                    baseRotor.TargetVelocityRad = baseRotorInverted ? -(float)(speed * baseRotorAngleError) : (float)(speed * baseRotorAngleError);

                    segmentOneRotor.TargetVelocityRad = segmentOneRotorInverted ? -(float)(speed * segmentOneRotorAngleError) : (float)(speed * segmentOneRotorAngleError);

                    segmentTwoRotor.TargetVelocityRad = segmentTwoRotorInverted ? -(float)(speed * segmentTwoRotorAngleError) : (float)(speed * segmentTwoRotorAngleError);

                }
                else
                {
                    baseRotor.TargetVelocityRad = 0;

                    segmentOneRotor.TargetVelocityRad = 0;

                    segmentTwoRotor.TargetVelocityRad = 0;
                }
            }
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
