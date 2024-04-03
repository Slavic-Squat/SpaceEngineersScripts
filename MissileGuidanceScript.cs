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
        MissileControl missile;
        Dictionary<string, Action<string>> commands = new Dictionary<string, Action<string>>();
        MyCommandLine commandLine = new MyCommandLine();

        public Program()
        {
            missile = new MissileControl(this, 1, true);

            commands["Launch"] = missile.Launch;
        }

        public void Save()
        {

        }

        public void Main(string argument, UpdateType updateSource)
        {
            missile.Run();

            if (commandLine.TryParse(argument))
            {
                string commandName = commandLine.Argument(0);
                string commandArgument = commandLine.Argument(1);
                Action<string> command;

                if (commandName != null && commandArgument != null)
                {
                    if (commands.TryGetValue(commandName, out command))
                    {
                        command(commandArgument);
                    }
                }
            }
        }

        public class PIDControl
        {
            public float Kp { get; }
            public float Ki { get; }
            public float Kd { get; }

            private float integralValue = 0;
            private float priorValue = 0;

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

                integralValue += (input * timeDelta);

                float differencial = (input - priorValue) / timeDelta;
                priorValue = input;
                float result = this.Kp * input + this.Ki * integralValue + this.Kd * differencial;

                return result;
            }

        }

        public class MovingAverage
        {
            int windowSize;
            Queue<Vector3D> values;
            Vector3D sum;
            public Vector3D average;

            public MovingAverage(int windowSize)
            {
                this.windowSize = windowSize;
                values = new Queue<Vector3D>();
                sum = Vector3D.Zero;
            }

            public void Update(Vector3D input)
            {
                values.Enqueue(input);
                sum += input;
                if (values.Count > windowSize)
                {
                    sum -= values.Dequeue();
                }
                if (values.Count == windowSize)
                {
                    average = sum / windowSize;

                    return;
                }
                else if (values.Count < windowSize)
                {
                    average = input;

                    return;
                }
            }
        }

        public class MissileGuidance
        {
            int maxSpeed = 500;
            double N;
            double maxForwardAccel;
            double maxRadialAccel;
            MovingAverage signalSmoother;

            public Vector3D vectorToAlign;
            public Vector3D totalAcceleration;

            public MissileGuidance(double maxForwardAccel, double maxRadialAccel, double N, int smoothing)
            {
                this.N = N;
                this.maxForwardAccel = maxForwardAccel;
                this.maxRadialAccel = maxRadialAccel;
                signalSmoother = new MovingAverage(smoothing);
            }

            public void Run(Vector3D missileVelocity, Vector3D missilePosition, Vector3D targetVelocity, Vector3D targetPosition)
            {
                double maxTotalAccel = Math.Sqrt((maxRadialAccel * maxRadialAccel) + (maxForwardAccel * maxForwardAccel));
                double maxAccelAngle = Math.Atan(maxForwardAccel / maxRadialAccel);

                Vector3D missileVelocityHeading = Vector3D.Normalize(missileVelocity);
                Vector3D rangeToTarget = targetPosition - missilePosition;
                Vector3D directionToTarget = Vector3D.Normalize(rangeToTarget);

                if (rangeToTarget.Length() > 5000)
                {
                    rangeToTarget = directionToTarget * 5000;
                }
                Vector3D relativeTargetVelocity = targetVelocity - missileVelocity;
                Vector3D rotationVector = Vector3D.Cross(rangeToTarget, relativeTargetVelocity) / Vector3D.Dot(rangeToTarget, rangeToTarget);

                Vector3D proNavAcceleration = -N * (relativeTargetVelocity.Length() / missileVelocity.Length()) * Vector3D.Cross(missileVelocity, rotationVector);
                signalSmoother.Update(proNavAcceleration);
                proNavAcceleration = signalSmoother.average;
                Vector3D proNavAccelerationDirection = Vector3D.Normalize(proNavAcceleration);
                double proNavAccelerationMagnitude = proNavAcceleration.Length();

                Vector3D accelerationAlongVelocity;
                if (proNavAccelerationMagnitude < maxRadialAccel)
                {
                    double accelerationAlongVelocityMagnitude = maxForwardAccel;
                    accelerationAlongVelocity = missileVelocityHeading * accelerationAlongVelocityMagnitude;

                    vectorToAlign = missileVelocityHeading;
                }
                else
                {
                    if (proNavAccelerationMagnitude > maxTotalAccel)
                    {
                        proNavAccelerationMagnitude = maxTotalAccel;
                        proNavAcceleration = proNavAccelerationDirection * proNavAccelerationMagnitude;
                    }
                    double totalAccelerationAngle = Math.Acos(proNavAccelerationMagnitude / maxTotalAccel);
                    double accelerationAlongVelocityMagnitude = Math.Sin(totalAccelerationAngle) * maxTotalAccel;
                    accelerationAlongVelocity = missileVelocityHeading * accelerationAlongVelocityMagnitude;

                    double rotation = maxAccelAngle - totalAccelerationAngle;
                    Vector3D axisOfRotation = Vector3D.Cross(proNavAccelerationDirection, missileVelocityHeading);
                    Quaternion alignmentCorrection = Quaternion.CreateFromAxisAngle(axisOfRotation, -(float)rotation);

                    vectorToAlign = Vector3D.Transform(missileVelocityHeading, alignmentCorrection);
                }

                if (missileVelocity.Length() > (maxSpeed - 1))
                {
                    accelerationAlongVelocity = Vector3D.Zero;
                }

                totalAcceleration = accelerationAlongVelocity + proNavAcceleration;
            }
        }

        public class MissileControl
        {
            Program program;
            int number;
            bool clusterMissile;

            MissileGuidance missileGuidance;

            enum Direction
            {
                Forward, Backward, Leftward, Rightward, Upward, Downward
            }

            public enum Stage
            {
                Idle, Launching, Flying, Interception
            }

            IMyBroadcastListener myBroadcastListener;

            List<IMyGyro> gyros = new List<IMyGyro>();
            IMyRemoteControl remoteControl;
            IMyShipMergeBlock mergeBlock;
            List<IMyWarhead> payload = new List<IMyWarhead>();
            List<IMyMotorStator> attachmentRotors = new List<IMyMotorStator>();

            List<IMyThrust> thrusters = new List<IMyThrust>();
            List<Vector3D> localThrusterDirections = new List<Vector3D>();
            double maxForwardThrust;
            double maxBackwardThrust;
            double maxLeftwardThrust;
            double maxRightwardThrust;
            double maxUpwardThrust;
            double maxDownwardThrust;

            double missileMass;

            double maxForwardAccel;
            double maxRadialAccel;

            Vector3D missilePosition;
            Vector3D missileVelocity;

            Vector3D targetPosition;
            Vector3D targetVelocity;
            Vector3D launcherPosition;

            Vector3D relativeTargetPosition;
            Vector3D closingVelocity;
            double distanceToTarget;
            double closingSpeed;
            double timeToTarget;

            public Stage stage = Stage.Idle;

            Dictionary<IMyThrust, MyTuple<Vector3D, Direction>> thrusterInfo = new Dictionary<IMyThrust, MyTuple<Vector3D, Direction>>();

            Vector3D localTotalAcceleration;
            Vector3D localVectorToAlign;

            PIDControl pitchController;
            PIDControl yawController;



            public MissileControl(Program program, int number, bool clusterMissile)
            {
                this.program = program;
                this.number = number;
                this.clusterMissile = clusterMissile;

                program.GridTerminalSystem.GetBlockGroupWithName("Missile Thrusters " + number).GetBlocksOfType<IMyThrust>(thrusters);
                program.GridTerminalSystem.GetBlockGroupWithName("Missile Gyros " + number).GetBlocksOfType<IMyGyro>(gyros);
                remoteControl = (IMyRemoteControl)program.GridTerminalSystem.GetBlockWithName("Missile Controller " + number);
                mergeBlock = (IMyShipMergeBlock)program.GridTerminalSystem.GetBlockWithName("Missile Merge Block " + number);

                if (clusterMissile == true)
                {
                    program.GridTerminalSystem.GetBlockGroupWithName("Payload " + number).GetBlocksOfType(payload);
                    program.GridTerminalSystem.GetBlockGroupWithName("Attachment Rotors " + number).GetBlocksOfType(attachmentRotors);
                }

                foreach (IMyThrust thruster in thrusters)
                {
                    Vector3D localThrusterDirection = Vector3D.Round(Vector3D.TransformNormal(thruster.WorldMatrix.Backward, MatrixD.Transpose(remoteControl.WorldMatrix)), 1);
                    localThrusterDirections.Add(localThrusterDirection);


                    if (localThrusterDirection.Z == -1)
                    {
                        maxForwardThrust += thruster.MaxThrust;
                        thrusterInfo.Add(thruster, new MyTuple<Vector3D, Direction>(localThrusterDirection, Direction.Forward));
                    }
                    else if (localThrusterDirection.Z == 1)
                    {
                        maxBackwardThrust += thruster.MaxThrust;
                        thrusterInfo.Add(thruster, new MyTuple<Vector3D, Direction>(localThrusterDirection, Direction.Backward));
                    }
                    else if (localThrusterDirection.X == -1)
                    {
                        maxLeftwardThrust += thruster.MaxThrust;
                        thrusterInfo.Add(thruster, new MyTuple<Vector3D, Direction>(localThrusterDirection, Direction.Leftward));
                    }
                    else if (localThrusterDirection.X == 1)
                    {
                        maxRightwardThrust += thruster.MaxThrust;
                        thrusterInfo.Add(thruster, new MyTuple<Vector3D, Direction>(localThrusterDirection, Direction.Rightward));
                    }
                    else if (localThrusterDirection.Y == 1)
                    {
                        maxUpwardThrust += thruster.MaxThrust;
                        thrusterInfo.Add(thruster, new MyTuple<Vector3D, Direction>(localThrusterDirection, Direction.Forward));
                    }
                    else if (localThrusterDirection.Y == -1)
                    {
                        maxDownwardThrust += thruster.MaxThrust;
                        thrusterInfo.Add(thruster, new MyTuple<Vector3D, Direction>(localThrusterDirection, Direction.Downward));
                    }
                }

                missileMass = remoteControl.CalculateShipMass().TotalMass;
                maxForwardAccel = maxForwardThrust / missileMass;
                maxRadialAccel = maxRightwardThrust / missileMass;

                pitchController = new PIDControl(1.0f, 0, 0.2f);
                yawController = new PIDControl(1.0f, 0, 0.2f);

                missileGuidance = new MissileGuidance(maxForwardAccel, maxRadialAccel, 3.5, 10);
            }

            public void Run()
            {
                if (myBroadcastListener != null)
                {
                    float timeDelta = (float)program.Runtime.TimeSinceLastRun.TotalSeconds;

                    missilePosition = remoteControl.CubeGrid.GetPosition();
                    missileVelocity = remoteControl.GetShipVelocities().LinearVelocity;

                    if (myBroadcastListener.HasPendingMessage)
                    {
                        MyIGCMessage message = myBroadcastListener.AcceptMessage();
                        MyTuple<ImmutableList<Vector3D>, MatrixD> laserTargetInfo = message.As<MyTuple<ImmutableList<Vector3D>, MatrixD>>();

                        targetPosition = laserTargetInfo.Item1[0];
                        targetVelocity = laserTargetInfo.Item1[1];

                        launcherPosition = laserTargetInfo.Item2.Translation;
                    }
                    relativeTargetPosition = targetPosition - missilePosition;
                    distanceToTarget = relativeTargetPosition.Length();
                    closingVelocity = targetVelocity - missileVelocity;
                    closingSpeed = Vector3D.Dot(relativeTargetPosition.Normalized(), closingVelocity.Normalized()) * closingVelocity.Length();
                    timeToTarget = distanceToTarget / closingSpeed;

                    switch (stage)
                    {
                        case Stage.Idle:

                            localVectorToAlign = -Vector3D.UnitZ;
                            localTotalAcceleration = Vector3D.Zero;

                            break;

                        case Stage.Launching:

                            mergeBlock.Enabled = false;
                            localVectorToAlign = -Vector3D.UnitZ;
                            localTotalAcceleration = maxForwardAccel * localVectorToAlign;

                            if ((missilePosition - launcherPosition).Length() > 100)
                            {
                                stage = Stage.Flying;
                            }

                            break;

                        case Stage.Flying:

                            missileGuidance.Run(missileVelocity, missilePosition, targetVelocity, targetPosition);
                            localTotalAcceleration = Vector3D.TransformNormal(missileGuidance.totalAcceleration, MatrixD.Transpose(remoteControl.WorldMatrix));
                            localVectorToAlign = Vector3D.TransformNormal(missileGuidance.vectorToAlign, MatrixD.Transpose(remoteControl.WorldMatrix));

                            if (timeToTarget > 0 && timeToTarget < 7)
                            {
                                stage = Stage.Interception;
                            }

                            break;

                        case Stage.Interception:

                            missileGuidance.Run(missileVelocity, missilePosition, targetVelocity, targetPosition);
                            localTotalAcceleration = Vector3D.TransformNormal(missileGuidance.totalAcceleration, MatrixD.Transpose(remoteControl.WorldMatrix));
                            localVectorToAlign = Vector3D.TransformNormal(missileGuidance.vectorToAlign, MatrixD.Transpose(remoteControl.WorldMatrix));

                            if (clusterMissile == true)
                            {
                                foreach (IMyGyro gyro in gyros)
                                {
                                    gyro.Roll = (float)(2 * Math.PI);
                                }

                                if (timeToTarget > 0 && timeToTarget < 5)
                                {
                                    foreach (IMyWarhead warhead in payload)
                                    {
                                        warhead.IsArmed = true;
                                        warhead.DetonationTime = (float)(timeToTarget - 0.1);
                                        warhead.StartCountdown();
                                    }
                                    foreach (IMyMotorStator attachmentRotor in attachmentRotors)
                                    {
                                        attachmentRotor.Detach();
                                    }
                                }
                            }
                            break;
                    }

                    double pitchError = Math.Atan2(localVectorToAlign.Y, -localVectorToAlign.Z);
                    double yawError = Math.Atan2(localVectorToAlign.X, -localVectorToAlign.Z);

                    foreach (IMyGyro gyro in gyros)
                    {
                        gyro.Pitch = -(float)pitchController.Run((float)pitchError, timeDelta);
                        gyro.Yaw = (float)yawController.Run((float)yawError, timeDelta);
                    }


                    foreach (KeyValuePair<IMyThrust, MyTuple<Vector3D, Direction>> thruster in thrusterInfo)
                    {
                        double maxAccel = 0;
                        switch (thruster.Value.Item2)
                        {
                            case Direction.Forward:
                                maxAccel = maxForwardThrust / missileMass;
                                break;

                            case Direction.Backward:
                                maxAccel = maxBackwardThrust / missileMass;
                                break;

                            case Direction.Leftward:
                                maxAccel = maxLeftwardThrust / missileMass;
                                break;

                            case Direction.Rightward:
                                maxAccel = maxRightwardThrust / missileMass;
                                break;

                            case Direction.Upward:
                                maxAccel = maxUpwardThrust / missileMass;
                                break;

                            case Direction.Downward:
                                maxAccel = maxDownwardThrust / missileMass;
                                break;
                        }
                        thruster.Key.ThrustOverridePercentage = (float)(Vector3D.Dot(localTotalAcceleration, thruster.Value.Item1) / maxAccel);
                    }
                }
            }

            public void Launch(string broadcastTag)
            {
                myBroadcastListener = program.IGC.RegisterBroadcastListener(broadcastTag);

                stage = Stage.Launching;

                if (program.Runtime.UpdateFrequency != UpdateFrequency.Update1)
                {
                    program.Runtime.UpdateFrequency = UpdateFrequency.Update1;
                }
            }
        }
    }
}
