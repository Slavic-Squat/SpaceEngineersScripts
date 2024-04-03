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

        MissileLauncher missileLauncher;
        Dictionary<string, Action<int>> commands = new Dictionary<string, Action<int>>();
        MyCommandLine commandLine = new MyCommandLine();

        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update1;

            missileLauncher = new MissileLauncher(this, 1, 1);

            commands["Launch"] = missileLauncher.LaunchNextAvailableMissile;
        }

        public void Save()
        {

        }

        public void Main(string argument, UpdateType updateSource)
        {
            missileLauncher.Run();
            if (commandLine.TryParse(argument))
            {
                string commandName = commandLine.Argument(0);
                string commandArgument = commandLine.Argument(1);
                Action<int> command;

                if (commandName != null && commandArgument != null)
                {
                    if (commands.TryGetValue(commandName, out command))
                    {
                        command(int.Parse(commandArgument));
                    }
                }
            }
        }

        public class TargetingLaser
        {
            Program program;

            string broadcastTag;

            IMyMotorStator azimuthRotor;
            IMyMotorStator elevationRotor;
            IMyShipController laserController;
            List<IMyCameraBlock> cameraArray = new List<IMyCameraBlock>();
            List<Vector3D> cameraPositionsFromRotationLocal = new List<Vector3D>();

            Vector3D rotationPointLocal;
            public MatrixD referenceMatrix;

            int raycastCounter;
            double maxRaycastDistance;
            double raycastDistanceGrowthSpeed;

            double timeSinceLastRaycast;
            double timeSinceLastDetection;
            double timeSinceLastUniqueDetection;
            double timeSinceLastTargetDetection;
            int raycastsSinceLastTargetDetection;

            int matchingDetectionCounter;

            MyDetectedEntityInfo detectedTarget;
            MyDetectedEntityInfo previouslyDetectedTarget;
            MyDetectedEntityInfo lockedTarget;

            Vector3D outputTargetPosition;
            Vector3D outputTargetVelocity;

            MatrixD launcherInfo;

            MyDetectedEntityInfo emptyTarget = new MyDetectedEntityInfo();

            Vector3D aimingVectorLocal;

            double azimuthError;
            double elevationError;
            PIDControl azimuthPID;
            PIDControl elevationPID;

            int elevationRotorSignCorrection;

            bool manualOverride = false;

            public TargetingLaser(Program program, int number, string broadcastTag, double maxRaycastDistance = 5000, double raycastDistanceGrowthSpeed = 200)
            {
                this.program = program;
                this.broadcastTag = broadcastTag;
                this.maxRaycastDistance = maxRaycastDistance;
                this.raycastDistanceGrowthSpeed = raycastDistanceGrowthSpeed;

                azimuthRotor = (IMyMotorStator)program.GridTerminalSystem.GetBlockWithName("Azimuth Rotor " + number);
                elevationRotor = (IMyMotorStator)program.GridTerminalSystem.GetBlockWithName("Elevation Rotor " + number);
                laserController = (IMyShipController)program.GridTerminalSystem.GetBlockWithName("Laser Controller " + number);
                program.GridTerminalSystem.GetBlockGroupWithName("Camera Array " + number).GetBlocksOfType<IMyCameraBlock>(cameraArray);

                rotationPointLocal = new Vector3D(0, Vector3D.TransformNormal(elevationRotor.GetPosition() - azimuthRotor.GetPosition(), MatrixD.Transpose(azimuthRotor.WorldMatrix)).Y, 0);

                referenceMatrix = azimuthRotor.WorldMatrix;

                Quaternion azimuthRotation = Quaternion.CreateFromAxisAngle(referenceMatrix.Up, -azimuthRotor.Angle);
                Quaternion elevationRotation = Quaternion.CreateFromAxisAngle(referenceMatrix.Right, -elevationRotor.Angle);
                Quaternion totalRotation = Quaternion.Concatenate(elevationRotation, azimuthRotation);

                MatrixD.Transform(ref referenceMatrix, ref totalRotation, out referenceMatrix);

                referenceMatrix.Translation = Vector3D.Transform(rotationPointLocal, azimuthRotor.WorldMatrix);

                foreach (IMyCameraBlock camera in cameraArray)
                {
                    camera.EnableRaycast = true;
                    Vector3D positionFromRotationLocal = Vector3D.TransformNormal(camera.GetPosition() - referenceMatrix.Translation, MatrixD.Transpose(referenceMatrix));
                    cameraPositionsFromRotationLocal.Add(positionFromRotationLocal);
                }

                azimuthPID = new PIDControl(25, 2, 0.1f);
                elevationPID = new PIDControl(25, 2, 0.1f);

                elevationRotorSignCorrection = -(int)Math.Round(Vector3D.Dot(elevationRotor.WorldMatrix.Up, referenceMatrix.Right));
            }

            public void Run()
            {
                double timeDeltaSeconds = program.Runtime.TimeSinceLastRun.TotalSeconds;
                double timeDeltaMiliseconds = program.Runtime.TimeSinceLastRun.TotalMilliseconds;
                timeSinceLastRaycast += timeDeltaMiliseconds;
                timeSinceLastDetection += timeDeltaSeconds;
                timeSinceLastUniqueDetection += timeDeltaSeconds;

                if (!lockedTarget.IsEmpty())
                {
                    timeSinceLastTargetDetection += timeDeltaSeconds;
                }

                referenceMatrix = azimuthRotor.WorldMatrix;

                Quaternion azimuthRotation = Quaternion.CreateFromAxisAngle(referenceMatrix.Up, -azimuthRotor.Angle);
                Quaternion elevationRotation = Quaternion.CreateFromAxisAngle(referenceMatrix.Right, -elevationRotor.Angle);
                Quaternion totalRotation = Quaternion.Concatenate(elevationRotation, azimuthRotation);

                MatrixD.Transform(ref referenceMatrix, ref totalRotation, out referenceMatrix);

                referenceMatrix.Translation = Vector3D.Transform(rotationPointLocal, azimuthRotor.WorldMatrix);


                if ((laserController.MoveIndicator.Y == -1 || (timeSinceLastTargetDetection > 5 && raycastsSinceLastTargetDetection >= 5)) && !lockedTarget.IsEmpty())
                {
                    lockedTarget = emptyTarget;
                    timeSinceLastTargetDetection = 0;
                    matchingDetectionCounter = 0;
                    raycastsSinceLastTargetDetection = 0;
                }

                Vector3D raycastDirectionLocal;
                double raycastDistance;

                if (Math.Abs(azimuthError) < 5 * Math.PI / 180 && Math.Abs(elevationError) < 5 * Math.PI / 180 && !lockedTarget.IsEmpty() && manualOverride == false)
                {
                    Vector3D raycastVectorLocal = aimingVectorLocal - cameraPositionsFromRotationLocal[raycastCounter % cameraArray.Count];
                    raycastDirectionLocal = Vector3D.Normalize(raycastVectorLocal);
                    raycastDistance = raycastVectorLocal.Length();
                }
                else
                {
                    Vector3D raycastVectorLocal = -Vector3D.UnitZ * maxRaycastDistance - cameraPositionsFromRotationLocal[raycastCounter % cameraArray.Count];
                    raycastDirectionLocal = Vector3D.Normalize(raycastVectorLocal);
                    raycastDistance = raycastVectorLocal.Length();
                }

                double raycastTimeDelta = raycastDistance / (2 * cameraArray.Count);

                if (cameraArray[raycastCounter % cameraArray.Count].TimeUntilScan(raycastDistance) == 0 && timeSinceLastRaycast >= raycastTimeDelta && ((!lockedTarget.IsEmpty() && manualOverride == false) || laserController.MoveIndicator.Y == 1))
                {
                    MyDetectedEntityInfo raycastResult = cameraArray[raycastCounter % cameraArray.Count].Raycast(raycastDistance, raycastDirectionLocal);
                    timeSinceLastRaycast = 0;
                    raycastCounter += 1;

                    if (!lockedTarget.IsEmpty() && raycastResult.EntityId != lockedTarget.EntityId)
                    {
                        raycastsSinceLastTargetDetection += 1;
                    }
                    if (!raycastResult.IsEmpty())
                    {
                        detectedTarget = raycastResult;
                        timeSinceLastDetection = 0;

                        if (lockedTarget.IsEmpty())
                        {
                            if (detectedTarget.EntityId == previouslyDetectedTarget.EntityId)
                            {
                                matchingDetectionCounter += 1;
                            }
                            else
                            {
                                timeSinceLastUniqueDetection = 0;
                                matchingDetectionCounter = 0;
                            }

                            previouslyDetectedTarget = detectedTarget;

                            if (timeSinceLastUniqueDetection > 2 && matchingDetectionCounter >= 3)
                            {
                                lockedTarget = detectedTarget;
                            }
                        }

                        else if (detectedTarget.EntityId == lockedTarget.EntityId)
                        {
                            lockedTarget = detectedTarget;
                            timeSinceLastTargetDetection = 0;
                            raycastsSinceLastTargetDetection = 0;
                        }
                    }
                }


                if (!lockedTarget.IsEmpty())
                {
                    double raycastDistanceGrowth = raycastDistanceGrowthSpeed * timeSinceLastTargetDetection;
                    Vector3D estimatedTargetPositionWorld = lockedTarget.Position + (Vector3D)lockedTarget.Velocity * timeSinceLastTargetDetection;
                    Vector3D estimatedTargetPositionLocal = Vector3D.TransformNormal(estimatedTargetPositionWorld - referenceMatrix.Translation, MatrixD.Transpose(referenceMatrix));
                    Vector3D estimatedTargetDirectionLocal = Vector3D.Normalize(estimatedTargetPositionLocal);

                    outputTargetPosition = estimatedTargetPositionWorld;
                    outputTargetVelocity = lockedTarget.Velocity;

                    List<Vector3D> targetKineticInfo = new List<Vector3D>();
                    targetKineticInfo.Add(outputTargetPosition);
                    targetKineticInfo.Add(outputTargetVelocity);
                    ImmutableList<Vector3D> immutableTargetKinecticInfo = targetKineticInfo.ToImmutableList();

                    launcherInfo = laserController.CubeGrid.WorldMatrix;

                    MyTuple<ImmutableList<Vector3D>, MatrixD> targetInfo = new MyTuple<ImmutableList<Vector3D>, MatrixD>(immutableTargetKinecticInfo, launcherInfo);

                    program.IGC.SendBroadcastMessage(broadcastTag, targetInfo);

                    if (manualOverride == false)
                    {
                        aimingVectorLocal = estimatedTargetDirectionLocal * (estimatedTargetPositionLocal.Length() + raycastDistanceGrowth);
                        aimingVectorLocal = aimingVectorLocal.Length() > maxRaycastDistance ? Vector3D.Normalize(aimingVectorLocal) * maxRaycastDistance : aimingVectorLocal;

                        double localTargetRange = aimingVectorLocal.Length();
                        azimuthError = Math.Atan2(aimingVectorLocal.X, -aimingVectorLocal.Z);
                        elevationError = Math.Asin(aimingVectorLocal.Y / localTargetRange);

                        azimuthRotor.TargetVelocityRad = azimuthPID.Run((float)azimuthError, (float)timeDeltaSeconds);
                        elevationRotor.TargetVelocityRad = elevationRotorSignCorrection * elevationPID.Run((float)elevationError, (float)timeDeltaSeconds);
                    }
                    else if (manualOverride == true)
                    {
                        aimingVectorLocal = -Vector3D.UnitZ * estimatedTargetPositionLocal.Length() + raycastDistanceGrowth;
                        aimingVectorLocal = aimingVectorLocal.Length() > maxRaycastDistance ? Vector3D.Normalize(aimingVectorLocal) * maxRaycastDistance : aimingVectorLocal;

                        elevationRotor.TargetVelocityRad = (float)(laserController.RotationIndicator.X * 0.05);
                        azimuthRotor.TargetVelocityRad = (float)(laserController.RotationIndicator.Y * 0.05);
                    }
                }

                else
                {
                    elevationRotor.TargetVelocityRad = (float)(laserController.RotationIndicator.X * 0.05);
                    azimuthRotor.TargetVelocityRad = (float)(laserController.RotationIndicator.Y * 0.05);
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

        public class MissileLauncher
        {
            List<TargetingLaser> targetingLasers = new List<TargetingLaser>();
            Queue<Missile> missileQueue = new Queue<Missile>();

            public MissileLauncher(Program program, int numberOfMissiles, int numberOfTargetingLasers)
            {
                for (int i = 1; i <= numberOfMissiles; i++)
                {
                    missileQueue.Enqueue(new Missile(program, i));
                }

                for (int i = 1; i <= numberOfTargetingLasers; i++)
                {
                    targetingLasers.Add(new TargetingLaser(program, i, "Targeting Laser Data " + i));
                }
            }

            public void Run()
            {
                foreach (TargetingLaser targetingLaser in targetingLasers)
                {
                    targetingLaser.Run();
                }
            }

            public void LaunchNextAvailableMissile(int targetingLaserNumber)
            {
                if (missileQueue.Count != 0)
                {
                    missileQueue.Dequeue().Launch("Targeting Laser Data " + targetingLaserNumber);
                }
            }


        }

        /* public class MissileLauncher
        {
            List<TargetingLaser> targetingLasers = new List<TargetingLaser>();
            List<MissileBay> missileBays = new List<MissileBay>();
            Queue<MissileBay> missileBayQueue = new Queue<MissileBay>();

            public MissileLauncher(Program program, int numberOfMissileBays, int numberOfTargetingLasers, double minimumFuelPercentage = 0.5)
            {
                for (int i = 1; i <= numberOfMissileBays; i++)
                {
                    missileBays.Add(new MissileBay(program, i, minimumFuelPercentage));
                }

                for (int i = 1; i <= numberOfTargetingLasers; i++)
                {
                    targetingLasers.Add(new TargetingLaser(program, i, "Targeting Laser Data " + i));
                }
            }

            public void Run()
            {
                foreach (MissileBay missileBay in missileBays)
                {
                    missileBay.Run();

                    if (missileBay.status == MissileBay.Status.Ready)
                    {
                        missileBayQueue.Enqueue(missileBay);
                    }
                }

                foreach (TargetingLaser targetingLaser in targetingLasers)
                {
                    targetingLaser.Run();
                }
            }

            public void LaunchNextAvailableMissile(int targetingLaserNumber)
            {
                if (missileBayQueue.Count != 0)
                {
                    missileBayQueue.Dequeue().Launch("Targeting Laser Data " + targetingLaserNumber);
                }
            }


        } */

         public class Missile
        {
            Program program;
            int number;
            IMyProgrammableBlock missileComputer;
            

            public Missile(Program program, int number)
            {
                this.program = program;
                this.number = number;
                missileComputer = (IMyProgrammableBlock)program.GridTerminalSystem.GetBlockWithName("Missile Computer " + number);
            }

            public void Launch(string broadcastTag)
            {
                missileComputer.TryRun("Launch " + "\"" + broadcastTag + "\"");
            }
        }
    }
}
