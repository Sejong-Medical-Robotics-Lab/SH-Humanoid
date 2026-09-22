using UnityEngine.InputSystem;
using System;
using System.Globalization;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

/// <summary>
/// Logs controller poses relative to TrackingSpace and shows a provisional base_link position.
/// </summary>
public class ControllerPoseLogger : MonoBehaviour
{
    [Header("TrackingSpace pose sources")]
    public Transform trackingSpace;
    public Transform leftController;
    public Transform rightController;

    [Header("Logging")]
    [Min(0.02f)] public float logInterval = 0.2f;

    [Header("ROS 2 UDP bridge")]
    public bool sendUdp = true;
    public string ros2ComputerAddress = "127.0.0.1";
    [Range(1024, 65535)] public int ros2UdpPort = 15000;
    [Min(0.02f)] public float sendInterval = 0.05f;
    public KeyCode calibrateKey = KeyCode.C;

    [Header("Robot neutral hand poses (base_link, metres)")]
    public Vector3 leftNeutralPosition = new Vector3(-0.166f, 0.003f, 0.582f);
    public Vector3 rightNeutralPosition = new Vector3(0.166f, 0.003f, 0.584f);

    private float timer;
    private float sendTimer;
    private bool hasWarnedAboutMissingReferences;
    private bool calibrated;
    private Vector3 leftCalibrationPosition;
    private Vector3 rightCalibrationPosition;
    private Quaternion leftCalibrationRotation;
    private Quaternion rightCalibrationRotation;
    private UdpClient udpClient;

    private void OnValidate()
    {
        logInterval = Mathf.Max(0.02f, logInterval);
        sendInterval = Mathf.Max(0.02f, sendInterval);
    }

    private void OnEnable()
    {
        if (sendUdp)
        {
            udpClient = new UdpClient();
        }
    }

    private void OnDisable()
    {
        udpClient?.Dispose();
        udpClient = null;
    }

    private void Update()
    {
        if (!HasRequiredReferences())
        {
            WarnAboutMissingReferencesOnce();
            return;
        }

        if (Keyboard.current != null && Keyboard.current.cKey.wasPressedThisFrame)
        {
            CalibrateNeutralPose();
        }

        timer += Time.deltaTime;
        if (timer < logInterval)
        {
            return;
        }

        timer = 0f;
        LogControllerPose("LEFT", leftController);
        LogControllerPose("RIGHT", rightController);

        sendTimer += logInterval;
        if (sendUdp && calibrated && sendTimer >= sendInterval)
        {
            sendTimer = 0f;
            SendControllerPose("L", leftController, leftCalibrationPosition,
                leftCalibrationRotation, leftNeutralPosition);
            SendControllerPose("R", rightController, rightCalibrationPosition,
                rightCalibrationRotation, rightNeutralPosition);
        }
    }

    private void CalibrateNeutralPose()
    {
        leftCalibrationPosition = trackingSpace.InverseTransformPoint(leftController.position);
        rightCalibrationPosition = trackingSpace.InverseTransformPoint(rightController.position);
        leftCalibrationRotation = Quaternion.Inverse(trackingSpace.rotation) * leftController.rotation;
        rightCalibrationRotation = Quaternion.Inverse(trackingSpace.rotation) * rightController.rotation;
        calibrated = true;
        Debug.Log(
            "[CTRL_POSE] Calibrated. Current controller poses now map to the URDF home hand poses. " +
            "UDP transmission is active; press C again to recalibrate.", this);
    }

    private void SendControllerPose(
        string side,
        Transform controller,
        Vector3 calibrationPosition,
        Quaternion calibrationRotation,
        Vector3 neutralRobotPosition)
    {
        Vector3 unityPosition = trackingSpace.InverseTransformPoint(controller.position);
        Quaternion unityRotation = Quaternion.Inverse(trackingSpace.rotation) * controller.rotation;
        Vector3 unityDelta = unityPosition - calibrationPosition;
        Quaternion unityRotationDelta = Quaternion.Inverse(calibrationRotation) * unityRotation;
        Vector3 robotPosition = neutralRobotPosition +
            RobotBaseLinkCoordinates.UnityPositionToRobotCandidate(unityDelta);
        Quaternion robotRotation =
            RobotBaseLinkCoordinates.UnityRotationToRobotCandidate(unityRotationDelta);
        string packet = string.Format(
            CultureInfo.InvariantCulture,
            "SHPOSE,1,{0},{1:F6},{2:F6},{3:F6},{4:F6},{5:F6},{6:F6},{7:F6},{8:F6}",
            side, Time.realtimeSinceStartupAsDouble,
            robotPosition.x, robotPosition.y, robotPosition.z,
            robotRotation.x, robotRotation.y, robotRotation.z, robotRotation.w);
        try
        {
            byte[] bytes = Encoding.ASCII.GetBytes(packet);
            udpClient.Send(bytes, bytes.Length, ros2ComputerAddress, ros2UdpPort);
        }
        catch (Exception exception)
        {
            Debug.LogWarning($"[CTRL_POSE] UDP send failed: {exception.Message}", this);
        }
    }

    private bool HasRequiredReferences()
    {
        return trackingSpace != null && leftController != null && rightController != null;
    }

    private void WarnAboutMissingReferencesOnce()
    {
        if (hasWarnedAboutMissingReferences)
        {
            return;
        }

        hasWarnedAboutMissingReferences = true;
        Debug.LogWarning(
            "[CTRL_POSE] TrackingSpace/LeftController/RightController reference is missing. " +
            "Assign all three references on PoseLogger. Logging is paused.", this);
    }

    private void LogControllerPose(string side, Transform controller)
    {
        Vector3 unityPosition = trackingSpace.InverseTransformPoint(controller.position);
        Quaternion unityRotation = Quaternion.Inverse(trackingSpace.rotation) * controller.rotation;
        Vector3 robotPosition = RobotBaseLinkCoordinates.UnityPositionToRobotCandidate(unityPosition);

        // Rotation remains in Unity TrackingSpace coordinates until real Quest testing.
        Debug.Log(
            $"[CTRL_POSE][{side}] " +
            $"UnityTracking Position={unityPosition:F3} " +
            $"RotationQuaternion={FormatQuaternion(unityRotation)} " +
            $"RotationEuler={unityRotation.eulerAngles:F1} | " +
            $"RobotBaseLink PositionCandidate={robotPosition:F3} " +
            "(VERIFY signs and rotation with real Quest)", this);
    }

    private static string FormatQuaternion(Quaternion rotation)
    {
        return $"({rotation.x:F3}, {rotation.y:F3}, {rotation.z:F3}, {rotation.w:F3})";
    }
}
