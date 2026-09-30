using UnityEngine.InputSystem;
using UnityEngine.XR;
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
    [Min(0.0f)] public float sendInterval = 0.0f;
    public KeyCode calibrateKey = KeyCode.C;

    [Header("Robot neutral hand poses (base_link, metres)")]
    public Vector3 leftNeutralPosition = new Vector3(-0.2048f, 0.2531f, 0.7589f);
    public Vector3 rightNeutralPosition = new Vector3(0.2088f, 0.2495f, 0.7595f);

    private float timer;
    private float sendTimer;
    private bool hasWarnedAboutMissingReferences;
    private bool calibrated;
    private bool leftHeld;
    private bool rightHeld;
    private Vector3 leftCalibrationPosition;
    private Vector3 rightCalibrationPosition;
    private Quaternion leftCalibrationRotation;
    private Quaternion rightCalibrationRotation;
    private UdpClient udpClient;

    private void OnValidate()
    {
        logInterval = Mathf.Max(0.02f, logInterval);
        sendInterval = Mathf.Max(0.0f, sendInterval);
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

        // 클러치: 그립 버튼을 누르는 동안만 추종. 누르는 순간 자동 보정.
        bool lNow = ReadGrip(XRNode.LeftHand);
        bool rNow = ReadGrip(XRNode.RightHand);

        if (lNow && !leftHeld) CalibrateSide(true);
        if (rNow && !rightHeld) CalibrateSide(false);
        leftHeld = lNow;
        rightHeld = rNow;

        sendTimer += Time.deltaTime;
        if (sendUdp && sendTimer >= sendInterval)
        {
            sendTimer = 0f;
            if (leftHeld)
            {
                SendControllerPose("L", leftController, leftCalibrationPosition,
                    leftCalibrationRotation, leftNeutralPosition);
            }
            if (rightHeld)
            {
                SendControllerPose("R", rightController, rightCalibrationPosition,
                    rightCalibrationRotation, rightNeutralPosition);
            }
        }

        timer += Time.deltaTime;
        if (timer < logInterval)
        {
            return;
        }

        timer = 0f;
        LogControllerPose("LEFT", leftController);
        LogControllerPose("RIGHT", rightController);
    }

    private static bool ReadGrip(XRNode node)
    {
        var devices = new System.Collections.Generic.List<UnityEngine.XR.InputDevice>();
        var side = node == XRNode.LeftHand
            ? UnityEngine.XR.InputDeviceCharacteristics.Left
            : UnityEngine.XR.InputDeviceCharacteristics.Right;
        UnityEngine.XR.InputDevices.GetDevicesWithCharacteristics(
            UnityEngine.XR.InputDeviceCharacteristics.HeldInHand | side, devices);

        foreach (var d in devices)
        {
            if (d.TryGetFeatureValue(UnityEngine.XR.CommonUsages.gripButton, out bool gb) && gb) return true;
            if (d.TryGetFeatureValue(UnityEngine.XR.CommonUsages.grip, out float g) && g > 0.5f) return true;
            if (d.TryGetFeatureValue(UnityEngine.XR.CommonUsages.triggerButton, out bool tb) && tb) return true;
        }
        return false;
    }

    private void CalibrateSide(bool left)
    {
        if (left)
        {
            leftCalibrationPosition = trackingSpace.InverseTransformPoint(leftController.position);
            leftCalibrationRotation = Quaternion.Inverse(trackingSpace.rotation) * leftController.rotation;
        }
        else
        {
            rightCalibrationPosition = trackingSpace.InverseTransformPoint(rightController.position);
            rightCalibrationRotation = Quaternion.Inverse(trackingSpace.rotation) * rightController.rotation;
        }
        calibrated = true;
        Debug.Log($"[CTRL_POSE] 클러치 ON ({(left ? "L" : "R")})", this);
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
            $"| Clutch L={leftHeld} R={rightHeld}", this);
    }

    private static string FormatQuaternion(Quaternion rotation)
    {
        return $"({rotation.x:F3}, {rotation.y:F3}, {rotation.z:F3}, {rotation.w:F3})";
    }
}
