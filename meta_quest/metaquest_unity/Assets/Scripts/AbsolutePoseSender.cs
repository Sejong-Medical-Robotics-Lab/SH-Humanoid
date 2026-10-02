using System;
using System.Globalization;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

/// <summary>
/// 유니트리 xr_teleoperate 방식용 송신기: 머리 + 양손의 "절대" 자세와 그립 상태를 보낸다.
/// 기존 ControllerPoseLogger(상대 이동, 포트 15000)는 그대로 두고, 이 컴포넌트를 같은 오브젝트에 추가.
/// 모든 값은 TrackingSpace 기준 → RobotBaseLinkCoordinates로 로봇 축(+X 오른쪽, +Y 정면, +Z 위)으로 변환.
/// 패킷: SHABS,1,t, head(x,y,z,qx,qy,qz,qw), gripL, left(7), gripR, right(7)   (필드 26개, UDP 15001)
/// </summary>
public class AbsolutePoseSender : MonoBehaviour
{
    [Header("OVRCameraRig 참조")]
    public Transform trackingSpace;     // OVRCameraRig/TrackingSpace
    public Transform head;              // OVRCameraRig/TrackingSpace/CenterEyeAnchor
    public Transform leftController;    // LeftControllerAnchor
    public Transform rightController;   // RightControllerAnchor

    [Header("UDP")]
    public bool sendUdp = true;
    public string ros2ComputerAddress = "192.168.50.77";
    [Range(1024, 65535)] public int ros2UdpPort = 15001;

    [Header("입력")]
    [Tooltip("씬에 OVRManager가 없는 구성이라 OVRInput.Update()를 직접 호출 (버튼 '누름 상태'만 읽으므로 중복 호출 무방)")]
    public bool callOvrInputUpdate = true;

    private UdpClient udpClient;
    private bool warned;

    private void OnEnable() { udpClient = new UdpClient(); }

    private void OnDisable()
    {
        udpClient?.Dispose();
        udpClient = null;
    }

    private void Update()
    {
        if (!sendUdp || udpClient == null) return;
        if (trackingSpace == null || head == null || leftController == null || rightController == null)
        {
            if (!warned)
            {
                warned = true;
                Debug.LogWarning("[ABS_POSE] TrackingSpace / Head(CenterEyeAnchor) / Left / Right 참조를 모두 연결하세요.", this);
            }
            return;
        }

        if (callOvrInputUpdate) OVRInput.Update();
        bool gripL = Grip(OVRInput.Controller.LTouch);
        bool gripR = Grip(OVRInput.Controller.RTouch);

        var sb = new StringBuilder(256);
        sb.Append("SHABS,1,");
        sb.Append(Time.realtimeSinceStartupAsDouble.ToString("F4", CultureInfo.InvariantCulture));
        AppendPose(sb, head);
        sb.Append(gripL ? ",1" : ",0");
        AppendPose(sb, leftController);
        sb.Append(gripR ? ",1" : ",0");
        AppendPose(sb, rightController);

        try
        {
            byte[] bytes = Encoding.ASCII.GetBytes(sb.ToString());
            udpClient.Send(bytes, bytes.Length, ros2ComputerAddress, ros2UdpPort);
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[ABS_POSE] UDP send failed: {e.Message}", this);
        }
    }

    private static bool Grip(OVRInput.Controller c)
    {
        return OVRInput.Get(OVRInput.Button.PrimaryHandTrigger, c)
            || OVRInput.Get(OVRInput.Button.PrimaryIndexTrigger, c);
    }

    private void AppendPose(StringBuilder sb, Transform t)
    {
        Vector3 p = trackingSpace.InverseTransformPoint(t.position);
        Quaternion q = Quaternion.Inverse(trackingSpace.rotation) * t.rotation;
        Vector3 rp = RobotBaseLinkCoordinates.UnityPositionToRobotCandidate(p);
        Quaternion rq = RobotBaseLinkCoordinates.UnityRotationToRobotCandidate(q);
        sb.AppendFormat(CultureInfo.InvariantCulture,
            ",{0:F5},{1:F5},{2:F5},{3:F6},{4:F6},{5:F6},{6:F6}",
            rp.x, rp.y, rp.z, rq.x, rq.y, rq.z, rq.w);
    }
}