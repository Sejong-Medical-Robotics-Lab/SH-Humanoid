using UnityEngine;

/// <summary>
/// Provisional mapping from Unity TrackingSpace to the existing robot base_link frame.
/// Unity: +X right, +Y up, +Z forward. Robot: +X right, +Y forward, +Z up.
/// IMPORTANT: axis signs and rotation still require validation with a real Quest.
/// </summary>
public static class RobotBaseLinkCoordinates
{
    public static Vector3 UnityPositionToRobotCandidate(Vector3 unityPosition)
    {
        // Candidate only. Do not treat signs as final before right/up/forward measurements.
        return new Vector3(unityPosition.x, unityPosition.z, unityPosition.y);
    }

    public static Quaternion UnityRotationToRobotCandidate(Quaternion unityRotation)
    {
        // Basis change for Unity (+X right, +Y up, +Z forward, left-handed)
        // to the documented robot frame (+X right, +Y forward, +Z up,
        // right-handed). Validate this with one-axis Quest rotations tomorrow.
        Quaternion robotRotation = new Quaternion(
            -unityRotation.x,
            -unityRotation.z,
            -unityRotation.y,
            unityRotation.w);
        return Quaternion.Normalize(robotRotation);
    }
}
