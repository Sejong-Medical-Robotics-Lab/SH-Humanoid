using UnityEngine;

/// <summary>
/// Creates a simple local XYZ guide at this GameObject's origin.
/// Unity coordinates: +X = right, +Y = up, +Z = forward.
/// No ROS or URDF coordinate conversion is performed here.
/// In MetaQuestTest.unity this object is parented to TrackingSpace so the guide
/// displays the same coordinate frame used by ControllerPoseLogger.
/// </summary>
public class AxisGuide : MonoBehaviour
{
    [Header("Axis Guide Settings (Unity units are meters)")]
    [Min(0.01f)] public float axisLength = 1.0f;
    [Min(0.001f)] public float thickness = 0.02f;
    [Min(0.001f)] public float originSize = 0.06f;
    [Min(0.001f)] public float labelSize = 0.1f;

    [Header("Optional Parts")]
    public bool showLabels = true;
    public bool showOriginSphere = true;

    private const string GeneratedRootName = "GeneratedAxisGuide";

    private void Start()
    {
        // Do not create a second guide if one already exists below this object.
        if (transform.Find(GeneratedRootName) == null)
        {
            CreateGuide();
        }
    }

    private void CreateGuide()
    {
        GameObject root = new GameObject(GeneratedRootName);
        root.transform.SetParent(transform, false);

        CreateAxis(root.transform, "X_Axis", Vector3.right, Color.red);
        CreateAxis(root.transform, "Y_Axis", Vector3.up, Color.green);
        CreateAxis(root.transform, "Z_Axis", Vector3.forward, Color.blue);

        if (showOriginSphere)
        {
            CreateOrigin(root.transform);
        }

        if (showLabels)
        {
            CreateLabel(root.transform, "X_Label", "X", Vector3.right, Color.red);
            CreateLabel(root.transform, "Y_Label", "Y", Vector3.up, Color.green);
            CreateLabel(root.transform, "Z_Label", "Z", Vector3.forward, Color.blue);
        }
    }

    private void CreateAxis(Transform parent, string objectName, Vector3 direction, Color color)
    {
        GameObject axis = GameObject.CreatePrimitive(PrimitiveType.Cube);
        axis.name = objectName;
        axis.transform.SetParent(parent, false);

        // Cubes are centered, so half-length placement makes each axis start at the origin.
        axis.transform.localPosition = direction * (axisLength * 0.5f);
        axis.transform.localRotation = Quaternion.FromToRotation(Vector3.right, direction);
        axis.transform.localScale = new Vector3(axisLength, thickness, thickness);
        SetColor(axis, color);

        // This is a visual debugging guide, so it does not need physics colliders.
        Collider axisCollider = axis.GetComponent<Collider>();
        if (axisCollider != null)
        {
            Destroy(axisCollider);
        }
    }

    private void CreateOrigin(Transform parent)
    {
        GameObject origin = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        origin.name = "Origin";
        origin.transform.SetParent(parent, false);
        origin.transform.localPosition = Vector3.zero;
        origin.transform.localScale = Vector3.one * originSize;
        SetColor(origin, Color.white);

        Collider originCollider = origin.GetComponent<Collider>();
        if (originCollider != null)
        {
            Destroy(originCollider);
        }
    }

    private void CreateLabel(Transform parent, string objectName, string labelText,
        Vector3 direction, Color color)
    {
        GameObject label = new GameObject(objectName);
        label.transform.SetParent(parent, false);
        label.transform.localPosition = direction * (axisLength + labelSize * 0.75f);
        label.transform.localRotation = Quaternion.identity;

        // Built-in TextMesh avoids a TextMeshPro package dependency.
        TextMesh textMesh = label.AddComponent<TextMesh>();
        textMesh.text = labelText;
        textMesh.color = color;
        textMesh.anchor = TextAnchor.MiddleCenter;
        textMesh.alignment = TextAlignment.Center;
        textMesh.characterSize = labelSize;
        textMesh.fontSize = 64;
    }

    private void SetColor(GameObject target, Color color)
    {
        Renderer targetRenderer = target.GetComponent<Renderer>();
        if (targetRenderer == null)
        {
            return;
        }

        Shader shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader == null)
        {
            shader = Shader.Find("Standard");
        }

        if (shader != null)
        {
            Material material = new Material(shader);
            material.color = color;
            targetRenderer.material = material;
        }
    }
}
