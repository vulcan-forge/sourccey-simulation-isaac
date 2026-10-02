using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using ServerCalculations.UnityBridge;
using UnityEngine;

// Add to the GameObject with FullRobotVisualDriver, in a COPY of your Unity scene.
// Sends raw rig angles, not calibrated hardware RobotAction arm fields.
[RequireComponent(typeof(FullRobotVisualDriver))]
public sealed class MuJoCoPreviewSender : MonoBehaviour
{
    public int port = 8765;
    [Range(10, 120)] public int sendHz = 60;
    private FullRobotVisualDriver source;
    private UdpClient udp;
    private float nextSend;
    [Serializable] private class Arm { public float[] degrees; public float closure; }
    [Serializable] private class Packet
    {
        public string schema = "sourccey.mujoco.v1";
        public float[] @base;
        public float elevator;
        public Arm left;
        public Arm right;
    }
    private void OnEnable()
    {
        source = GetComponent<FullRobotVisualDriver>();
        udp = new UdpClient();
        udp.Connect(IPAddress.Loopback, port);
    }
    private static float Angle(ArticulationBody body) =>
        body != null && body.jointPosition.dofCount > 0 ? body.jointPosition[0] * Mathf.Rad2Deg : 0;
    private static Arm ReadArm(ArticulationBody pan, ArticulationBody lift, ArticulationBody elbow,
        VRToRobotMapper2 mapper, TeleopInputState input) => new Arm
    {
        degrees = new[] { Angle(pan), Angle(lift), Angle(elbow),
            mapper != null ? mapper.GetCurrentLogicalWristFlexDegrees() : 0,
            mapper != null ? mapper.GetCurrentLogicalWristRollDegrees() : 0 },
        closure = input != null ? input.CurrentGripperValue : 100
    };
    private void LateUpdate()
    {
        if (Time.unscaledTime < nextSend || udp == null) return;
        if (source.leftPan == null || source.leftLift == null || source.leftElbow == null ||
            source.rightPan == null || source.rightLift == null || source.rightElbow == null) return;
        nextSend = Time.unscaledTime + 1f / sendHz;
        var action = RobotController.Instance != null ? RobotController.Instance.CurrentAction : null;
        var packet = new Packet {
            @base = new[] { (float)(action?.x_vel ?? 0), (float)(action?.y_vel ?? 0), (float)(action?.theta_vel ?? 0) },
            elevator = (float)(action?.z_vel ?? 100),
            left = ReadArm(source.leftPan, source.leftLift, source.leftElbow, source.leftMapper, source.leftInput),
            right = ReadArm(source.rightPan, source.rightLift, source.rightElbow, source.rightMapper, source.rightInput)
        };
        byte[] bytes = Encoding.UTF8.GetBytes(JsonUtility.ToJson(packet));
        try { udp.Send(bytes, bytes.Length); }
        catch (SocketException) { /* Simulator may be closed; retry on next frame. */ }
    }
    private void OnDisable() { udp?.Close(); udp = null; }
}
