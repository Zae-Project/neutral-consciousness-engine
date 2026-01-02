using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;

/// <summary>
/// ROS Bridge Client for Unity
/// 
/// Handles communication between Unity and the ROS 2 backend,
/// sending sensory data and receiving motor commands.
/// </summary>
public class RosBridgeClient : MonoBehaviour
{
    [Header("ROS Connection")]
    [SerializeField] private string rosIP = "127.0.0.1";
    [SerializeField] private int rosPort = 10000;
    
    [Header("Camera Settings")]
    [SerializeField] private Camera virtualCamera;
    [SerializeField] private int imageWidth = 640;
    [SerializeField] private int imageHeight = 480;
    [SerializeField] private float publishRate = 30f;
    
    private ROSConnection ros;
    private float lastPublishTime;
    private RenderTexture renderTexture;
    private Texture2D capturedImage;
    
    // Topic names
    private const string CAMERA_TOPIC = "unity/camera/raw";
    private const string KILL_SWITCH_TOPIC = "firewall/kill_switch";
    
    void Start()
    {
        InitializeROS();
        InitializeCamera();
        SubscribeToTopics();
    }
    
    private void InitializeROS()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Connect(rosIP, rosPort);
        
        // Register publishers
        ros.RegisterPublisher<ImageMsg>(CAMERA_TOPIC);
        
        Debug.Log($"ROS Bridge initialized - Connecting to {rosIP}:{rosPort}");
    }
    
    private void InitializeCamera()
    {
        if (virtualCamera == null)
        {
            virtualCamera = Camera.main;
        }
        
        // Create render texture for camera capture
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        capturedImage = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
    }
    
    private void SubscribeToTopics()
    {
        // Subscribe to kill switch from Neural Firewall
        ros.Subscribe<BoolMsg>(KILL_SWITCH_TOPIC, OnKillSwitchReceived);
    }
    
    void Update()
    {
        // Publish camera feed at specified rate
        if (Time.time - lastPublishTime >= 1f / publishRate)
        {
            PublishCameraFeed();
            lastPublishTime = Time.time;
        }
    }
    
    private void PublishCameraFeed()
    {
        if (virtualCamera == null || ros == null) return;
        
        // Capture camera view
        RenderTexture previousRT = RenderTexture.active;
        RenderTexture.active = renderTexture;
        
        virtualCamera.targetTexture = renderTexture;
        virtualCamera.Render();
        
        capturedImage.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        capturedImage.Apply();
        
        virtualCamera.targetTexture = null;
        RenderTexture.active = previousRT;
        
        // Convert to ROS Image message
        byte[] imageData = capturedImage.GetRawTextureData();
        
        ImageMsg imageMsg = new ImageMsg
        {
            height = (uint)imageHeight,
            width = (uint)imageWidth,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(imageWidth * 3),
            data = imageData
        };
        
        ros.Publish(CAMERA_TOPIC, imageMsg);
    }
    
    private void OnKillSwitchReceived(BoolMsg msg)
    {
        if (msg.data)
        {
            Debug.LogWarning("KILL SWITCH RECEIVED - Disconnecting from ROS");
            DisconnectSafely();
        }
    }
    
    private void DisconnectSafely()
    {
        // Stop all communication
        Debug.Log("Emergency disconnection initiated");
        
        // Disable camera publishing
        enabled = false;
        
        // Additional safety measures could be implemented here:
        // - Save current state
        // - Freeze simulation
        // - Alert user
    }
    
    void OnDestroy()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
        }
    }
}
