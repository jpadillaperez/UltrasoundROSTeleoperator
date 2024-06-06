using UnityEngine;
using Microsoft.MixedReality.OpenXR;
using TMPro;
using UnityEngine.XR.OpenXR.Input;

public class TestQRCodeDetection : MonoBehaviour
{
    [SerializeField] private GameObject mainText;
    [SerializeField] private ARMarkerManager markerManager;
    [SerializeField] private Transform ur5;
    [SerializeField] private Transform setup;

    private TextMeshProUGUI m_TextMeshPro;

    private void Start()
    {
        if (markerManager == null)
        {
            Debug.LogError("ARMarkerManager is not assigned.");
            return;
        }

        // Subscribe to the markersChanged event
        markerManager.markersChanged += OnMarkersChanged;
        
        // Initialize TextMeshPro
        m_TextMeshPro = (mainText != null) ? mainText.GetComponent<TextMeshProUGUI>() : null;
        if (m_TextMeshPro == null)
        {
            Debug.LogError("TextMeshProUGUI component not found on mainText GameObject.");
        }
    }

    /// <summary>
    /// Handles the markersChanged event and processes added, updated, and removed markers.
    /// </summary>
    /// <param name="args">Event arguments containing information about added, updated, and removed markers.</param>
    private void OnMarkersChanged(ARMarkersChangedEventArgs args)
    {
        foreach (var addedMarker in args.added)
        {
            HandleAddedMarker(addedMarker);
        }

        foreach (var updatedMarker in args.updated)
        {
            HandleUpdatedMarker(updatedMarker);
        }

        foreach (var removedMarkerId in args.removed)
        {
            HandleRemovedMarker(removedMarkerId);
        }
    }
    
    /// <summary>
    /// Handles logic for newly added markers.
    /// </summary>
    /// <param name="addedMarker">The newly added ARMarker.</param>
    private void HandleAddedMarker(ARMarker addedMarker)
    {
        Debug.Log($"QR Code Detected! Marker ID: {addedMarker.trackableId}");
        // You can access more information about the marker using addedMarker properties
        // For example, addedMarker.GetDecodedString() or addedMarker.GetQRCodeProperties()
        // Additional handling logic for newly added markers
    }
    
    /// <summary>
    /// Handles logic for updated markers.
    /// </summary>
    /// <param name="updatedMarker">The updated ARMarker.</param>
    private void HandleUpdatedMarker(ARMarker updatedMarker)
    {
        // Debug.Log($"QR Code updated! Marker ID: {updatedMarker}");
        // You can access information about the marker using updatedMarker properties
        // Additional handling logic for updated markers
        
        // Get the decoded string from the added marker
        string qrCodeString = updatedMarker.GetDecodedString();
        Debug.Log($"QR Code Updated! Decoded String: {qrCodeString}");
        
        if (qrCodeString == "M1")
        {
            ur5.transform.parent = updatedMarker.transform;

            Vector3 pos = new Vector3(0.015f, -0.282f, -0.011f);
            Quaternion rot = Quaternion.Euler(89.3716202f, 180.197952f, 180.000946f);

            ur5.localPosition = pos;
            ur5.localRotation = rot; 

            Matrix4x4 m = ur5.localToWorldMatrix;
            // Rotate around Y axis by 90 degrees
            m = m * Matrix4x4.Rotate(Quaternion.Euler(0, 90, 0));

            setup.position = m.GetColumn(3);
            setup.rotation = m.rotation;

            return;
        }


        // Set the QR code string to the TextMeshPro component
        if (m_TextMeshPro != null)
        {
            m_TextMeshPro.text = qrCodeString;
        }
    }
    
    /// <summary>
    /// Handles logic for removed markers.
    /// </summary>
    /// <param name="removedMarkerId">The ID of the removed marker.</param>
    private void HandleRemovedMarker(ARMarker removedMarkerId)
    {
        Debug.Log($"QR Code Removed! Marker ID: {removedMarkerId}");
        
        // Clear the TextMeshPro text when a marker is removed
        if (m_TextMeshPro != null)
        {
            m_TextMeshPro.text = string.Empty;
        }
    }
}
