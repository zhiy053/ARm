using System.Collections.Generic;
using Unity.XR.CoreUtils;
using UnityEngine;
using UnityEngine.XR.ARFoundation;

namespace AR80sRetro
{
    public sealed class RetroReplacementManager : MonoBehaviour
    {
        private sealed class TrackedReplacement
        {
            public GameObject Instance;
            public ARAnchor Anchor;
            public Vector3 LockedScale;
            public Vector3 TargetLocalPosition;
            public Quaternion TargetLocalRotation = Quaternion.identity;
            public int ConfirmedFrames;
            public Vector3 ConfirmationPositionSum;
            public int RelocationFrames;
            public Vector3 RelocationPositionSum;
            public float LastSeenTime;
        }

        [Header("Dependencies")]
        [SerializeField] private RetroPrefabLibrary prefabLibrary;
        [SerializeField] private ARRaycastPositionSolver positionSolver;
        [SerializeField] private Transform contentRoot;
        [SerializeField] private Camera arCamera;

        private ARAnchorManager anchorManager;

        [Header("Tracking stability")]
        [SerializeField, Min(0f)] private float lostGraceSeconds = 3f;
        [SerializeField, Min(0.01f)] private float duplicateRadiusMeters = 0.08f;
        [SerializeField, Min(0f)] private float positionDeadZoneMeters = 0.04f;
        [SerializeField, Min(1)] private int relocationConfirmationFrames = 3;
        [SerializeField, Min(0.01f)] private float relocationConsistencyRadiusMeters = 0.06f;
        [SerializeField, Min(0f)] private float maxMeasurementJumpMeters = 0.75f;

        private readonly Dictionary<string, TrackedReplacement> trackedByLabel =
            new Dictionary<string, TrackedReplacement>();
        private readonly HashSet<string> processedLabels = new HashSet<string>();

        private void Awake()
        {
            if (arCamera == null)
            {
                arCamera = Camera.main;
            }

            EnsureAnchorManager();
        }

        private void Reset()
        {
            positionSolver = FindObjectOfType<ARRaycastPositionSolver>();
            arCamera = Camera.main;
            contentRoot = transform;
        }

        private void Update()
        {
            SmoothVisuals();
            RemoveExpiredReplacements();
        }

        public void ApplyDetections(IReadOnlyList<DetectionResult> detections)
        {
            if (detections == null || prefabLibrary == null || positionSolver == null)
            {
                return;
            }

            float now = Time.time;
            processedLabels.Clear();
            for (int i = 0; i < detections.Count; i++)
            {
                DetectionResult detection = detections[i];
                if (!prefabLibrary.TryGetRule(detection.Label, out RetroReplacementRule rule)
                    || !detection.IsValid(rule.MinConfidence))
                {
                    continue;
                }

                string key = GetTrackingKey(rule);
                // The current data model tracks one replacement per label. YOLO results
                // are confidence-sorted, so only the best detection may advance that
                // label's confirmation counter during this inference cycle.
                if (!processedLabels.Add(key))
                {
                    continue;
                }

                // A valid 2D detection keeps an existing replacement alive even when the
                // current frame cannot produce a plane raycast.
                if (trackedByLabel.TryGetValue(key, out TrackedReplacement existing))
                {
                    existing.LastSeenTime = now;
                }

                if (!positionSolver.TrySolvePose(detection, out Pose pose))
                {
                    continue;
                }

                pose.position += Vector3.up * rule.VerticalOffsetMeters;
                UpdateReplacement(key, rule, detection, pose, now);
            }
        }

        private void UpdateReplacement(
            string key,
            RetroReplacementRule rule,
            DetectionResult detection,
            Pose pose,
            float now)
        {
            if (!trackedByLabel.TryGetValue(key, out TrackedReplacement tracked))
            {
                tracked = new TrackedReplacement();
                trackedByLabel.Add(key, tracked);
            }

            tracked.LastSeenTime = now;

            if (tracked.Instance == null)
            {
                ConfirmInitialPlacement(tracked, rule, detection, pose);
                return;
            }

            ConsiderRelocation(tracked, rule, pose);
        }

        private void ConfirmInitialPlacement(
            TrackedReplacement tracked,
            RetroReplacementRule rule,
            DetectionResult detection,
            Pose pose)
        {
            if (tracked.ConfirmedFrames == 0)
            {
                tracked.ConfirmedFrames = 1;
                tracked.ConfirmationPositionSum = pose.position;
            }
            else
            {
                Vector3 averagePosition =
                    tracked.ConfirmationPositionSum / tracked.ConfirmedFrames;
                if (Vector3.Distance(averagePosition, pose.position) > duplicateRadiusMeters)
                {
                    tracked.ConfirmedFrames = 1;
                    tracked.ConfirmationPositionSum = pose.position;
                }
                else
                {
                    tracked.ConfirmedFrames++;
                    tracked.ConfirmationPositionSum += pose.position;
                }
            }

            if (tracked.ConfirmedFrames < rule.ConfirmationFrames)
            {
                return;
            }

            Pose stablePose = new Pose(
                tracked.ConfirmationPositionSum / tracked.ConfirmedFrames,
                pose.rotation);
            CreateReplacement(tracked, rule, detection, stablePose);
        }

        private void CreateReplacement(
            TrackedReplacement tracked,
            RetroReplacementRule rule,
            DetectionResult detection,
            Pose pose)
        {
            tracked.Anchor = CreateAnchor(rule.DetectionLabel, pose);
            Transform parent = tracked.Anchor != null
                ? tracked.Anchor.transform
                : contentRoot;

            tracked.Instance = Instantiate(rule.Prefab, pose.position, pose.rotation, parent);
            tracked.Instance.transform.localScale = rule.SpawnScale;
            tracked.LockedScale = EstimateTargetScale(tracked.Instance, rule, detection, pose);
            tracked.Instance.transform.localScale = tracked.LockedScale;
            AlignBottomToPlane(tracked.Instance, pose.position.y);

            // The bottom-aligned local pose and the initial scale become immutable. Raw
            // detector box changes no longer make the model rotate or "breathe".
            tracked.TargetLocalPosition = tracked.Instance.transform.localPosition;
            tracked.TargetLocalRotation = tracked.Instance.transform.localRotation;
            tracked.ConfirmedFrames = 0;
            tracked.ConfirmationPositionSum = Vector3.zero;
        }

        private void ConsiderRelocation(
            TrackedReplacement tracked,
            RetroReplacementRule rule,
            Pose measuredPose)
        {
            Vector3 currentPosition = tracked.Anchor != null
                ? tracked.Anchor.transform.position
                : tracked.Instance.transform.position;
            float distance = Vector3.Distance(currentPosition, measuredPose.position);

            if (distance <= positionDeadZoneMeters)
            {
                ResetRelocationCandidate(tracked);
                return;
            }

            if (maxMeasurementJumpMeters > 0f && distance > maxMeasurementJumpMeters)
            {
                ResetRelocationCandidate(tracked);
                return;
            }

            if (tracked.RelocationFrames == 0)
            {
                tracked.RelocationFrames = 1;
                tracked.RelocationPositionSum = measuredPose.position;
                return;
            }

            Vector3 averagePosition =
                tracked.RelocationPositionSum / tracked.RelocationFrames;
            if (Vector3.Distance(averagePosition, measuredPose.position)
                > relocationConsistencyRadiusMeters)
            {
                tracked.RelocationFrames = 1;
                tracked.RelocationPositionSum = measuredPose.position;
                return;
            }

            tracked.RelocationFrames++;
            tracked.RelocationPositionSum += measuredPose.position;
            if (tracked.RelocationFrames < relocationConfirmationFrames)
            {
                return;
            }

            Vector3 stablePosition =
                tracked.RelocationPositionSum / tracked.RelocationFrames;
            Quaternion lockedRotation = tracked.Anchor != null
                ? tracked.Anchor.transform.rotation
                : tracked.Instance.transform.rotation;
            RelocateAnchor(tracked, rule.DetectionLabel, new Pose(stablePosition, lockedRotation));
            ResetRelocationCandidate(tracked);
        }

        private void RelocateAnchor(
            TrackedReplacement tracked,
            string label,
            Pose stablePose)
        {
            ARAnchor previousAnchor = tracked.Anchor;
            ARAnchor newAnchor = CreateAnchor(label, stablePose);
            if (newAnchor == null)
            {
                return;
            }

            // Preserve the visible world pose, then let Update move the visual smoothly
            // toward its locked local pose under the new anchor.
            tracked.Instance.transform.SetParent(newAnchor.transform, true);
            tracked.Anchor = newAnchor;

            if (previousAnchor != null)
            {
                Destroy(previousAnchor.gameObject);
            }
        }

        private ARAnchor CreateAnchor(string label, Pose pose)
        {
            EnsureAnchorManager();
            if (anchorManager == null || !anchorManager.isActiveAndEnabled)
            {
                Debug.LogWarning(
                    "AR Anchor Manager is unavailable. Falling back to an untracked transform.",
                    this);
                return null;
            }

            GameObject anchorObject = new GameObject($"{label} Stable Anchor");
            anchorObject.transform.SetPositionAndRotation(pose.position, pose.rotation);
            return anchorObject.AddComponent<ARAnchor>();
        }

        private void EnsureAnchorManager()
        {
            if (anchorManager != null)
            {
                return;
            }

            anchorManager = FindObjectOfType<ARAnchorManager>();
            if (anchorManager != null)
            {
                return;
            }

            XROrigin origin = FindObjectOfType<XROrigin>();
            if (origin != null)
            {
                anchorManager = origin.gameObject.AddComponent<ARAnchorManager>();
            }
        }

        private void SmoothVisuals()
        {
            foreach (KeyValuePair<string, TrackedReplacement> item in trackedByLabel)
            {
                TrackedReplacement tracked = item.Value;
                if (tracked.Instance == null)
                {
                    continue;
                }

                if (!prefabLibrary.TryGetRule(item.Key, out RetroReplacementRule rule))
                {
                    continue;
                }

                float smoothing = Mathf.Clamp01(rule.PositionSmoothing);
                float frameIndependentSmoothing = smoothing >= 1f
                    ? 1f
                    : 1f - Mathf.Pow(1f - smoothing, Time.deltaTime * 60f);
                Transform instanceTransform = tracked.Instance.transform;
                instanceTransform.localPosition = Vector3.Lerp(
                    instanceTransform.localPosition,
                    tracked.TargetLocalPosition,
                    frameIndependentSmoothing);
                instanceTransform.localRotation = Quaternion.Slerp(
                    instanceTransform.localRotation,
                    tracked.TargetLocalRotation,
                    frameIndependentSmoothing);
                instanceTransform.localScale = tracked.LockedScale;
            }
        }

        private Vector3 EstimateTargetScale(
            GameObject instance,
            RetroReplacementRule rule,
            DetectionResult detection,
            Pose pose)
        {
            if (!rule.EstimateScaleFromBoundingBox || arCamera == null)
            {
                return rule.SpawnScale;
            }

            if (!TryGetRendererBounds(instance, out Bounds bounds) || bounds.size.y <= 0.0001f)
            {
                return rule.SpawnScale;
            }

            float distance = Vector3.Distance(arCamera.transform.position, pose.position);
            float visibleWorldHeight = 2f
                * distance
                * Mathf.Tan(arCamera.fieldOfView * 0.5f * Mathf.Deg2Rad);
            float targetHeight = detection.NormalizedBox.height
                * visibleWorldHeight
                * rule.EstimatedHeightMultiplier;
            float scaleMultiplier = targetHeight / bounds.size.y;
            Vector2 range = rule.ScaleMultiplierRange;
            if (range.x <= 0f && range.y <= 0f)
            {
                range = new Vector2(0.25f, 4f);
            }
            scaleMultiplier = Mathf.Clamp(
                scaleMultiplier,
                Mathf.Min(range.x, range.y),
                Mathf.Max(range.x, range.y));
            return Vector3.Scale(instance.transform.localScale, Vector3.one * scaleMultiplier);
        }

        private static void AlignBottomToPlane(GameObject instance, float planeHeight)
        {
            if (!TryGetRendererBounds(instance, out Bounds bounds))
            {
                return;
            }

            Transform instanceTransform = instance.transform;
            Vector3 position = instanceTransform.position;
            position.y += planeHeight - bounds.min.y;
            instanceTransform.position = position;
        }

        private static bool TryGetRendererBounds(GameObject instance, out Bounds bounds)
        {
            Renderer[] renderers = instance.GetComponentsInChildren<Renderer>();
            if (renderers.Length == 0)
            {
                bounds = default;
                return false;
            }

            bounds = renderers[0].bounds;
            for (int i = 1; i < renderers.Length; i++)
            {
                bounds.Encapsulate(renderers[i].bounds);
            }

            return true;
        }

        private void RemoveExpiredReplacements()
        {
            if (lostGraceSeconds <= 0f)
            {
                return;
            }

            float now = Time.time;
            s_ReusableExpiredKeys.Clear();

            foreach (KeyValuePair<string, TrackedReplacement> item in trackedByLabel)
            {
                if (now - item.Value.LastSeenTime > lostGraceSeconds)
                {
                    s_ReusableExpiredKeys.Add(item.Key);
                }
            }

            for (int i = 0; i < s_ReusableExpiredKeys.Count; i++)
            {
                string key = s_ReusableExpiredKeys[i];
                DestroyTrackedReplacement(trackedByLabel[key]);
                trackedByLabel.Remove(key);
            }
        }

        private void OnDestroy()
        {
            foreach (KeyValuePair<string, TrackedReplacement> item in trackedByLabel)
            {
                DestroyTrackedReplacement(item.Value);
            }

            trackedByLabel.Clear();
        }

        private static void DestroyTrackedReplacement(TrackedReplacement tracked)
        {
            if (tracked.Anchor != null)
            {
                Destroy(tracked.Anchor.gameObject);
            }
            else if (tracked.Instance != null)
            {
                Destroy(tracked.Instance);
            }
        }

        private static void ResetRelocationCandidate(TrackedReplacement tracked)
        {
            tracked.RelocationFrames = 0;
            tracked.RelocationPositionSum = Vector3.zero;
        }

        private static string GetTrackingKey(RetroReplacementRule rule)
        {
            return rule.DetectionLabel.ToLowerInvariant();
        }

        private static readonly List<string> s_ReusableExpiredKeys = new List<string>();
    }
}
