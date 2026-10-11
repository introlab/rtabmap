// Tracking recovery guidance, independent of ARKit so transitions can be tested.
struct ScanTrackingState {
    enum Guidance: Equatable {
        case trackingLost
        case trackingResumed

        var message: String {
            switch self {
            case .trackingLost:
                return "Tracking lost. Move slowly back to an area you already scanned."
            case .trackingResumed:
                return "Tracking resumed. Revisit a scanned area to help align the map."
            }
        }
    }

    private var hasTracked = false
    private var wasLost = false
    private var recoveryDeadline: Double?

    // Use ARFrame.timestamp (monotonic seconds), not wall-clock time.
    mutating func update(isTracking: Bool, timestamp: Double) -> Guidance? {
        if isTracking {
            if wasLost {
                recoveryDeadline = timestamp + 8.0
            }
            hasTracked = true
            wasLost = false
        } else if hasTracked {
            wasLost = true
            recoveryDeadline = nil
            return .trackingLost
        }

        if let deadline = recoveryDeadline, timestamp < deadline {
            return .trackingResumed
        }
        recoveryDeadline = nil
        return nil
    }

    mutating func reset() {
        self = ScanTrackingState()
    }
}
