// Run with the same Swift source compiled into the app; no ARKit/device needed.
var checks = 0
func expect(_ actual: ScanTrackingState.Guidance?, _ expected: ScanTrackingState.Guidance?, _ description: String) {
    precondition(actual == expected, description)
    checks += 1
}

var state = ScanTrackingState()
expect(state.update(isTracking: false, timestamp: 0), nil, "Startup is not a lost scan")
expect(state.update(isTracking: false, timestamp: 5), nil, "Prolonged initialization is not a lost scan")
expect(state.update(isTracking: true, timestamp: 6), nil, "First pose does not report recovery")
expect(state.update(isTracking: true, timestamp: 7), nil, "Healthy tracking stays quiet")
expect(state.update(isTracking: false, timestamp: 8), .trackingLost, "Loss after a pose needs guidance")
expect(state.update(isTracking: false, timestamp: 80), .trackingLost, "Guidance remains during long loss")
expect(state.update(isTracking: true, timestamp: 81), .trackingResumed, "Recovery reminds users to overlap")
expect(state.update(isTracking: true, timestamp: 88.9), .trackingResumed, "Recovery message stays visible")
expect(state.update(isTracking: true, timestamp: 89), nil, "Repeated frames do not extend the deadline")
expect(state.update(isTracking: true, timestamp: 90), nil, "Normal tracking remains quiet after expiry")

expect(state.update(isTracking: false, timestamp: 91), .trackingLost, "A second loss is reported")
expect(state.update(isTracking: true, timestamp: 92), .trackingResumed, "A second recovery is reported")
expect(state.update(isTracking: false, timestamp: 93), .trackingLost, "Loss preempts a recovery reminder")
expect(state.update(isTracking: true, timestamp: 94), .trackingResumed, "Recovery restarts its own deadline")
expect(state.update(isTracking: true, timestamp: 101), .trackingResumed, "Old deadline cannot hide new recovery")
expect(state.update(isTracking: true, timestamp: 102), nil, "New deadline eventually expires")

state.reset()
expect(state.update(isTracking: false, timestamp: 0), nil, "New scan cannot inherit a loss warning")
expect(state.update(isTracking: true, timestamp: 1), nil, "New scan cannot inherit a recovery reminder")
expect(state.update(isTracking: false, timestamp: 2), .trackingLost, "New scan still detects losses")
state.reset()
expect(state.update(isTracking: false, timestamp: 3), nil, "Stopping during a loss resets guidance")
expect(state.update(isTracking: true, timestamp: 4), nil, "Resuming starts a fresh tracking history")
expect(state.update(isTracking: false, timestamp: 5), .trackingLost, "Resumed recording detects losses")
expect(state.update(isTracking: true, timestamp: 6), .trackingResumed, "Resumed recording detects recovery")
state.reset()
expect(state.update(isTracking: true, timestamp: 7), nil, "Stopping during recovery resets its deadline")

print("ScanTrackingState: \(checks) checks passed")
