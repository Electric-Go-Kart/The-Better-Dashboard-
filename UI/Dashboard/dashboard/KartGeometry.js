.pragma library

// Tunable geometry for the What The Kart Sees page. Distances in metres.
var kartLengthM = 3.0
var kartWidthM = 1.5
// Lidar sits this far behind the kart's centre.
var lidarBehindCentreM = 0.5

var rangeM = 20
var dangerM = 5
var cautionM = 10

// Combined horizontal field of view of the two cameras (split down the centre line).
var cameraFovDeg = 160

// Kart-body rectangle relative to the lidar (x forward, y right).
function kartFrontM() { return kartLengthM / 2 + lidarBehindCentreM }
function kartRearM() { return kartLengthM / 2 - lidarBehindCentreM }

function formatDistance(metres, imperial, approximate) {
    const prefix = approximate ? "~" : ""
    if (imperial)
        return prefix + Math.round(metres * 3.28084) + " ft"
    return prefix + metres.toFixed(1) + " m"
}
