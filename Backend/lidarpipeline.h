#pragma once

#include <QList>
#include <QPointF>
#include <vector>

// Fusion/tracking stage of docs/ml-integration-spec.md section 4.5:
// range mask, kart body mask, adjacent-gap clustering on the polar scan, camera-wedge fusion (pixel
// columns -> bearing wedge, matched to clusters by angular overlap and class size priors),
// 3-of-5 persistence and a constant-velocity Kalman tracker with sticky classes.
// Not built yet: the homography range fallback for boxes with no lidar cluster (step 4).
// No Qt event loop or I/O, so it can run in a worker thread or a test.
//
// Input scans are metres in the kart frame relative to the lidar (x forward, y right), ordered by
// sensor angle (as Stl27lReader emits them). Velocities are relative to the kart (no ego-motion compensation).
class LidarPipeline
{
public:
    enum Class { Unknown = -1, Person, Bike, Vehicle, PoleBarrier, Cone, ClassCount };
    static const char *className(int c);
    static int classFromName(const char *name);

    // Pinhole camera, yaw clockwise from the kart's forward (left camera negative).
    struct Camera {
        double yawDeg = 0;
        double hfovDeg = 70.4;
        int widthPx = 640;
    };

    // One 2D detection in pixel columns of camera `cam`.
    struct CameraBox {
        int cam = 0;
        double u0 = 0, u1 = 0;
        int cls = Unknown;
        double conf = 0;
    };

    // A detection after conversion to a bearing wedge (deg, clockwise from forward) and merging
    // of boxes that are cut by the seam between two cameras.
    struct BoxResult {
        double a0 = 0, a1 = 0;
        int cam = 0;
        int cls = Unknown;
        double conf = 0;
        bool truncated = false;
        int cluster = -1;
        double match = 0;
    };

    struct Params {
        std::vector<Camera> cameras;
        // Box edges within this many px of the image border are treated as cut off.
        double edgePx = 3.0;
        // Boxes of the same class from different cameras whose wedges come within this gap are one object.
        double seamGapDeg = 2.0;
        // Cluster angular span is widened by this much on each side before comparing with a wedge.
        double wedgePadDeg = 1.0;
        double minOverlap = 0.25;
        double minMatch = 0.3;
        double voteDecay = 0.7;
        double minClassVote = 0.3;
        double rangeM = 20.0;
        // Kart body rectangle relative to the lidar (see KartGeometry.js). Points inside it are dropped.
        double bodyFrontM = 2.0;
        double bodyRearM = 1.0;
        double bodyHalfWidthM = 0.75;
        double bodyMarginM = 0.1;
        // Two neighbouring points join a cluster if closer than gapBaseM + gapPerRangeM * range.
        double gapBaseM = 0.12;
        double gapPerRangeM = 0.0073;
        int minClusterPoints = 3;
        // Clusters longer than this are treated as structure (walls), 0 disables.
        double maxExtentM = 5.0;
        double gateM = 1.5;
        double measSigmaM = 0.15;
        double accelSigma = 2.0;
        int persistWindow = 5;
        int persistHits = 3;
        double coastMs = 500.0;
    };

    struct Cluster {
        double x = 0, y = 0;
        double minX = 0, maxX = 0, minY = 0, maxY = 0;
        double a0 = 0, a1 = 0;       // bearing span, deg clockwise from forward
        double range = 0;            // centroid range, m
        double extent = 0;           // bounding-box diagonal, m
        int points = 0;
    };

    struct Track {
        int id = 0;
        double x = 0, y = 0, vx = 0, vy = 0;
        double ageMs = 0;
        int hits = 0;
        bool confirmed = false;
        int cls = Unknown;
        double clsConf = 0;
        bool fused = false;          // matched a camera box on the last scan
        Cluster last;
    };

    LidarPipeline() = default;
    explicit LidarPipeline(const Params &p) : m_p(p) {}

    // tMs is a monotonic timestamp of the scan. Returns the confirmed tracks.
    std::vector<Track> process(const QList<QPointF> &scan, double tMs,
                               const std::vector<CameraBox> &boxes = {});

    const std::vector<Cluster> &lastClusters() const { return m_clusters; }
    const std::vector<BoxResult> &lastBoxes() const { return m_boxes; }
    const std::vector<Track> &tracks() const { return m_tracks; }
    // Points that survived the range and body masks on the last scan.
    const QList<QPointF> &lastFiltered() const { return m_filtered; }
    const Params &params() const { return m_p; }

private:
    struct Axis { double p, v, pp, pv, vv; };
    struct Internal {
        Track pub;
        Axis ax, ay;
        double votes[ClassCount] = {};
        unsigned history = 0;
        int cluster = -1;
        double lastMs = 0;
    };

    std::vector<Cluster> cluster(const QList<QPointF> &pts) const;
    std::vector<BoxResult> toWedges(const std::vector<CameraBox> &boxes) const;
    // Per cluster: index into m_boxes or -1.
    std::vector<int> fuse();
    bool inBody(const QPointF &p) const;
    void predict(Axis &a, double dt) const;
    void update(Axis &a, double z) const;

    Params m_p;
    QList<QPointF> m_filtered;
    std::vector<Cluster> m_clusters;
    std::vector<BoxResult> m_boxes;
    std::vector<Internal> m_tracks_i;
    std::vector<Track> m_tracks;
    double m_lastMs = -1;
    int m_nextId = 1;
};
