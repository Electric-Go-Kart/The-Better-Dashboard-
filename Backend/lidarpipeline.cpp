#include "lidarpipeline.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {
constexpr double kRad2Deg = 57.29577951308232;

// Plausible visible lidar extent (bounding-box diagonal of the cluster) per class, metres.
struct Prior { double lo, hi; };
const Prior kPriors[LidarPipeline::ClassCount] = {
    {0.15, 1.0},   // person
    {0.2, 2.2},    // bike
    {0.8, 5.5},    // vehicle
    {0.05, 3.0},   // pole/barrier
    {0.08, 0.6},   // cone
};

double priorScore(double w, int cls)
{
    if (cls < 0 || cls >= LidarPipeline::ClassCount)
        return 0.0;
    const Prior &p = kPriors[cls];
    if (w < p.lo)
        return w / p.lo;
    if (w > p.hi)
        return p.hi / w;
    return 1.0;
}

int popcount(unsigned v, int bits)
{
    int n = 0;
    for (int i = 0; i < bits; ++i)
        n += (v >> i) & 1u;
    return n;
}
}

const char *LidarPipeline::className(int c)
{
    static const char *names[] = {"person", "bike", "vehicle", "pole/barrier", "cone"};
    return c >= 0 && c < ClassCount ? names[c] : "unknown";
}

int LidarPipeline::classFromName(const char *name)
{
    for (int c = 0; c < ClassCount; ++c)
        if (std::strcmp(name, className(c)) == 0)
            return c;
    return Unknown;
}

bool LidarPipeline::inBody(const QPointF &p) const
{
    return p.x() < m_p.bodyFrontM + m_p.bodyMarginM && p.x() > -(m_p.bodyRearM + m_p.bodyMarginM)
        && std::abs(p.y()) < m_p.bodyHalfWidthM + m_p.bodyMarginM;
}

std::vector<LidarPipeline::Cluster> LidarPipeline::cluster(const QList<QPointF> &pts) const
{
    std::vector<std::vector<QPointF>> groups;
    auto gapOk = [this](const QPointF &a, const QPointF &b) {
        const double r = std::min(std::hypot(a.x(), a.y()), std::hypot(b.x(), b.y()));
        return std::hypot(a.x() - b.x(), a.y() - b.y()) < m_p.gapBaseM + m_p.gapPerRangeM * r;
    };
    for (int i = 0; i < pts.size(); ++i) {
        if (i == 0 || !gapOk(pts[i - 1], pts[i]))
            groups.emplace_back();
        groups.back().push_back(pts[i]);
    }
    // The scan is circular: the last cluster may continue into the first.
    if (groups.size() > 1 && gapOk(pts.last(), pts.first())) {
        groups.front().insert(groups.front().begin(), groups.back().begin(), groups.back().end());
        groups.pop_back();
    }

    std::vector<Cluster> out;
    for (const auto &g : groups) {
        if (int(g.size()) < m_p.minClusterPoints)
            continue;
        Cluster c;
        c.points = int(g.size());
        c.minX = c.minY = 1e9;
        c.maxX = c.maxY = -1e9;
        c.a0 = 1e9;
        c.a1 = -1e9;
        for (const QPointF &p : g) {
            const double a = std::atan2(p.y(), p.x()) * kRad2Deg;
            c.a0 = std::min(c.a0, a);
            c.a1 = std::max(c.a1, a);
            c.x += p.x();
            c.y += p.y();
            c.minX = std::min(c.minX, p.x());
            c.maxX = std::max(c.maxX, p.x());
            c.minY = std::min(c.minY, p.y());
            c.maxY = std::max(c.maxY, p.y());
        }
        c.x /= c.points;
        c.y /= c.points;
        c.range = std::hypot(c.x, c.y);
        c.extent = std::hypot(c.maxX - c.minX, c.maxY - c.minY);
        if (m_p.maxExtentM > 0 && c.extent > m_p.maxExtentM)
            continue;
        out.push_back(c);
    }
    return out;
}

std::vector<LidarPipeline::BoxResult> LidarPipeline::toWedges(const std::vector<CameraBox> &boxes) const
{
    struct Item { BoxResult b; bool tl, tr; };
    std::vector<Item> items;
    for (const CameraBox &cb : boxes) {
        if (cb.cam < 0 || cb.cam >= int(m_p.cameras.size()) || cb.cls < 0 || cb.cls >= ClassCount)
            continue;
        const Camera &cam = m_p.cameras[cb.cam];
        const double cx = cam.widthPx / 2.0;
        const double fx = cx / std::tan(cam.hfovDeg / 2.0 / kRad2Deg);
        const double u0 = std::min(cb.u0, cb.u1), u1 = std::max(cb.u0, cb.u1);
        Item it;
        it.b.a0 = cam.yawDeg + std::atan((u0 - cx) / fx) * kRad2Deg;
        it.b.a1 = cam.yawDeg + std::atan((u1 - cx) / fx) * kRad2Deg;
        it.b.cam = cb.cam;
        it.b.cls = cb.cls;
        it.b.conf = cb.conf;
        it.tl = u0 <= m_p.edgePx;
        it.tr = u1 >= cam.widthPx - m_p.edgePx;
        it.b.truncated = it.tl || it.tr;
        items.push_back(it);
    }
    std::sort(items.begin(), items.end(), [](const Item &a, const Item &b) { return a.b.a0 < b.b.a0; });

    std::vector<Item> merged;
    for (const Item &it : items) {
        if (!merged.empty()) {
            Item &cur = merged.back();
            // Only the cut edges at a camera seam may join: left box cut on its right, right box on its left.
            if (cur.b.cam != it.b.cam && cur.b.cls == it.b.cls && cur.tr && it.tl
                && it.b.a0 <= cur.b.a1 + m_p.seamGapDeg) {
                cur.b.a1 = std::max(cur.b.a1, it.b.a1);
                cur.b.conf = std::max(cur.b.conf, it.b.conf);
                cur.b.cam = it.b.cam;
                cur.tr = it.tr;
                cur.b.truncated = cur.tl || cur.tr;
                continue;
            }
        }
        merged.push_back(it);
    }
    std::vector<BoxResult> out;
    for (const Item &it : merged)
        out.push_back(it.b);
    return out;
}

std::vector<int> LidarPipeline::fuse()
{
    struct Pair { double match; int box, cluster; };
    std::vector<Pair> pairs;
    for (size_t bi = 0; bi < m_boxes.size(); ++bi) {
        const BoxResult &b = m_boxes[bi];
        for (size_t ci = 0; ci < m_clusters.size(); ++ci) {
            const Cluster &c = m_clusters[ci];
            const double c0 = c.a0 - m_p.wedgePadDeg, c1 = c.a1 + m_p.wedgePadDeg;
            const double inter = std::min(c1, b.a1) - std::max(c0, b.a0);
            if (inter <= 0)
                continue;
            const double uni = std::max(c1, b.a1) - std::min(c0, b.a0);
            const double ovl = inter / uni;
            if (ovl < m_p.minOverlap)
                continue;
            const double sizeScore = priorScore(c.extent, b.cls);
            // Width the box would have at the cluster's range; meaningless if the image cut the box.
            const double boxWidth = 2.0 * c.range * std::tan((b.a1 - b.a0) / 2.0 / kRad2Deg);
            const double boxScore = b.truncated ? 1.0 : priorScore(boxWidth, b.cls);
            const double match = ovl * std::sqrt(sizeScore * boxScore);
            if (match >= m_p.minMatch)
                pairs.push_back({match, int(bi), int(ci)});
        }
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair &a, const Pair &b) { return a.match > b.match; });
    std::vector<int> clusterBox(m_clusters.size(), -1);
    for (const Pair &p : pairs) {
        if (m_boxes[p.box].cluster >= 0 || clusterBox[p.cluster] >= 0)
            continue;
        m_boxes[p.box].cluster = p.cluster;
        m_boxes[p.box].match = p.match;
        clusterBox[p.cluster] = p.box;
    }
    return clusterBox;
}

void LidarPipeline::predict(Axis &a, double dt) const
{
    const double q = m_p.accelSigma * m_p.accelSigma;
    a.p += a.v * dt;
    const double pp = a.pp + dt * (a.pv + a.pv) + dt * dt * a.vv + q * dt * dt * dt * dt / 4;
    const double pv = a.pv + dt * a.vv + q * dt * dt * dt / 2;
    const double vv = a.vv + q * dt * dt;
    a.pp = pp;
    a.pv = pv;
    a.vv = vv;
}

void LidarPipeline::update(Axis &a, double z) const
{
    const double r = m_p.measSigmaM * m_p.measSigmaM;
    const double s = a.pp + r;
    const double kp = a.pp / s, kv = a.pv / s;
    const double innov = z - a.p;
    a.p += kp * innov;
    a.v += kv * innov;
    const double pp = (1 - kp) * a.pp;
    const double pv = (1 - kp) * a.pv;
    const double vv = a.vv - kv * a.pv;
    a.pp = pp;
    a.pv = pv;
    a.vv = vv;
}

std::vector<LidarPipeline::Track> LidarPipeline::process(const QList<QPointF> &scan, double tMs,
                                                         const std::vector<CameraBox> &boxes)
{
    m_filtered.clear();
    const double r2 = m_p.rangeM * m_p.rangeM;
    for (const QPointF &p : scan) {
        if (p.x() * p.x() + p.y() * p.y() > r2 || inBody(p))
            continue;
        m_filtered.append(p);
    }
    m_clusters = cluster(m_filtered);
    m_boxes = toWedges(boxes);
    const std::vector<int> clusterBox = fuse();

    const double dt = m_lastMs < 0 ? 0.0 : std::max(0.0, (tMs - m_lastMs) / 1000.0);
    m_lastMs = tMs;
    for (Internal &t : m_tracks_i) {
        predict(t.ax, dt);
        predict(t.ay, dt);
        t.history <<= 1;
        t.cluster = -1;
    }

    // Greedy nearest-neighbour association, closest pairs first.
    struct Pair { double d; size_t t, c; };
    std::vector<Pair> pairs;
    for (size_t ti = 0; ti < m_tracks_i.size(); ++ti)
        for (size_t ci = 0; ci < m_clusters.size(); ++ci) {
            const double d = std::hypot(m_tracks_i[ti].ax.p - m_clusters[ci].x, m_tracks_i[ti].ay.p - m_clusters[ci].y);
            if (d < m_p.gateM)
                pairs.push_back({d, ti, ci});
        }
    std::sort(pairs.begin(), pairs.end(), [](const Pair &a, const Pair &b) { return a.d < b.d; });
    std::vector<bool> trackUsed(m_tracks_i.size(), false), clusterUsed(m_clusters.size(), false);
    for (const Pair &p : pairs) {
        if (trackUsed[p.t] || clusterUsed[p.c])
            continue;
        trackUsed[p.t] = clusterUsed[p.c] = true;
        Internal &t = m_tracks_i[p.t];
        update(t.ax, m_clusters[p.c].x);
        update(t.ay, m_clusters[p.c].y);
        t.history |= 1u;
        t.lastMs = tMs;
        t.pub.last = m_clusters[p.c];
        t.cluster = int(p.c);
        ++t.pub.hits;
    }
    for (size_t ci = 0; ci < m_clusters.size(); ++ci) {
        if (clusterUsed[ci])
            continue;
        Internal t;
        t.pub.id = m_nextId++;
        t.pub.last = m_clusters[ci];
        t.cluster = int(ci);
        t.pub.hits = 1;
        t.ax = {m_clusters[ci].x, 0, 1.0, 0, 25.0};
        t.ay = {m_clusters[ci].y, 0, 1.0, 0, 25.0};
        t.history = 1u;
        t.lastMs = tMs;
        m_tracks_i.push_back(t);
    }

    m_tracks_i.erase(std::remove_if(m_tracks_i.begin(), m_tracks_i.end(), [&](const Internal &t) {
        return tMs - t.lastMs > m_p.coastMs
            || (!t.pub.confirmed && popcount(t.history, m_p.persistWindow) == 0);
    }), m_tracks_i.end());

    m_tracks.clear();
    std::vector<Track> confirmed;
    for (Internal &t : m_tracks_i) {
        t.pub.fused = false;
        if (t.cluster >= 0 && clusterBox[t.cluster] >= 0) {
            const BoxResult &b = m_boxes[clusterBox[t.cluster]];
            // Votes only change when a camera sees the track, so the class sticks while it is out of view.
            for (double &v : t.votes)
                v *= m_p.voteDecay;
            t.votes[b.cls] = std::min(3.0, t.votes[b.cls] + std::sqrt(b.conf * b.match));
            t.pub.fused = true;
        }
        int best = Unknown;
        double bestVote = m_p.minClassVote;
        for (int c = 0; c < ClassCount; ++c)
            if (t.votes[c] >= bestVote) {
                best = c;
                bestVote = t.votes[c];
            }
        t.pub.cls = best;
        t.pub.clsConf = best == Unknown ? 0.0 : std::min(1.0, bestVote);
        const unsigned mask = (1u << m_p.persistWindow) - 1u;
        if (!t.pub.confirmed && popcount(t.history & mask, m_p.persistWindow) >= m_p.persistHits)
            t.pub.confirmed = true;
        t.pub.x = t.ax.p;
        t.pub.y = t.ay.p;
        t.pub.vx = t.ax.v;
        t.pub.vy = t.ay.v;
        t.pub.ageMs = tMs - t.lastMs;
        m_tracks.push_back(t.pub);
        if (t.pub.confirmed)
            confirmed.push_back(t.pub);
    }
    return confirmed;
}
