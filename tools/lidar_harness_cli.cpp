// Headless driver for the real lidar code. stdin is a sequence of frames: 1 type byte, u32 little-endian
// length, payload.
//   type 0: raw STL-27L bytes, fed to the production Stl27lReader (framing, CRC, angle decode, masks)
//   type 1: JSON {"scan":N,"boxes":[{"cam":0,"u0":..,"u1":..,"cls":"person","conf":0.9}]}, camera boxes
//           for revolution N, converted and matched by LidarPipeline
// Each revolution goes through LidarPipeline (range/body mask, clustering, camera fusion, persistence,
// tracker) and one JSON line is printed on stdout. Driven by tools/lidar_harness.py.
#include <QCoreApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <map>
#include <QStringList>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include "Backend/lidarpipeline.h"
#include "Backend/stl27lreader.h"

namespace {
void usage()
{
    std::fputs("lidar_harness_cli [--yaw DEG] [--mirrored] [--min-m M] [--max-m M] [--mask FROM,TO]...\n"
               "  [--body FRONT,REAR,HALFWIDTH] [--range M] [--period-ms MS] [--set key=value]...\n"
               "  [--camera YAW_DEG,HFOV_DEG,WIDTH_PX]...\n"
               "stdin: framed packets and camera boxes. stdout: one JSON object per revolution.\n", stderr);
}

void printCluster(const LidarPipeline::Cluster &c)
{
    std::printf("{\"x\":%.3f,\"y\":%.3f,\"x0\":%.3f,\"x1\":%.3f,\"y0\":%.3f,\"y1\":%.3f,\"n\":%d}",
                c.x, c.y, c.minX, c.maxX, c.minY, c.maxY, c.points);
}
}

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    Stl27lReader reader;
    LidarPipeline::Params params;
    double periodMs = 100.0;
    double minM = 0.03, maxM = 25.0;

    for (int i = 1; i < argc; ++i) {
        const QString a = QString::fromLocal8Bit(argv[i]);
        auto next = [&]() -> QString {
            if (i + 1 >= argc) {
                usage();
                std::exit(2);
            }
            return QString::fromLocal8Bit(argv[++i]);
        };
        if (a == "--yaw") {
            reader.setMountYawDeg(next().toDouble());
        } else if (a == "--mirrored") {
            reader.setMirrored(true);
        } else if (a == "--min-m") {
            minM = next().toDouble();
        } else if (a == "--max-m") {
            maxM = next().toDouble();
        } else if (a == "--mask") {
            const QStringList v = next().split(',');
            if (v.size() == 2)
                reader.addMaskDeg(v[0].toDouble(), v[1].toDouble());
        } else if (a == "--body") {
            const QStringList v = next().split(',');
            if (v.size() == 3) {
                params.bodyFrontM = v[0].toDouble();
                params.bodyRearM = v[1].toDouble();
                params.bodyHalfWidthM = v[2].toDouble();
            }
        } else if (a == "--camera") {
            const QStringList v = next().split(',');
            if (v.size() == 3) {
                LidarPipeline::Camera c;
                c.yawDeg = v[0].toDouble();
                c.hfovDeg = v[1].toDouble();
                c.widthPx = v[2].toInt();
                params.cameras.push_back(c);
            }
        } else if (a == "--range") {
            params.rangeM = next().toDouble();
        } else if (a == "--period-ms") {
            periodMs = next().toDouble();
        } else if (a == "--set") {
            const QStringList kv = next().split('=');
            if (kv.size() != 2)
                continue;
            const double v = kv[1].toDouble();
            const QString k = kv[0];
            if (k == "gapBaseM") params.gapBaseM = v;
            else if (k == "gapPerRangeM") params.gapPerRangeM = v;
            else if (k == "minClusterPoints") params.minClusterPoints = int(v);
            else if (k == "maxExtentM") params.maxExtentM = v;
            else if (k == "gateM") params.gateM = v;
            else if (k == "measSigmaM") params.measSigmaM = v;
            else if (k == "accelSigma") params.accelSigma = v;
            else if (k == "persistWindow") params.persistWindow = int(v);
            else if (k == "persistHits") params.persistHits = int(v);
            else if (k == "coastMs") params.coastMs = v;
            else if (k == "bodyMarginM") params.bodyMarginM = v;
            else if (k == "edgePx") params.edgePx = v;
            else if (k == "seamGapDeg") params.seamGapDeg = v;
            else if (k == "wedgePadDeg") params.wedgePadDeg = v;
            else if (k == "minOverlap") params.minOverlap = v;
            else if (k == "minMatch") params.minMatch = v;
            else if (k == "voteDecay") params.voteDecay = v;
            else if (k == "minClassVote") params.minClassVote = v;
            else {
                std::fprintf(stderr, "unknown parameter %s\n", qPrintable(k));
                return 2;
            }
        } else {
            usage();
            return 2;
        }
    }

    reader.setRangeLimitsM(minM, maxM);
    LidarPipeline pipeline(params);
    long scanIndex = 0;
    std::map<long, std::vector<LidarPipeline::CameraBox>> pendingBoxes;
    QObject::connect(&reader, &Stl27lReader::scanReady, [&](const QList<QPointF> &scan) {
        const double t = scanIndex++ * periodMs;
        std::vector<LidarPipeline::CameraBox> boxes;
        auto it = pendingBoxes.find(scanIndex - 1);
        if (it != pendingBoxes.end()) {
            boxes = it->second;
            pendingBoxes.erase(pendingBoxes.begin(), std::next(it));
        }
        const auto confirmed = pipeline.process(scan, t, boxes);
        std::printf("{\"scan\":%ld,\"t\":%.1f,\"raw\":%d,\"points\":[", scanIndex - 1, t, int(scan.size()));
        const QList<QPointF> &pts = pipeline.lastFiltered();
        for (int i = 0; i < pts.size(); ++i)
            std::printf("%s[%.3f,%.3f]", i ? "," : "", pts[i].x(), pts[i].y());
        std::printf("],\"clusters\":[");
        const auto &cl = pipeline.lastClusters();
        for (size_t i = 0; i < cl.size(); ++i) {
            if (i)
                std::printf(",");
            printCluster(cl[i]);
        }
        std::printf("],\"boxes\":[");
        const auto &bx = pipeline.lastBoxes();
        for (size_t i = 0; i < bx.size(); ++i)
            std::printf("%s{\"a0\":%.2f,\"a1\":%.2f,\"cam\":%d,\"cls\":\"%s\",\"conf\":%.2f,\"trunc\":%d,\"cluster\":%d,\"match\":%.2f}",
                        i ? "," : "", bx[i].a0, bx[i].a1, bx[i].cam, LidarPipeline::className(bx[i].cls),
                        bx[i].conf, bx[i].truncated ? 1 : 0, bx[i].cluster, bx[i].match);
        std::printf("],\"tracks\":[");
        for (size_t i = 0; i < confirmed.size(); ++i) {
            const auto &k = confirmed[i];
            std::printf("%s{\"id\":%d,\"x\":%.3f,\"y\":%.3f,\"vx\":%.3f,\"vy\":%.3f,\"age\":%.0f,\"hits\":%d,\"n\":%d,\"w\":%.2f,\"l\":%.2f,\"cls\":\"%s\",\"cconf\":%.2f,\"src\":\"%s\"}",
                        i ? "," : "", k.id, k.x, k.y, k.vx, k.vy, k.ageMs, k.hits, k.last.points,
                        k.last.maxY - k.last.minY, k.last.maxX - k.last.minX, LidarPipeline::className(k.cls),
                        k.clsConf, k.fused ? "fused" : "lidar");
        }
        std::printf("],\"good\":%lld,\"bad\":%lld}\n", reader.goodPackets(), reader.badPackets());
        std::fflush(stdout);
    });

    auto readExact = [](char *dst, size_t n) {
        size_t got = 0;
        while (got < n) {
            const ssize_t r = ::read(0, dst + got, n - got);
            if (r <= 0)
                return false;
            got += size_t(r);
        }
        return true;
    };
    std::vector<char> payload;
    for (;;) {
        unsigned char head[5];
        if (!readExact(reinterpret_cast<char *>(head), 5))
            break;
        const quint32 len = head[1] | (head[2] << 8) | (head[3] << 16) | (quint32(head[4]) << 24);
        payload.resize(len);
        if (len && !readExact(payload.data(), len))
            break;
        if (head[0] == 0) {
            reader.feed(QByteArray(payload.data(), int(len)));
        } else if (head[0] == 1) {
            const QJsonObject o = QJsonDocument::fromJson(QByteArray(payload.data(), int(len))).object();
            std::vector<LidarPipeline::CameraBox> list;
            for (const QJsonValue &v : o["boxes"].toArray()) {
                const QJsonObject b = v.toObject();
                LidarPipeline::CameraBox cb;
                cb.cam = b["cam"].toInt();
                cb.u0 = b["u0"].toDouble();
                cb.u1 = b["u1"].toDouble();
                cb.cls = LidarPipeline::classFromName(b["cls"].toString().toUtf8().constData());
                cb.conf = b["conf"].toDouble();
                list.push_back(cb);
            }
            pendingBoxes[long(o["scan"].toInteger())] = list;
        }
    }
    return 0;
}
