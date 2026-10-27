#include "stl27lreader.h"

#include <QDebug>
#include <QtEndian>
#include <QtMath>

namespace {
constexpr quint8 kHeader = 0x54;
constexpr quint8 kVerLen = 0x2C;
constexpr int kPointsPerPacket = 12;
constexpr int kPacketSize = 47;

const quint8 kCrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8};
}

Stl27lReader::Stl27lReader(QObject *parent)
    : QObject(parent)
{
    m_retryTimer.setInterval(2000);
    m_retryTimer.setSingleShot(true);
    connect(&m_retryTimer, &QTimer::timeout, this, &Stl27lReader::reopen);
    connect(&m_port, &QSerialPort::readyRead, this, &Stl27lReader::readSerial);
    connect(&m_port, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError e) {
        if (e != QSerialPort::NoError)
            handleError();
    });
}

quint8 Stl27lReader::crc8(const quint8 *data, int len)
{
    quint8 crc = 0;
    while (len--)
        crc = kCrcTable[(crc ^ *data++) & 0xff];
    return crc;
}

bool Stl27lReader::open(const QString &device, qint32 baud)
{
    m_device = device;
    m_baud = baud;
    m_port.setPortName(device);
    m_port.setBaudRate(baud);
    m_port.setDataBits(QSerialPort::Data8);
    m_port.setParity(QSerialPort::NoParity);
    m_port.setStopBits(QSerialPort::OneStop);
    m_port.setFlowControl(QSerialPort::NoFlowControl);
    if (!m_port.open(QIODevice::ReadOnly)) {
        qWarning() << "STL-27L: cannot open" << device << m_port.errorString() << "- retrying";
        m_retryTimer.start();
        return false;
    }
    qInfo() << "STL-27L: reading" << device << "at" << baud << "baud";
    return true;
}

void Stl27lReader::reopen()
{
    if (!m_port.isOpen())
        open(m_device, m_baud);
}

void Stl27lReader::handleError()
{
    if (m_port.isOpen())
        m_port.close();
    m_buffer.clear();
    m_scan.clear();
    m_lastDeg = -1;
    if (!m_retryTimer.isActive())
        m_retryTimer.start();
}

void Stl27lReader::readSerial()
{
    feed(m_port.readAll());
}

void Stl27lReader::feed(const QByteArray &bytes)
{
    m_buffer.append(bytes);
    const quint8 *raw = reinterpret_cast<const quint8 *>(m_buffer.constData());
    int pos = 0;
    const int size = m_buffer.size();

    while (size - pos >= kPacketSize) {
        if (raw[pos] != kHeader || raw[pos + 1] != kVerLen) {
            ++pos;
            continue;
        }
        if (crc8(raw + pos, kPacketSize - 1) != raw[pos + kPacketSize - 1]) {
            ++m_bad;
            ++pos;
            continue;
        }
        decodePacket(raw + pos);
        ++m_good;
        pos += kPacketSize;
    }
    m_buffer.remove(0, pos);

    // Never let a stream with no valid packets grow the buffer without bound.
    if (m_buffer.size() > 4 * kPacketSize)
        m_buffer.remove(0, m_buffer.size() - kPacketSize);

    if ((m_good + m_bad) % 200 == 0)
        emit statsChanged();
}

bool Stl27lReader::masked(double sensorDeg) const
{
    for (const Mask &m : m_masks) {
        const bool inside = m.from <= m.to ? (sensorDeg >= m.from && sensorDeg <= m.to)
                                           : (sensorDeg >= m.from || sensorDeg <= m.to);
        if (inside)
            return true;
    }
    return false;
}

void Stl27lReader::decodePacket(const quint8 *p)
{
    const quint16 startRaw = qFromLittleEndian<quint16>(p + 4);
    const quint16 endRaw = qFromLittleEndian<quint16>(p + 4 + 2 + kPointsPerPacket * 3);
    const quint32 span = (quint32(endRaw) + 36000 - startRaw) % 36000;
    const double step = span / double(kPointsPerPacket - 1) / 100.0;
    const double start = startRaw / 100.0;

    // The angle wrapping past 360 deg marks the start of a new revolution.
    if (m_lastDeg > 300.0 && start < 60.0) {
        if (!m_scan.isEmpty())
            emit scanReady(m_scan);
        m_scan.clear();
    }
    m_lastDeg = start;

    for (int i = 0; i < kPointsPerPacket; ++i) {
        const quint16 mm = qFromLittleEndian<quint16>(p + 6 + i * 3);
        if (mm == 0)
            continue;
        const double r = mm / 1000.0;
        if (r < m_minM || r > m_maxM)
            continue;
        double deg = std::fmod(start + i * step, 360.0);
        if (masked(deg))
            continue;
        if (m_mirrored)
            deg = -deg;
        const double rad = qDegreesToRadians(deg + m_yawDeg);
        m_scan.append(QPointF(r * std::cos(rad), r * std::sin(rad)));
    }
}
