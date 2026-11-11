import 'dart:math' as math;
import 'dart:typed_data';
import '../binary/binary_snapshot.dart';

/// Timestamped 3D transform keyframe used in ghost tracks and state snapshots.
class TransformSample {
  final double time;
  final double px, py, pz;
  final double qx, qy, qz, qw;
  final double sx, sy, sz;

  const TransformSample({
    required this.time,
    this.px = 0.0,
    this.py = 0.0,
    this.pz = 0.0,
    this.qx = 0.0,
    this.qy = 0.0,
    this.qz = 0.0,
    this.qw = 1.0,
    this.sx = 1.0,
    this.sy = 1.0,
    this.sz = 1.0,
  });

  /// Smoothly interpolates position, scale, and shortest-path normalized quaternion rotation.
  static TransformSample lerp(TransformSample a, TransformSample b, double t) {
    final ct = t.clamp(0.0, 1.0);
    final invT = 1.0 - ct;

    final px = a.px * invT + b.px * ct;
    final py = a.py * invT + b.py * ct;
    final pz = a.pz * invT + b.pz * ct;

    final sx = a.sx * invT + b.sx * ct;
    final sy = a.sy * invT + b.sy * ct;
    final sz = a.sz * invT + b.sz * ct;

    // Shortest-path quaternion NLERP
    final dot = a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw;
    final bSign = dot < 0.0 ? -1.0 : 1.0;

    var rx = a.qx * invT + (b.qx * bSign) * ct;
    var ry = a.qy * invT + (b.qy * bSign) * ct;
    var rz = a.qz * invT + (b.qz * bSign) * ct;
    var rw = a.qw * invT + (b.qw * bSign) * ct;

    final qLen = math.sqrt(rx * rx + ry * ry + rz * rz + rw * rw);
    if (qLen > 1e-12) {
      rx /= qLen;
      ry /= qLen;
      rz /= qLen;
      rw /= qLen;
    } else {
      rx = 0.0;
      ry = 0.0;
      rz = 0.0;
      rw = 1.0;
    }

    return TransformSample(
      time: a.time * invT + b.time * ct,
      px: px,
      py: py,
      pz: pz,
      qx: rx,
      qy: ry,
      qz: rz,
      qw: rw,
      sx: sx,
      sy: sy,
      sz: sz,
    );
  }
}

/// Recorded motion track supporting smooth binary-searched interpolation and compact binary export.
class GhostTrack {
  final List<TransformSample> _samples = [];

  GhostTrack();

  List<TransformSample> get samples => List.unmodifiable(_samples);
  int get length => _samples.length;
  bool get isEmpty => _samples.isEmpty;

  double get duration => _samples.length < 2 ? 0.0 : _samples.last.time - _samples.first.time;
  double get startTime => _samples.isEmpty ? 0.0 : _samples.first.time;
  double get endTime => _samples.isEmpty ? 0.0 : _samples.last.time;

  void addSample(TransformSample sample) {
    _samples.add(sample);
  }

  void clear() => _samples.clear();

  /// Evaluates smoothly interpolated transform at [time].
  TransformSample? evaluate(double time) {
    if (_samples.isEmpty) return null;
    if (_samples.length == 1) return _samples.first;

    if (time <= _samples.first.time) return _samples.first;
    if (time >= _samples.last.time) return _samples.last;

    int lo = 0;
    int hi = _samples.length - 1;
    while (lo < hi) {
      final mid = (lo + hi + 1) >> 1;
      if (_samples[mid].time <= time) {
        lo = mid;
      } else {
        hi = mid - 1;
      }
    }

    final a = _samples[lo];
    final b = _samples[lo + 1];
    final span = b.time - a.time;
    final factor = span > 1e-9 ? ((time - a.time) / span).clamp(0.0, 1.0) : 0.0;
    return TransformSample.lerp(a, b, factor);
  }

  /// Packs this entire ghost track into a compact binary byte stream using smallest-three quaternion packing.
  Uint8List encodeBinary() {
    final w = BinarySnapshotWriter();
    w.writeVarUint(_samples.length);

    for (final s in _samples) {
      final isUnitScale = (s.sx - 1.0).abs() < 1e-4 &&
          (s.sy - 1.0).abs() < 1e-4 &&
          (s.sz - 1.0).abs() < 1e-4;
      w.writeFloat32(s.time);
      w.writeFloat32(s.px);
      w.writeFloat32(s.py);
      w.writeFloat32(s.pz);
      w.writeSmallestThreeQuat(s.qx, s.qy, s.qz, s.qw);
      w.writeBool(!isUnitScale);
      if (!isUnitScale) {
        w.writeFloat32(s.sx);
        w.writeFloat32(s.sy);
        w.writeFloat32(s.sz);
      }
    }
    return w.toBytes();
  }

  /// Restores a [GhostTrack] serialized via [encodeBinary].
  factory GhostTrack.decodeBinary(Uint8List bytes) {
    final r = BinarySnapshotReader(bytes);
    final count = r.readVarUint();
    final track = GhostTrack();

    for (int i = 0; i < count; i++) {
      final time = r.readFloat32();
      final px = r.readFloat32();
      final py = r.readFloat32();
      final pz = r.readFloat32();
      final q = r.readSmallestThreeQuat();
      final hasScale = r.readBool();
      double sx = 1.0, sy = 1.0, sz = 1.0;
      if (hasScale) {
        sx = r.readFloat32();
        sy = r.readFloat32();
        sz = r.readFloat32();
      }
      track.addSample(TransformSample(
        time: time,
        px: px,
        py: py,
        pz: pz,
        qx: q.x,
        qy: q.y,
        qz: q.z,
        qw: q.w,
        sx: sx,
        sy: sy,
        sz: sz,
      ));
    }
    return track;
  }
}

/// Transport controller playing back a [GhostTrack].
class GhostPlayer {
  GhostTrack track;
  double currentTime = 0.0;
  double playbackSpeed;
  bool loop;
  bool isPlaying = false;

  GhostPlayer(
    this.track, {
    this.playbackSpeed = 1.0,
    this.loop = true,
  });

  void play() {
    if (!track.isEmpty && currentTime < track.startTime) {
      currentTime = track.startTime;
    }
    isPlaying = true;
  }

  void pause() => isPlaying = false;

  void stop() {
    isPlaying = false;
    currentTime = track.startTime;
  }

  void seek(double time) {
    currentTime = time;
  }

  TransformSample? advance(double dt) {
    if (track.isEmpty) return null;

    if (isPlaying) {
      currentTime += dt * playbackSpeed;
      if (currentTime > track.endTime) {
        if (loop && track.duration > 1e-6) {
          currentTime = track.startTime + ((currentTime - track.startTime) % track.duration);
        } else {
          currentTime = track.endTime;
          isPlaying = false;
        }
      } else if (currentTime < track.startTime) {
        if (loop && track.duration > 1e-6) {
          currentTime = track.endTime - ((track.startTime - currentTime) % track.duration);
        } else {
          currentTime = track.startTime;
          isPlaying = false;
        }
      }
    }

    return track.evaluate(currentTime);
  }
}
