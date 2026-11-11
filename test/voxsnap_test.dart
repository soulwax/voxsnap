import 'dart:math' as math;
import 'dart:typed_data';
import 'package:test/test.dart';
import 'package:voxsnap/voxsnap.dart';

void main() {
  group('BinarySnapshotWriter, Reader & Compression', () {
    test('serializes varints, strings, quantized vec3, and smallest-three quaternions', () {
      final w = BinarySnapshotWriter();
      w.writeVarUint(42);
      w.writeVarUint(123456);
      w.writeVarInt(-9876);
      w.writeString('Checkpoint_Alpha');
      w.writeQuantizedVec3(125.5, -42.25, 899.75);

      // Arbitrary 3D rotation quaternion
      final angle = 1.1;
      final axis = [0.26726, 0.53452, 0.80178];
      final sinHalf = math.sin(angle * 0.5);
      final qx = axis[0] * sinHalf;
      final qy = axis[1] * sinHalf;
      final qz = axis[2] * sinHalf;
      final qw = math.cos(angle * 0.5);
      w.writeSmallestThreeQuat(qx, qy, qz, qw);

      final bytes = w.toBytes();
      final r = BinarySnapshotReader(bytes);

      expect(r.readVarUint(), equals(42));
      expect(r.readVarUint(), equals(123456));
      expect(r.readVarInt(), equals(-9876));
      expect(r.readString(), equals('Checkpoint_Alpha'));

      final vec = r.readQuantizedVec3();
      expect(vec.x, closeTo(125.5, 0.05));
      expect(vec.y, closeTo(-42.25, 0.05));
      expect(vec.z, closeTo(899.75, 0.05));

      final rq = r.readSmallestThreeQuat();
      // Compare |dot(q, rq)| which must be ~1.0 (handles q == -q equivalence)
      final dot = (qx * rq.x + qy * rq.y + qz * rq.z + qw * rq.w).abs();
      expect(dot, greaterThan(0.9999));
      expect(r.isAtEnd, isTrue);
    });

    test('BinaryDeltaCodec heavily compresses sparse updates bit-identically', () {
      final base = Uint8List.fromList(List.generate(256, (i) => i % 251));
      final updated = Uint8List.fromList(base);
      // Mutate only 3 bytes in the middle
      updated[60] = 0xAA;
      updated[61] = 0xBB;
      updated[200] = 0xCC;

      final delta = BinaryDeltaCodec.encodeDelta(base, updated);
      // Must compress way smaller than 256 bytes!
      expect(delta.length, lessThan(24));

      final reconstructed = BinaryDeltaCodec.decodeDelta(base, delta);
      expect(reconstructed, equals(updated));
    });
  });

  group('RewindBuffer', () {
    test('maintains bounded circular capacity and supports time interpolation & branching', () {
      final buf = RewindBuffer<double>(capacity: 5);

      for (int i = 0; i < 8; i++) {
        buf.push(i * 1.0, i * 10.0);
      }

      // Capacity 5 -> oldest is tick 3 (time 3.0, val 30.0), latest is tick 7 (time 7.0, val 70.0)
      expect(buf.length, equals(5));
      expect(buf.oldest!.time, equals(3.0));
      expect(buf.latest!.time, equals(7.0));

      // Sample midway at time 4.25
      final sample = buf.sampleInterpolated(4.25);
      expect(sample, isNotNull);
      expect(sample!.a.state, equals(40.0));
      expect(sample.b.state, equals(50.0));
      expect(sample.t, closeTo(0.25, 1e-4));

      // Truncate future after 5.0
      buf.truncateAfter(5.0);
      expect(buf.length, equals(3)); // times 3, 4, 5
      expect(buf.latest!.time, equals(5.0));
    });
  });

  group('GhostTrack & GhostPlayer', () {
    test('evaluates smooth interpolated transforms and round-trips compact binary', () {
      final track = GhostTrack();
      track.addSample(const TransformSample(time: 0.0, px: 0.0, py: 0.0, pz: 0.0));
      track.addSample(TransformSample(
        time: 2.0,
        px: 20.0,
        py: 10.0,
        pz: -4.0,
        qy: math.sin(0.5),
        qw: math.cos(0.5),
      ));

      final mid = track.evaluate(1.0)!;
      expect(mid.px, closeTo(10.0, 1e-4));
      expect(mid.py, closeTo(5.0, 1e-4));
      expect(mid.pz, closeTo(-2.0, 1e-4));

      // Compact binary encode -> decode
      final bin = track.encodeBinary();
      // 2 samples with unit scale -> ~38 bytes total
      expect(bin.length, lessThan(48));

      final decodedTrack = GhostTrack.decodeBinary(bin);
      expect(decodedTrack.length, equals(2));

      final player = GhostPlayer(decodedTrack, loop: true)..play();
      final p1 = player.advance(1.0)!;
      expect(p1.px, closeTo(10.0, 1e-3));

      // Wrap around at end -> time 2.5 wraps to 0.5 -> px = 5.0
      final pWrap = player.advance(1.5)!;
      expect(pWrap.px, closeTo(5.0, 1e-3));
    });
  });
}
