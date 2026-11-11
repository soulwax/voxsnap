import 'dart:convert';
import 'dart:math' as math;
import 'dart:typed_data';

const double _kInvSqrt2 = 0.7071067811865475; // 1 / sqrt(2)

/// Fast, growable binary stream writer with varints, quantized vectors, and 32-bit quaternion packing.
class BinarySnapshotWriter {
  final BytesBuilder _bytes = BytesBuilder(copy: true);
  final ByteData _scratch = ByteData(8);

  int get length => _bytes.length;

  void writeBool(bool v) => _bytes.addByte(v ? 1 : 0);

  void writeUint8(int v) => _bytes.addByte(v & 0xFF);

  void writeUint16(int v) {
    _scratch.setUint16(0, v & 0xFFFF, Endian.little);
    _bytes.add(_scratch.buffer.asUint8List(0, 2));
  }

  void writeUint32(int v) {
    _scratch.setUint32(0, v & 0xFFFFFFFF, Endian.little);
    _bytes.add(_scratch.buffer.asUint8List(0, 4));
  }

  void writeInt32(int v) {
    _scratch.setInt32(0, v, Endian.little);
    _bytes.add(_scratch.buffer.asUint8List(0, 4));
  }

  void writeFloat32(double v) {
    _scratch.setFloat32(0, v, Endian.little);
    _bytes.add(_scratch.buffer.asUint8List(0, 4));
  }

  void writeFloat64(double v) {
    _scratch.setFloat64(0, v, Endian.little);
    _bytes.add(_scratch.buffer.asUint8List(0, 8));
  }

  /// Unsigned LEB128 variable-length integer (1 byte for values < 128).
  void writeVarUint(int value) {
    var v = value < 0 ? 0 : value;
    while (v >= 0x80) {
      _bytes.addByte((v & 0x7F) | 0x80);
      v >>= 7;
    }
    _bytes.addByte(v & 0x7F);
  }

  /// Signed LEB128 integer using ZigZag encoding.
  void writeVarInt(int value) {
    final zigzag = (value << 1) ^ (value >> 63);
    writeVarUint(zigzag);
  }

  void writeString(String text) {
    final encoded = utf8.encode(text);
    writeVarUint(encoded.length);
    _bytes.add(encoded);
  }

  void writeBytes(Uint8List data) {
    writeVarUint(data.length);
    _bytes.add(data);
  }

  /// Packs a bounded float into 16 bits within `[min, max]`.
  void writeQuantizedFloat16(double value, {double min = -1000.0, double max = 1000.0}) {
    final t = ((value - min) / math.max(1e-6, max - min)).clamp(0.0, 1.0);
    writeUint16((t * 65535.0).round());
  }

  /// Packs a 3D coordinate vector into 6 bytes using 16-bit uniform quantization per axis.
  void writeQuantizedVec3(
    double x,
    double y,
    double z, {
    double min = -1000.0,
    double max = 1000.0,
  }) {
    writeQuantizedFloat16(x, min: min, max: max);
    writeQuantizedFloat16(y, min: min, max: max);
    writeQuantizedFloat16(z, min: min, max: max);
  }

  /// Compresses a unit quaternion into 4 bytes (32 bits) via smallest-three projection.
  void writeSmallestThreeQuat(double x, double y, double z, double w) {
    final len = math.sqrt(x * x + y * y + z * z + w * w);
    final invLen = len > 1e-12 ? 1.0 / len : 1.0;
    final qx = len > 1e-12 ? x * invLen : 0.0;
    final qy = len > 1e-12 ? y * invLen : 0.0;
    final qz = len > 1e-12 ? z * invLen : 0.0;
    final qw = len > 1e-12 ? w * invLen : 1.0;

    final comps = [qx, qy, qz, qw];
    int maxIdx = 0;
    double maxAbs = comps[0].abs();
    for (int i = 1; i < 4; i++) {
      final a = comps[i].abs();
      if (a > maxAbs) {
        maxAbs = a;
        maxIdx = i;
      }
    }

    final sign = comps[maxIdx] < 0 ? -1.0 : 1.0;
    int packed = (maxIdx & 0x3) << 30;
    int shift = 20;

    for (int i = 0; i < 4; i++) {
      if (i == maxIdx) continue;
      final c = (comps[i] * sign).clamp(-_kInvSqrt2, _kInvSqrt2);
      final norm = (c / _kInvSqrt2) * 0.5 + 0.5;
      final q10 = (norm * 1023.0).round().clamp(0, 1023);
      packed |= (q10 << shift);
      shift -= 10;
    }

    writeUint32(packed);
  }

  Uint8List toBytes() => _bytes.takeBytes();
}

/// Companion reader deserializing compact binary snapshots written by [BinarySnapshotWriter].
class BinarySnapshotReader {
  final Uint8List _data;
  final ByteData _view;
  int _offset = 0;

  BinarySnapshotReader(Uint8List data)
      : _data = data,
        _view = ByteData.sublistView(data);

  int get offset => _offset;
  int get remaining => _data.length - _offset;
  bool get isAtEnd => _offset >= _data.length;

  bool readBool() => readUint8() != 0;

  int readUint8() {
    final v = _data[_offset];
    _offset += 1;
    return v;
  }

  int readUint16() {
    final v = _view.getUint16(_offset, Endian.little);
    _offset += 2;
    return v;
  }

  int readUint32() {
    final v = _view.getUint32(_offset, Endian.little);
    _offset += 4;
    return v;
  }

  int readInt32() {
    final v = _view.getInt32(_offset, Endian.little);
    _offset += 4;
    return v;
  }

  double readFloat32() {
    final v = _view.getFloat32(_offset, Endian.little);
    _offset += 4;
    return v;
  }

  double readFloat64() {
    final v = _view.getFloat64(_offset, Endian.little);
    _offset += 8;
    return v;
  }

  int readVarUint() {
    int result = 0;
    int shift = 0;
    while (true) {
      final b = _data[_offset++];
      result |= (b & 0x7F) << shift;
      if ((b & 0x80) == 0) break;
      shift += 7;
    }
    return result;
  }

  int readVarInt() {
    final zigzag = readVarUint();
    // Arithmetic right shift restores sign bit
    return (zigzag >> 1) ^ -(zigzag & 1);
  }

  String readString() {
    final len = readVarUint();
    final str = utf8.decode(_data.sublist(_offset, _offset + len));
    _offset += len;
    return str;
  }

  Uint8List readBytes() {
    final len = readVarUint();
    final slice = Uint8List.fromList(_data.sublist(_offset, _offset + len));
    _offset += len;
    return slice;
  }

  double readQuantizedFloat16({double min = -1000.0, double max = 1000.0}) {
    final q = readUint16();
    final t = q / 65535.0;
    return min + t * (max - min);
  }

  ({double x, double y, double z}) readQuantizedVec3({
    double min = -1000.0,
    double max = 1000.0,
  }) {
    final x = readQuantizedFloat16(min: min, max: max);
    final y = readQuantizedFloat16(min: min, max: max);
    final z = readQuantizedFloat16(min: min, max: max);
    return (x: x, y: y, z: z);
  }

  ({double x, double y, double z, double w}) readSmallestThreeQuat() {
    final packed = readUint32();
    final maxIdx = (packed >> 30) & 0x3;

    final out = [0.0, 0.0, 0.0, 0.0];
    double sumSq = 0.0;
    int shift = 20;

    for (int i = 0; i < 4; i++) {
      if (i == maxIdx) continue;
      final q10 = (packed >> shift) & 0x3FF;
      shift -= 10;
      final norm = q10 / 1023.0;
      final c = ((norm - 0.5) * 2.0) * _kInvSqrt2;
      out[i] = c;
      sumSq += c * c;
    }

    out[maxIdx] = math.sqrt(math.max(0.0, 1.0 - sumSq));
    return (x: out[0], y: out[1], z: out[2], w: out[3]);
  }
}

/// Bit-exact XOR + Zero-Run-Length delta compressor for sequential snapshot frames.
class BinaryDeltaCodec {
  /// Encodes [current] relative to [baseline]. Unchanged zero-runs compress heavily.
  static Uint8List encodeDelta(Uint8List baseline, Uint8List current) {
    final writer = BinarySnapshotWriter();
    writer.writeVarUint(current.length);
    final commonLen = math.min(baseline.length, current.length);

    int i = 0;
    while (i < commonLen) {
      final xorByte = baseline[i] ^ current[i];
      if (xorByte == 0) {
        int run = 0;
        while (i + run < commonLen && (baseline[i + run] ^ current[i + run]) == 0) {
          run++;
        }
        writer.writeUint8(0); // 0 marker = zero run
        writer.writeVarUint(run);
        i += run;
      } else {
        writer.writeUint8(xorByte);
        i++;
      }
    }

    // Write any trailing extra bytes if current grew past baseline
    if (current.length > commonLen) {
      writer.writeBytes(Uint8List.sublistView(current, commonLen));
    }

    return writer.toBytes();
  }

  /// Reconstructs the original [current] bytes from [baseline] and [delta].
  static Uint8List decodeDelta(Uint8List baseline, Uint8List delta) {
    final reader = BinarySnapshotReader(delta);
    final targetLen = reader.readVarUint();
    final out = Uint8List(targetLen);
    final commonLen = math.min(baseline.length, targetLen);

    int outIdx = 0;
    while (outIdx < commonLen) {
      final b = reader.readUint8();
      if (b == 0) {
        final run = reader.readVarUint();
        for (int k = 0; k < run && outIdx < commonLen; k++) {
          out[outIdx] = baseline[outIdx];
          outIdx++;
        }
      } else {
        out[outIdx] = baseline[outIdx] ^ b;
        outIdx++;
      }
    }

    if (targetLen > commonLen && !reader.isAtEnd) {
      final tail = reader.readBytes();
      out.setRange(commonLen, targetLen, tail);
    }

    return out;
  }
}
