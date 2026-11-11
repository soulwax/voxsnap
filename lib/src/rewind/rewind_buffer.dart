/// Single chronological entry stored inside a [RewindBuffer].
class SnapshotFrame<T> {
  final int tick;
  final double time;
  final T state;

  const SnapshotFrame({
    required this.tick,
    required this.time,
    required this.state,
  });
}

/// Bounded O(1) circular ring buffer recording historical snapshots for time-rewind and replay scrubbing.
class RewindBuffer<T> {
  final int capacity;
  final List<SnapshotFrame<T>?> _buffer;

  int _head = 0; // Index of oldest element
  int _count = 0;
  int _nextTick = 0;

  RewindBuffer({this.capacity = 600})
      : assert(capacity > 0),
        _buffer = List<SnapshotFrame<T>?>.filled(capacity, null, growable: false);

  int get length => _count;
  bool get isEmpty => _count == 0;
  bool get isNotEmpty => _count > 0;
  bool get isFull => _count == capacity;

  SnapshotFrame<T>? get oldest => _count == 0 ? null : _at(0);
  SnapshotFrame<T>? get latest => _count == 0 ? null : _at(_count - 1);

  double get totalDuration {
    if (_count < 2) return 0.0;
    return latest!.time - oldest!.time;
  }

  SnapshotFrame<T> _at(int logicalIndex) {
    final phys = (_head + logicalIndex) % capacity;
    return _buffer[phys]!;
  }

  /// Appends a new snapshot at [time]. Overwrites the oldest entry cleanly when full.
  void push(double time, T state, {int? tick}) {
    final frame = SnapshotFrame<T>(
      tick: tick ?? _nextTick++,
      time: time,
      state: state,
    );

    if (_count < capacity) {
      final idx = (_head + _count) % capacity;
      _buffer[idx] = frame;
      _count++;
    } else {
      _buffer[_head] = frame;
      _head = (_head + 1) % capacity;
    }
  }

  /// Pops and returns the most recent snapshot (useful when stepping backward frame by frame).
  SnapshotFrame<T>? popLatest() {
    if (_count == 0) return null;
    final idx = (_head + _count - 1) % capacity;
    final frame = _buffer[idx];
    _buffer[idx] = null;
    _count--;
    return frame;
  }

  /// Truncates all snapshots strictly newer than [cutTime] (used when branching a new timeline after rewind).
  void truncateAfter(double cutTime) {
    while (_count > 0 && latest!.time > cutTime) {
      popLatest();
    }
  }

  /// Clears all recorded history.
  void clear() {
    for (int i = 0; i < capacity; i++) {
      _buffer[i] = null;
    }
    _head = 0;
    _count = 0;
    _nextTick = 0;
  }

  /// Finds bracketing frames `(a, b)` and linear factor `t` in `[0, 1]` around [targetTime].
  ({SnapshotFrame<T> a, SnapshotFrame<T> b, double t})? sampleInterpolated(double targetTime) {
    if (_count == 0) return null;
    if (_count == 1) {
      final only = _at(0);
      return (a: only, b: only, t: 0.0);
    }

    if (targetTime <= oldest!.time) {
      final first = _at(0);
      return (a: first, b: first, t: 0.0);
    }
    if (targetTime >= latest!.time) {
      final last = _at(_count - 1);
      return (a: last, b: last, t: 0.0);
    }

    // Binary search for largest index i where _at(i).time <= targetTime
    int lo = 0;
    int hi = _count - 1;
    while (lo < hi) {
      final mid = (lo + hi + 1) >> 1;
      if (_at(mid).time <= targetTime) {
        lo = mid;
      } else {
        hi = mid - 1;
      }
    }

    final a = _at(lo);
    final b = _at(lo + 1);
    final span = b.time - a.time;
    final factor = span > 1e-9 ? ((targetTime - a.time) / span).clamp(0.0, 1.0) : 0.0;
    return (a: a, b: b, t: factor);
  }

  /// Returns nearest recorded frame at or prior to [targetTime].
  SnapshotFrame<T>? sampleAtTime(double targetTime) {
    final bracket = sampleInterpolated(targetTime);
    if (bracket == null) return null;
    return bracket.t >= 0.5 ? bracket.b : bracket.a;
  }
}
