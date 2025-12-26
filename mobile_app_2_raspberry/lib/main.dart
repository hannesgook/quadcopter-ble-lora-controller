// © 2025 Hannes Göök
// Licensed under the MIT License

import 'dart:async';
import 'dart:convert';
import 'dart:math';
import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'package:permission_handler/permission_handler.dart';
import 'dart:typed_data';

void main() => runApp(const DroneApp());

class DroneApp extends StatelessWidget {
  const DroneApp({super.key});

  @override
  Widget build(BuildContext context) {
    const bg = Color(0xFF050816);
    const primary = Color(0xFF4F46E5);
    const primaryLight = Color(0xFF818CF8);
    const surface = Color(0xFF0B1020);

    final scheme = ColorScheme.dark(
      primary: primary,
      secondary: primaryLight,
      surface: surface,
      background: bg,
    );

    return MaterialApp(
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        useMaterial3: true,
        colorScheme: scheme,
        scaffoldBackgroundColor: scheme.background,
        appBarTheme: const AppBarTheme(
          backgroundColor: Colors.transparent,
          foregroundColor: Colors.white,
          elevation: 0,
          centerTitle: true,
        ),
        filledButtonTheme: FilledButtonThemeData(
          style: ButtonStyle(
            backgroundColor: MaterialStatePropertyAll(scheme.primary),
            foregroundColor: const MaterialStatePropertyAll(Colors.white),
            shape: const MaterialStatePropertyAll(
              RoundedRectangleBorder(
                borderRadius: BorderRadius.all(Radius.circular(14)),
              ),
            ),
            padding: const MaterialStatePropertyAll(
              EdgeInsets.symmetric(horizontal: 16, vertical: 14),
            ),
          ),
        ),
        cardTheme: CardThemeData(
          color: scheme.surface,
          surfaceTintColor: Colors.transparent,
          elevation: 4,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(16),
            side: BorderSide(color: primary.withOpacity(0.16), width: 1),
          ),
        ),
        textTheme: const TextTheme(
          bodyMedium: TextStyle(color: Colors.white70),
          bodyLarge: TextStyle(
            color: Colors.white,
            fontSize: 16,
            fontWeight: FontWeight.w500,
          ),
          titleLarge: TextStyle(
            color: Colors.white,
            fontWeight: FontWeight.w700,
            fontSize: 18,
          ),
        ),
      ),
      home: const ControlScreen(),
    );
  }
}

class ControlScreen extends StatefulWidget {
  const ControlScreen({super.key});
  @override
  State<ControlScreen> createState() => _ControlScreenState();
}

class _ControlScreenState extends State<ControlScreen> {
  final _ble = FlutterReactiveBle();

  Characteristic? _rxResolved;

  double _p = 0.0;
  double _i = 0.0;
  double _d = 0.0;

  double _throttle = 0.0;
  double get throttle => _throttle;

  double get P => _p;
  double get I => _i;
  double get D => _d;

  bool _joystickActive = false;
  bool _writeBusy = false;
  Uint8List? _pendingBytes;

  Timer? _sendTimer;
  final Duration _sendInterval = Duration(milliseconds: 100);
  double _latestX = 0, _latestY = 0;

  final Uuid _serviceUuid = Uuid.parse("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
  final Uuid _rxCharUuid = Uuid.parse("6e400002-b5a3-f393-e0a9-e50e24dcca9e");

  String _targetName = "Ble-LoRa-Bridge";

  DiscoveredDevice? _device;
  StreamSubscription<DiscoveredDevice>? _scanSub;
  StreamSubscription<ConnectionStateUpdate>? _connSub;

  bool _scanning = false;
  bool _connecting = false;
  bool _connected = false;
  bool _ready = false;

  String _status = "Disconnected";
  String _lastError = "";

  int _diagServiceCount = 0;
  bool _diagHasService = false;
  bool _diagRxWritable = false;

  double _jx = 0;
  double _jy = 0;

  @override
  void initState() {
    super.initState();
    _ble.statusStream.listen((s) {
      if (s != BleStatus.ready) setState(() => _status = "Bluetooth not ready");
    });
    _ensurePermissions();
  }

  Widget verticalSlider({
    required double value,
    required double min,
    required double max,
    required int divisions,
    required String label,
    required ValueChanged<double> onChanged,
  }) {
    return RotatedBox(
      quarterTurns: -1,
      child: Slider(
        value: value,
        min: min,
        max: max,
        divisions: divisions,
        label: label,
        onChanged: onChanged,
      ),
    );
  }

  @override
  void dispose() {
    _scanSub?.cancel();
    _connSub?.cancel();
    super.dispose();
  }

  Future<void> _ensurePermissions() async {
    if (!Platform.isAndroid) return;
    final scan = await Permission.bluetoothScan.request();
    final conn = await Permission.bluetoothConnect.request();
    PermissionStatus? loc;
    if (Platform.isAndroid) {
      loc = await Permission.locationWhenInUse.request();
    }
    if (!scan.isGranted || !conn.isGranted || (loc != null && !loc.isGranted)) {
      setState(
        () => _status = "Grant Nearby devices (and Location on Android 10–11)",
      );
    }
  }

  Future<void> _scanAndConnect() async {
    if (_scanning || _connecting || _connected) return;
    setState(() {
      _status = "Scanning";
      _scanning = true;
      _ready = false;
      _lastError = "";
      _diagServiceCount = 0;
      _diagHasService = false;
      _diagRxWritable = false;
      _rxResolved = null;
    });

    _scanSub = _ble
        .scanForDevices(
          withServices: [_serviceUuid],
          scanMode: ScanMode.lowLatency,
        )
        .listen(
          (d) {
            if (d.name == _targetName ||
                d.serviceUuids.contains(_serviceUuid)) {
              _device = d;
              _scanSub?.cancel();
              _scanning = false;
              _connect();
            }
          },
          onError: (e) {
            setState(() {
              _status = "Scan error";
              _lastError = e.toString();
              _scanning = false;
            });
          },
        );

    await Future.delayed(const Duration(seconds: 8));
    if (mounted && !_connected && _scanning) {
      _scanSub?.cancel();
      setState(() {
        _status = "Device not found";
        _scanning = false;
      });
    }
  }

  Future<void> _connect() async {
    if (_device == null) return;
    setState(() {
      _connecting = true;
      _status = "Connecting";
      _ready = false;
      _lastError = "";
    });

    _connSub = _ble
        .connectToDevice(
          id: _device!.id,
          connectionTimeout: const Duration(seconds: 10),
        )
        .listen(
          (update) async {
            if (update.connectionState == DeviceConnectionState.connected) {
              _connected = true;
              _connecting = false;
              _status = "Optimizing link";
              try {
                await _ble.requestMtu(deviceId: _device!.id, mtu: 185);
              } catch (_) {}

              _status = "Discovering";
              final ok = await _resolveExactInstances();
              if (!ok) {
                _disconnect(msg: "Correct service/char not found");
                return;
              }
              _ready = true;
              setState(() => _status = "Connected");
              _startSending();
            } else if (update.connectionState ==
                DeviceConnectionState.disconnected) {
              _disconnect(msg: "Disconnected");
            }
            setState(() {});
          },
          onError: (e) {
            _disconnect(msg: "Connection error: $e");
          },
        );
  }

  Future<bool> _resolveExactInstances() async {
    try {
      final services = await _ble.getDiscoveredServices(_device!.id);
      _diagServiceCount = services.length;

      final nusServices = services.where((s) => s.id == _serviceUuid).toList();
      _diagHasService = nusServices.isNotEmpty;

      Service? chosen;
      Characteristic? rxC;

      for (final s in nusServices) {
        final rxList = s.characteristics
            .where((c) => c.id == _rxCharUuid)
            .toList();
        if (rxList.isNotEmpty) {
          final rxWritable = rxList.firstWhere(
            (c) => c.isWritableWithoutResponse || c.isWritableWithResponse,
            orElse: () => rxList.first,
          );
          chosen = s;
          rxC = rxWritable;
          break;
        }
      }

      if (chosen == null || rxC == null) {
        _diagRxWritable = false;
        _rxResolved = null;
        setState(() {});
        return false;
      }

      _rxResolved = rxC;
      _diagRxWritable =
          rxC.isWritableWithoutResponse || rxC.isWritableWithResponse;

      setState(() {});
      return _diagRxWritable;
    } catch (e) {
      _lastError = "resolveExactInstances: $e";
      setState(() {});
      return false;
    }
  }

  void _disconnect({String msg = "Disconnected"}) {
    _connSub?.cancel();
    _stopSending();
    _connected = false;
    _connecting = false;
    _scanning = false;
    _ready = false;
    _rxResolved = null;
    setState(() {
      _status = msg;
      _lastError = "";
    });
  }

  void _startSending() {
    _sendTimer ??= Timer.periodic(_sendInterval, (_) => _tickSend());
  }

  void _stopSending() {
    _sendTimer?.cancel();
    _sendTimer = null;
  }

  double _r2(double v) => (v * 100).round() / 100.0;

  Future<void> _tickSend() async {
    if (!_ready || !_connected || _rxResolved == null) return;

    final px = _r2(_latestX);
    final py = _r2(_latestY);

    final tStr = throttle.toStringAsFixed(2);
    final pStr = P.toStringAsFixed(2);
    final iStr = I.toStringAsFixed(2);
    final dStr = D.toStringAsFixed(2);

    final bytes = Uint8List.fromList(
      utf8.encode('{"t":$tStr,"x":$px,"y":$py,"p":$pStr,"i":$iStr,"d":$dStr}'),
    );

    await _writeCoalesced(bytes);
  }

  Future<void> _writeCoalesced(Uint8List bytes) async {
    if (_writeBusy) {
      _pendingBytes = bytes;
      return;
    }

    _writeBusy = true;
    try {
      await _rxResolved!.write(bytes, withResponse: false);
    } catch (e) {
      setState(() {
        _status = "Write failed";
        _lastError = e.toString();
      });
    } finally {
      _writeBusy = false;

      if (_joystickActive && _pendingBytes != null) {
        final next = _pendingBytes!;
        _pendingBytes = null;
        _writeBusy = true;
        try {
          await _rxResolved!.write(next, withResponse: false);
        } catch (e) {
          setState(() {
            _status = "Write failed";
            _lastError = e.toString();
          });
        } finally {
          _writeBusy = false;
        }
      } else {
        _pendingBytes = null;
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    final screenWidth = MediaQuery.of(context).size.width;
    final maxContentWidth = min(screenWidth, 800.0) - 32.0;
    final canConnect = !_connected && !_connecting && !_scanning;

    double joySizeTarget = (screenWidth.clamp(0.0, 800.0)) * 0.6;
    const gaps = 24.0;

    double sideWidth = (maxContentWidth - joySizeTarget - gaps) / 2.0;

    if (sideWidth < 160.0) {
      sideWidth = 160.0;
      joySizeTarget = maxContentWidth - 2 * sideWidth - gaps;
      if (joySizeTarget < 120.0) {
        joySizeTarget = 120.0;
      }
    }

    final joySize = joySizeTarget;

    return Scaffold(
      appBar: AppBar(
        title: const Text("Drone App 2025"),
        actions: [
          Padding(
            padding: const EdgeInsets.only(right: 16),
            child: Icon(
              _ready ? Icons.bluetooth_connected : Icons.bluetooth_disabled,
            ),
          ),
        ],
      ),
      body: Center(
        child: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 800),
          child: Padding(
            padding: const EdgeInsets.all(16),
            child: SingleChildScrollView(
              physics: const NeverScrollableScrollPhysics(),
              child: Column(
                children: [
                  Card(
                    child: Padding(
                      padding: const EdgeInsets.symmetric(
                        horizontal: 16,
                        vertical: 14,
                      ),
                      child: Row(
                        children: [
                          Expanded(
                            child: Column(
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: [
                                Text(
                                  "Status: $_status",
                                  style: Theme.of(context).textTheme.bodyLarge,
                                ),
                                if (_lastError.isNotEmpty)
                                  Padding(
                                    padding: const EdgeInsets.only(top: 4),
                                    child: Text(
                                      _lastError,
                                      style: Theme.of(
                                        context,
                                      ).textTheme.bodyMedium,
                                    ),
                                  ),
                              ],
                            ),
                          ),
                          FilledButton(
                            onPressed: canConnect ? _scanAndConnect : null,
                            child: const Text("Connect"),
                          ),
                          const SizedBox(width: 10),
                          FilledButton(
                            onPressed: _connected ? _disconnect : null,
                            child: const Text("Disconnect"),
                          ),
                        ],
                      ),
                    ),
                  ),

                  const SizedBox(height: 16),

                  if (_connected && !_ready)
                    Card(
                      child: Padding(
                        padding: const EdgeInsets.all(16),
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Text(
                              "Diagnostics",
                              style: Theme.of(context).textTheme.titleLarge,
                            ),
                            const SizedBox(height: 8),
                            Text("Services discovered: $_diagServiceCount"),
                            Text(
                              "Has NUS service: ${_diagHasService ? "Yes" : "No"}",
                            ),
                            Text(
                              "RX writable: ${_diagRxWritable ? "Yes" : "No"}",
                            ),
                            const SizedBox(height: 8),
                            const Text(
                              "Waiting for correct device to unlock controls",
                            ),
                          ],
                        ),
                      ),
                    ),

                  const SizedBox(height: 8),

                  if (_ready)
                    Column(
                      children: [
                        Row(
                          mainAxisAlignment: MainAxisAlignment.center,
                          crossAxisAlignment: CrossAxisAlignment.center,
                          children: [
                            SizedBox(
                              width: sideWidth,
                              child: Column(
                                mainAxisAlignment: MainAxisAlignment.center,
                                children: [
                                  Text(
                                    "Responsiveness (P): ${P.toStringAsFixed(2)}",
                                    style: Theme.of(
                                      context,
                                    ).textTheme.bodyLarge,
                                  ),
                                  Slider(
                                    value: P,
                                    min: 0.0,
                                    max: 1.0,
                                    divisions: 100,
                                    label: P.toStringAsFixed(2),
                                    onChanged: (v) {
                                      setState(() {
                                        _p = double.parse(v.toStringAsFixed(2));
                                      });
                                      _startSending();
                                    },
                                  ),
                                  const SizedBox(height: 8),
                                  Text(
                                    "Drift correction (I): ${I.toStringAsFixed(2)}",
                                    style: Theme.of(
                                      context,
                                    ).textTheme.bodyLarge,
                                  ),
                                  Slider(
                                    value: I,
                                    min: 0.0,
                                    max: 1.0,
                                    divisions: 100,
                                    label: I.toStringAsFixed(2),
                                    onChanged: (v) {
                                      setState(() {
                                        _i = double.parse(v.toStringAsFixed(2));
                                      });
                                      _startSending();
                                    },
                                  ),
                                  const SizedBox(height: 8),
                                  Text(
                                    "Overshoot damping (D): ${D.toStringAsFixed(2)}",
                                    style: Theme.of(
                                      context,
                                    ).textTheme.bodyLarge,
                                  ),
                                  Slider(
                                    value: D,
                                    min: 0.0,
                                    max: 1.0,
                                    divisions: 100,
                                    label: D.toStringAsFixed(2),
                                    onChanged: (v) {
                                      setState(() {
                                        _d = double.parse(v.toStringAsFixed(2));
                                      });
                                      _startSending();
                                    },
                                  ),
                                ],
                              ),
                            ),

                            const SizedBox(width: 12),

                            SizedBox(
                              width: joySize,
                              height: joySize,
                              child: _FixedBaseJoystick(
                                onStart: () {
                                  _joystickActive = true;
                                  _startSending();
                                },
                                onChanged: (x, y) {
                                  setState(() {
                                    _jx = x;
                                    _jy = y;
                                  });
                                  _latestX = x;
                                  _latestY = y;
                                },
                                onRelease: () {
                                  _joystickActive = false;
                                },
                              ),
                            ),

                            const SizedBox(width: 12),

                            SizedBox(
                              width: sideWidth,
                              child: Column(
                                mainAxisAlignment: MainAxisAlignment.center,
                                children: [
                                  Text(
                                    "Throttle: ${throttle.toStringAsFixed(2)}",
                                    style: Theme.of(
                                      context,
                                    ).textTheme.bodyLarge,
                                  ),
                                  const SizedBox(height: 8),
                                  SizedBox(
                                    height: joySize * 0.8,
                                    child: verticalSlider(
                                      value: throttle,
                                      min: 0.0,
                                      max: 1.0,
                                      divisions: 100,
                                      label: throttle.toStringAsFixed(2),
                                      onChanged: (v) {
                                        setState(() {
                                          _throttle = double.parse(
                                            v.toStringAsFixed(2),
                                          );
                                        });
                                        _startSending();
                                      },
                                    ),
                                  ),
                                ],
                              ),
                            ),
                          ],
                        ),

                        const SizedBox(height: 24),
                      ],
                    )
                  else
                    Padding(
                      padding: const EdgeInsets.only(top: 16),
                      child: Text(
                        "Connect to the desired device to enable joystick",
                        style: Theme.of(context).textTheme.bodyMedium,
                      ),
                    ),

                  const Padding(
                    padding: EdgeInsets.only(bottom: 8, top: 16),
                    child: Text(
                      "App by Hannes Göök",
                      textAlign: TextAlign.center,
                      style: TextStyle(
                        color: Colors.white54,
                        fontSize: 14,
                        fontStyle: FontStyle.italic,
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );
  }
}

class _FixedBaseJoystick extends StatefulWidget {
  final void Function(double x, double y) onChanged;
  final VoidCallback? onStart;
  final VoidCallback? onRelease;

  const _FixedBaseJoystick({
    required this.onChanged,
    this.onStart,
    this.onRelease,
  });

  @override
  State<_FixedBaseJoystick> createState() => _FixedBaseJoystickState();
}

class _FixedBaseJoystickState extends State<_FixedBaseJoystick> {
  Offset _knob = Offset.zero;
  double _radius = 0;
  bool _latched = false;
  bool _gestureLatched = false;

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (context, c) {
        final size = min(c.maxWidth, c.maxHeight);
        _radius = size * 0.35;
        final knobRadius = size * 0.12;

        return GestureDetector(
          onPanStart: (d) {
            widget.onStart?.call();
            _gestureLatched = false;
            _latched = false;
            _updateFromLocal(d.localPosition, size);
          },
          onPanUpdate: (d) => _updateFromLocal(d.localPosition, size),
          onPanEnd: (_) {
            _gestureLatched = false;
            if (!_latched) {
              setState(() => _knob = Offset.zero);
              widget.onChanged(0, 0);
            }
            widget.onRelease?.call();
          },
          child: CustomPaint(
            painter: _JoystickPainter(
              baseRadius: _radius,
              knobCenter: _knob,
              knobRadius: knobRadius,
            ),
            child: const SizedBox.expand(),
          ),
        );
      },
    );
  }

  void _updateFromLocal(Offset p, double size) {
    if (_gestureLatched) return;

    final center = Offset(size / 2, size / 2);
    final v = p - center;
    final dist = v.distance;

    if (dist > _radius) {
      final snappedAngle = _nearestOctant(v.direction);
      final snappedVec = Offset.fromDirection(snappedAngle, _radius);
      setState(() {
        _knob = snappedVec;
        _latched = true;
        _gestureLatched = true;
      });
      final nx = (_knob.dx / _radius).clamp(-1.0, 1.0);
      final ny = (-_knob.dy / _radius).clamp(-1.0, 1.0);
      widget.onChanged(nx.toDouble(), ny.toDouble());
      return;
    }

    if (_latched) return;

    setState(() => _knob = v);
    final nx = (v.dx / _radius).clamp(-1.0, 1.0);
    final ny = (-v.dy / _radius).clamp(-1.0, 1.0);
    widget.onChanged(nx.toDouble(), ny.toDouble());
  }

  double _nearestOctant(double angle) {
    double a = angle;
    while (a <= -pi) a += 2 * pi;
    while (a > pi) a -= 2 * pi;
    const step = pi / 4;
    final idx = (a / step).round();
    return idx * step.toDouble();
  }
}

class _JoystickPainter extends CustomPainter {
  final double baseRadius;
  final double knobRadius;
  final Offset knobCenter;
  _JoystickPainter({
    required this.baseRadius,
    required this.knobCenter,
    required this.knobRadius,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);

    const bg = Color(0xFF050816);
    const ring = Color(0xFF111827);
    const accent = Color(0xFF4F46E5);
    const knob = Color(0xFF818CF8);

    final basePaint = Paint()..color = bg;
    final ringPaint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = baseRadius * 0.08
      ..color = ring;
    final tickPaint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = baseRadius * 0.02
      ..color = ring.withOpacity(0.9);
    final knobPaint = Paint()..color = knob;
    final accentPaint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = baseRadius * 0.03
      ..color = accent;

    canvas.drawCircle(center, baseRadius, basePaint);
    canvas.drawCircle(center, baseRadius, ringPaint);
    canvas.drawCircle(center, baseRadius * 0.5, accentPaint);

    for (int i = 0; i < 8; i++) {
      final a = i * pi / 4;
      final p1 = center + Offset.fromDirection(a, baseRadius * 0.7);
      final p2 = center + Offset.fromDirection(a, baseRadius * 0.95);
      canvas.drawLine(p1, p2, tickPaint);
    }

    final knobPos = center + knobCenter;
    canvas.drawCircle(knobPos, knobRadius, knobPaint);
    canvas.drawCircle(knobPos, knobRadius, accentPaint);
  }

  @override
  bool shouldRepaint(covariant _JoystickPainter oldDelegate) {
    return oldDelegate.knobCenter != knobCenter ||
        oldDelegate.baseRadius != baseRadius ||
        oldDelegate.knobRadius != knobRadius;
  }
}
