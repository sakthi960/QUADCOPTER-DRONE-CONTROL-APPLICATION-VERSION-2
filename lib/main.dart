// ignore_for_file: deprecated_member_use, use_build_context_synchronously

import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:math' as math;
import 'dart:io';
import 'dart:convert';
import 'package:flutter/services.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:sensors_plus/sensors_plus.dart';

void main() {
  runApp(const DroneControlApp());
}

class DroneControlApp extends StatelessWidget {
  const DroneControlApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'UDP Drone Control',
      theme: ThemeData(
        brightness: Brightness.dark,
        primaryColor: const Color(0xFF4A5FFF),
        scaffoldBackgroundColor: const Color(0xFF1A1D2E),
        cardColor: const Color(0xFF262B3F),
      ),
      home: const DroneHomePage(),
      debugShowCheckedModeBanner: false,
    );
  }
}

class DroneHomePage extends StatefulWidget {
  const DroneHomePage({super.key});

  @override
  State<DroneHomePage> createState() => _DroneHomePageState();
}

class _DroneHomePageState extends State<DroneHomePage> {
  // Connection
  bool isConnected = false;
  bool isESP32Connected = false;
  String currentDroneIp = "10.189.133.122";
  int currentPort = 8888;
  RawDatagramSocket? socket;

  // IP Presets
  final String networkModeIP = "10.189.133.122";

  // UDP Send Timer (50Hz = 20ms)
  Timer? udpSendTimer;

  // Control Values (PWM range: 1000-2000)
  int throttle = 1150;
  int roll = 1500;
  int pitch = 1500;
  int yaw = 1500;
  int arm = 0;

  // Joystick positions (-1.0 to +1.0)
  double leftX = 0.0;
  double leftY = -1.0;  // ✅ START AT BOTTOM (throttle idle)
  double rightX = 0.0;
  double rightY = 0.0;

  // ✅ NEW: Sensitivity controls
  double throttleSensitivity = 1.0;
  double stickSensitivity = 1.0;

  // Telemetry
  String flightMode = "DISARMED";
  double altitude = 0.0;
  double batteryVoltage = 11.1;
  double batteryPercent = 0.0;
  int packetsPerSecond = 0;
  int _packetCounter = 0;
  Timer? _ppsTimer;

  // LED Control
  bool isLEDActive = false;

  // Compass
  double compassDirection = 0.0;
  StreamSubscription<GyroscopeEvent>? _gyroscopeSubscription;

  // ESP32 Connectivity Timer
  Timer? _connectivityTimer;

  // Connection handshake
  Completer<bool>? _connectionCompleter;
  Timer? _connectionTimeout;
  DateTime? _lastPacketTime;

  @override
  void initState() {
    super.initState();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
    _loadSavedSettings();
    _startPPSCounter();
    _startCompass();
    _startConnectivityCheck();
  }

  @override
  void dispose() {
    udpSendTimer?.cancel();
    _ppsTimer?.cancel();
    _gyroscopeSubscription?.cancel();
    _connectivityTimer?.cancel();
    _connectionTimeout?.cancel();
    socket?.close();
    super.dispose();
  }

  void _startCompass() {
    _gyroscopeSubscription = gyroscopeEvents.listen((GyroscopeEvent event) {
      setState(() {
        compassDirection += event.z * 0.1;
        compassDirection %= 360.0;
      });
    });
  }

  void _startConnectivityCheck() {
    _connectivityTimer = Timer.periodic(const Duration(seconds: 2), (_) async {
      if (isConnected) {
        if (_lastPacketTime != null) {
          final timeSinceLastPacket = DateTime.now().difference(_lastPacketTime!);
          if (timeSinceLastPacket.inSeconds > 5) {
            setState(() {
              isESP32Connected = false;
            });
          }
        }
      }
    });
  }

  Future<void> _loadSavedSettings() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final savedIP = prefs.getString('drone_ip');
      final savedPort = prefs.getInt('drone_port');
      final savedThrottleSens = prefs.getDouble('throttle_sensitivity');
      final savedStickSens = prefs.getDouble('stick_sensitivity');

      if (savedIP != null && savedIP.isNotEmpty) {
        setState(() {
          currentDroneIp = savedIP;
        });
      }

      if (savedPort != null) {
        setState(() {
          currentPort = savedPort;
        });
      }

      if (savedThrottleSens != null) {
        setState(() {
          throttleSensitivity = savedThrottleSens;
        });
      }

      if (savedStickSens != null) {
        setState(() {
          stickSensitivity = savedStickSens;
        });
      }

      debugPrint('Settings loaded');
    } catch (e) {
      debugPrint('Error loading settings: $e');
    }
  }

  Future<void> _saveIP(String ip) async {
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setString('drone_ip', ip);
    } catch (e) {
      debugPrint('Error saving IP: $e');
    }
  }

  Future<void> _savePort(int port) async {
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setInt('drone_port', port);
    } catch (e) {
      debugPrint('Error saving port: $e');
    }
  }

  Future<void> _saveSensitivity(double throttle, double stick) async {
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setDouble('throttle_sensitivity', throttle);
      await prefs.setDouble('stick_sensitivity', stick);
    } catch (e) {
      debugPrint('Error saving sensitivity: $e');
    }
  }

  void _startPPSCounter() {
    _ppsTimer = Timer.periodic(const Duration(seconds: 1), (_) {
      setState(() {
        packetsPerSecond = _packetCounter;
        _packetCounter = 0;
      });
    });
  }

  Future<void> initUdp() async {
    try {
      socket?.close();
      socket = null;
      _connectionTimeout?.cancel();

      debugPrint('Attempting connection to $currentDroneIp:$currentPort');

      socket = await RawDatagramSocket.bind(InternetAddress.anyIPv4, 0);

      _connectionCompleter = Completer<bool>();
      _connectionTimeout = Timer(const Duration(seconds: 5), () {
        if (_connectionCompleter != null && !_connectionCompleter!.isCompleted) {
          _connectionCompleter!.complete(false);
        }
      });

      socket?.listen((event) {
        if (event == RawSocketEvent.read) {
          Datagram? dg = socket?.receive();
          if (dg != null) {
            String data = utf8.decode(dg.data).trim();
            _lastPacketTime = DateTime.now();

            if (data == "PONG") {
              if (_connectionCompleter != null && !_connectionCompleter!.isCompleted) {
                _connectionCompleter!.complete(true);
              }
            } else {
              _processTelemetry(data);
            }
          }
        }
      });

      for (int i = 0; i < 3; i++) {
        socket?.send(utf8.encode("PING\n"), InternetAddress(currentDroneIp), currentPort);
        await Future.delayed(const Duration(milliseconds: 100));
      }

      bool handshakeSuccess = await _connectionCompleter!.future;

      _connectionTimeout?.cancel();
      _connectionTimeout = null;
      _connectionCompleter = null;

      if (handshakeSuccess) {
        setState(() {
          isConnected = true;
          isESP32Connected = true;
          flightMode = arm == 1 ? "ARMED" : "DISARMED";
        });

        _startContinuousSend();

        if (mounted) {
          ScaffoldMessenger.of(context).showSnackBar(
            SnackBar(
              content: Text('✓ Connected to ESP32 at $currentDroneIp:$currentPort'),
              backgroundColor: Colors.green,
              duration: const Duration(seconds: 2),
            ),
          );
        }
      } else {
        socket?.close();
        socket = null;
        setState(() {
          isConnected = false;
          isESP32Connected = false;
        });
        if (mounted) {
          ScaffoldMessenger.of(context).showSnackBar(
            SnackBar(
              content: Text(
                '✗ Connection Failed: No response from $currentDroneIp:$currentPort',
              ),
              backgroundColor: Colors.red,
              duration: const Duration(seconds: 4),
            ),
          );
        }
      }
    } catch (e) {
      _connectionTimeout?.cancel();
      _connectionTimeout = null;
      _connectionCompleter = null;

      socket?.close();
      socket = null;
      setState(() {
        isConnected = false;
        isESP32Connected = false;
      });
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('✗ Connection Error: $e'),
            backgroundColor: Colors.red,
            duration: const Duration(seconds: 3),
          ),
        );
      }
    }
  }

  int mapJoystick(double value) {
    if (value.abs() < 0.05) value = 0;
    double scaled = value * stickSensitivity;
    scaled = scaled.clamp(-1.0, 1.0);
    return (1500 + scaled * 500).toInt().clamp(1000, 2000);
  }

  int mapThrottle(double value) {
    double scaled = value * throttleSensitivity;
    scaled = scaled.clamp(-1.0, 1.0);
    int pwm = (1150 + ((scaled + 1) / 2) * (1900 - 1150)).toInt();
    return pwm.clamp(1150, 1900);
  }

  String buildPacket() {
    roll = mapJoystick(rightX);
    pitch = mapJoystick(-rightY);
    yaw = mapJoystick(leftX);
    throttle = mapThrottle(leftY);
    return "$roll,$pitch,$yaw,$throttle,$arm\n";
  }

  void sendPacket() {
    if (socket == null || !isConnected) return;
    try {
      String packet = buildPacket();
      socket?.send(utf8.encode(packet), InternetAddress(currentDroneIp), currentPort);
      _packetCounter++;
    } catch (e) {
      debugPrint('Send error: $e');
    }
  }

  void _startContinuousSend() {
    udpSendTimer?.cancel();
    udpSendTimer = Timer.periodic(
      const Duration(milliseconds: 20),
      (_) => sendPacket(),
    );
  }

  void armMotors() {
    if (!isConnected) {
      _showError('Not connected to drone');
      return;
    }
    if (throttle > 1200) {
      _showError('Cannot ARM - Lower throttle to idle first!');
      return;
    }
    setState(() {
      arm = 1;
      flightMode = "ARMED";
    });
    _showSuccess('Motors ARMED');
  }

  void disarmMotors() {
    setState(() {
      arm = 0;
      flightMode = "DISARMED";
    });
    _showSuccess('Motors DISARMED');
  }

  void emergencyStop() {
    setState(() {
      arm = 0;
      flightMode = "EMERGENCY";
    });
    sendPacket();
    _showError('EMERGENCY STOP ACTIVATED');
  }

  void disconnect() {
    udpSendTimer?.cancel();
    socket?.close();
    socket = null;

    setState(() {
      isConnected = false;
      isESP32Connected = false;
      arm = 0;
      throttle = 1150;
      roll = 1500;
      pitch = 1500;
      yaw = 1500;
      leftX = 0;
      leftY = -1.0;
      rightX = 0;
      rightY = 0;
      flightMode = "DISCONNECTED";
      packetsPerSecond = 0;
    });

    _showSuccess('Disconnected');
  }

  void toggleLED() {
    if (!isConnected) {
      _showError('Not connected to drone');
      return;
    }

    try {
      String ledCommand = isLEDActive ? "LED:OFF\n" : "LED:ON\n";
      socket?.send(utf8.encode(ledCommand), InternetAddress(currentDroneIp), currentPort);

      setState(() {
        isLEDActive = !isLEDActive;
      });

      _showSuccess('Drone light ${isLEDActive ? 'ON' : 'OFF'}');
    } catch (e) {
      _showError('LED command failed: $e');
    }
  }

  void _processTelemetry(String data) {
    try {
      Map<String, String> telemetry = {};
      for (var pair in data.split(',')) {
        var kv = pair.split(':');
        if (kv.length == 2) {
          telemetry[kv[0]] = kv[1];
        }
      }

      setState(() {
        altitude = double.tryParse(telemetry['ALT'] ?? '0') ?? altitude;
        batteryVoltage = double.tryParse(telemetry['BAT'] ?? '11.1') ?? batteryVoltage;
        batteryPercent = ((batteryVoltage - 10.2) / (12.6 - 10.2) * 100).clamp(0, 100);
        if (telemetry['MODE'] != null) flightMode = telemetry['MODE']!;
      });
    } catch (e) {
      debugPrint('Telemetry parse error: $e');
    }
  }

  void _updateLeftJoystick(Offset position, Size size) {
    double centerX = size.width / 2;
    double centerY = size.height / 2;
    double dx = position.dx - centerX;
    double dy = position.dy - centerY;
    double distance = math.sqrt(dx * dx + dy * dy);
    double maxDistance = centerX - 40;

    if (distance > maxDistance) {
      double angle = math.atan2(dy, dx);
      dx = math.cos(angle) * maxDistance;
      dy = math.sin(angle) * maxDistance;
    }

    setState(() {
      leftX = (dx / maxDistance).clamp(-1.0, 1.0);
      leftY = (dy / maxDistance).clamp(-1.0, 1.0);
    });
  }

  void _updateRightJoystick(Offset position, Size size) {
    double centerX = size.width / 2;
    double centerY = size.height / 2;
    double dx = position.dx - centerX;
    double dy = position.dy - centerY;
    double distance = math.sqrt(dx * dx + dy * dy);
    double maxDistance = centerX - 40;

    if (distance > maxDistance) {
      double angle = math.atan2(dy, dx);
      dx = math.cos(angle) * maxDistance;
      dy = math.sin(angle) * maxDistance;
    }

    setState(() {
      rightX = (dx / maxDistance).clamp(-1.0, 1.0);
      rightY = (dy / maxDistance).clamp(-1.0, 1.0);
    });
  }

  void _showError(String msg) {
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text(msg), backgroundColor: Colors.red),
      );
    }
  }

  void _showSuccess(String msg) {
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text(msg), backgroundColor: Colors.green),
      );
    }
  }

  void _showConnectionBottomSheet() {
    showDialog(
      context: context,
      builder: (BuildContext context) {
        return Dialog(
          backgroundColor: const Color(0xFF262B3F),
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
          child: Container(
            padding: const EdgeInsets.all(24),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                const Icon(Icons.wifi, color: Color(0xFF4A5FFF), size: 64),
                const SizedBox(height: 16),
                const Text(
                  'Connect to ESP32 Drone',
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 24,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                const SizedBox(height: 24),
                ElevatedButton(
                  onPressed: () {
                    _handleIPChange(networkModeIP);
                    Navigator.pop(context);
                    Future.delayed(const Duration(milliseconds: 300), initUdp);
                  },
                  style: ElevatedButton.styleFrom(
                    backgroundColor: const Color(0xFF4A5FFF),
                    padding: const EdgeInsets.symmetric(horizontal: 32, vertical: 16),
                  ),
                  child: const Text('Connect to Drone', textAlign: TextAlign.center),
                ),
                const SizedBox(height: 16),
                ElevatedButton(
                  onPressed: () {
                    Navigator.pop(context);
                    Navigator.push(
                      context,
                      MaterialPageRoute(
                        builder: (context) => SettingsPage(
                          currentIP: currentDroneIp,
                          currentPort: currentPort,
                          isConnected: isConnected,
                          throttleSensitivity: throttleSensitivity,
                          stickSensitivity: stickSensitivity,
                          onIPChanged: _handleIPChange,
                          onPortChanged: _handlePortChange,
                          onSensitivityChanged: _handleSensitivityChange,
                        ),
                      ),
                    );
                  },
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.grey[700],
                    padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 12),
                  ),
                  child: const Text('Custom Settings'),
                ),
              ],
            ),
          ),
        );
      },
    );
  }

  void _handleIPChange(String newIP) {
    if (isConnected) disconnect();
    setState(() {
      currentDroneIp = newIP;
    });
    _saveIP(newIP);
  }

  void _handlePortChange(int newPort) {
    if (isConnected) disconnect();
    setState(() {
      currentPort = newPort;
    });
    _savePort(newPort);
  }

  void _handleSensitivityChange(double throttle, double stick) {
    setState(() {
      throttleSensitivity = throttle.clamp(0.5, 2.0);
      stickSensitivity = stick.clamp(0.5, 2.0);
    });
    _saveSensitivity(throttleSensitivity, stickSensitivity);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      drawer: _buildDrawer(),
      body: SafeArea(
        child: Column(
          children: [
            _buildTopBar(),
            Expanded(
              child: Row(
                children: [
                  Expanded(
                    child: Center(
                      child: _buildJoystick(
                        leftX,
                        leftY,
                        'YAW / THROTTLE',
                        isESP32Connected ? (pos, size) => _updateLeftJoystick(pos, size) : null,
                        () => setState(() => leftX = 0), // Only center YAW
                        isThrottleStick: true,
                      ),
                    ),
                  ),
                  SizedBox(
                    width: 100,
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        _buildControlButton(
                          'ARM',
                          Icons.lock_open,
                          Colors.green,
                          (isConnected && isESP32Connected && arm == 0) ? armMotors : null,
                        ),
                        const SizedBox(height: 8),
                        _buildControlButton(
                          'DISARM',
                          Icons.lock,
                          Colors.orange,
                          (isConnected && isESP32Connected && arm == 1) ? disarmMotors : null,
                        ),
                        const SizedBox(height: 8),
                        _buildControlButton(
                          isLEDActive ? 'LED ON' : 'LED OFF',
                          isLEDActive ? Icons.lightbulb : Icons.lightbulb_outline,
                          isLEDActive ? Colors.amber : Colors.blue,
                          (isConnected && isESP32Connected) ? toggleLED : null,
                        ),
                        const SizedBox(height: 8),
                        _buildControlButton(
                          'EMERGENCY',
                          Icons.warning,
                          Colors.red,
                          (isConnected && isESP32Connected) ? emergencyStop : null,
                        ),
                      ],
                    ),
                  ),
                  Expanded(
                    child: Center(
                      child: _buildJoystick(
                        rightX,
                        rightY,
                        'ROLL / PITCH',
                        isESP32Connected ? (pos, size) => _updateRightJoystick(pos, size) : null,
                        () => setState(() {
                          rightX = 0;
                          rightY = 0;
                        }),
                        isThrottleStick: false,
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildControlButton(String label, IconData icon, Color color, VoidCallback? onPressed) {
    return SizedBox(
      width: 80,
      height: 40,
      child: ElevatedButton(
        onPressed: onPressed,
        style: ElevatedButton.styleFrom(
          backgroundColor: color.withOpacity(0.2),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(8),
            side: BorderSide(color: color, width: 1),
          ),
          padding: EdgeInsets.zero,
        ),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(icon, color: color, size: 16),
            Text(
              label,
              style: TextStyle(
                color: color,
                fontWeight: FontWeight.bold,
                fontSize: label.length > 6 ? 8 : 10,
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildTopBar() {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      decoration: BoxDecoration(
        color: const Color(0xFF262B3F),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.3),
            blurRadius: 8,
            offset: const Offset(0, 2),
          ),
        ],
      ),
      child: Row(
        children: [
          Builder(
            builder: (context) => IconButton(
              icon: const Icon(Icons.menu, color: Colors.white),
              onPressed: () => Scaffold.of(context).openDrawer(),
            ),
          ),
          const Icon(Icons.flight, color: Color(0xFF4A5FFF), size: 24),
          const SizedBox(width: 8),
          const Text(
            'UDP Drone Control',
            style: TextStyle(
              fontSize: 18,
              fontWeight: FontWeight.bold,
              color: Color(0xFF4A5FFF),
            ),
          ),
          const Spacer(),
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 4),
            decoration: BoxDecoration(
              color: _getFlightModeColor().withOpacity(0.2),
              border: Border.all(color: _getFlightModeColor(), width: 2),
              borderRadius: BorderRadius.circular(16),
            ),
            child: Row(
              children: [
                Icon(_getFlightModeIcon(), size: 16, color: _getFlightModeColor()),
                const SizedBox(width: 6),
                Text(
                  flightMode,
                  style: TextStyle(
                    color: _getFlightModeColor(),
                    fontWeight: FontWeight.bold,
                    fontSize: 12,
                  ),
                ),
              ],
            ),
          ),
          const SizedBox(width: 12),
          if (!isConnected)
            ElevatedButton.icon(
              onPressed: _showConnectionBottomSheet,
              icon: const Icon(Icons.wifi, size: 18),
              label: const Text('Connect'),
              style: ElevatedButton.styleFrom(
                backgroundColor: const Color(0xFF4A5FFF),
                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
              ),
            )
          else
            ElevatedButton.icon(
              onPressed: disconnect,
              icon: const Icon(Icons.close, size: 18),
              label: const Text('Disconnect'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.orange,
                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
              ),
            ),
        ],
      ),
    );
  }

  Color _getFlightModeColor() {
    switch (flightMode) {
      case "DISARMED":
        return Colors.blue;
      case "ARMED":
        return Colors.yellow;
      case "EMERGENCY":
        return Colors.red;
      default:
        return Colors.grey;
    }
  }

  IconData _getFlightModeIcon() {
    switch (flightMode) {
      case "DISARMED":
        return Icons.lock;
      case "ARMED":
        return Icons.lock_open;
      case "EMERGENCY":
        return Icons.warning;
      default:
        return Icons.help_outline;
    }
  }

  Widget _buildJoystick(
    double x,
    double y,
    String label,
    Function(Offset, Size)? onUpdate,
    VoidCallback onReset,
    {required bool isThrottleStick}
  ) {
    const double joystickSize = 240;
    const double handleSize = 60;
    const double maxRadius = (joystickSize / 2) - (handleSize / 2) - 10;

    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        Text(
          label,
          style: const TextStyle(
            color: Colors.white70,
            fontSize: 12,
            fontWeight: FontWeight.bold,
          ),
        ),
        const SizedBox(height: 8),
        SizedBox(
          width: joystickSize,
          height: joystickSize,
          child: GestureDetector(
            onPanStart: onUpdate != null
                ? (details) => onUpdate(details.localPosition, const Size(joystickSize, joystickSize))
                : null,
            onPanUpdate: onUpdate != null
                ? (details) => onUpdate(details.localPosition, const Size(joystickSize, joystickSize))
                : null,
            onPanEnd: (_) => onReset(),
            child: Container(
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: [
                    (isThrottleStick ? const Color(0xFFFF9800) : const Color(0xFF4A5FFF)).withOpacity(0.2),
                    (isThrottleStick ? const Color(0xFFFF5722) : const Color(0xFF9C27B0)).withOpacity(0.2),
                  ],
                ),
                borderRadius: BorderRadius.circular(20),
                border: Border.all(
                  color: (isThrottleStick ? const Color(0xFFFF9800) : const Color(0xFF4A5FFF)).withOpacity(0.3),
                  width: 2,
                ),
              ),
              child: Stack(
                alignment: Alignment.center,
                clipBehavior: Clip.none,
                children: [
                  if (isThrottleStick) ...[
                    Positioned(
                      top: 15,
                      child: Text(
                        'MAX',
                        style: TextStyle(
                          color: Colors.green.withOpacity(0.6),
                          fontSize: 10,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                    ),
                    Positioned(
                      bottom: 15,
                      child: Text(
                        'IDLE',
                        style: TextStyle(
                          color: Colors.orange.withOpacity(0.6),
                          fontSize: 10,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                    ),
                  ],
                  Container(
                    width: 40,
                    height: 40,
                    decoration: BoxDecoration(
                      border: Border.all(
                        color: isThrottleStick ? Colors.orange.withOpacity(0.4) : Colors.white24,
                        width: 1,
                      ),
                      shape: BoxShape.circle,
                    ),
                  ),
                  Positioned(
                    left: (joystickSize / 2) + (x * maxRadius) - (handleSize / 2),
                    top: (joystickSize / 2) + (y * maxRadius) - (handleSize / 2),
                    child: Container(
                      width: handleSize,
                      height: handleSize,
                      decoration: BoxDecoration(
                        shape: BoxShape.circle,
                        gradient: RadialGradient(
                          colors: [
                            isThrottleStick ? const Color(0xFFFF9800) : const Color(0xFF4A5FFF),
                            isThrottleStick ? const Color(0xFFFF5722) : const Color(0xFF9C27B0),
                          ],
                        ),
                        boxShadow: [
                          BoxShadow(
                            color: (isThrottleStick ? Colors.orange : Colors.blue).withOpacity(0.5),
                            blurRadius: 10,
                            spreadRadius: 2,
                          ),
                        ],
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildDrawer() {
    return Drawer(
      backgroundColor: const Color(0xFF262B3F),
      child: ListView(
        padding: EdgeInsets.zero,
        children: [
          DrawerHeader(
            decoration: const BoxDecoration(
              gradient: LinearGradient(
                colors: [Color(0xFF4A5FFF), Color(0xFF9C27B0)],
              ),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisAlignment: MainAxisAlignment.end,
              children: [
                const Icon(Icons.flight, size: 48, color: Colors.white),
                const SizedBox(height: 8),
                const Text(
                  'Drone Control',
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 24,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                Text(
                  'v4.0 - Safe Flight Edition',
                  style: TextStyle(
                    color: Colors.white.withOpacity(0.7),
                    fontSize: 12,
                  ),
                ),
              ],
            ),
          ),
          _buildDrawerItem(
            Icons.dashboard,
            'Telemetry',
            () {
              Navigator.pop(context);
              Navigator.push(
                context,
                MaterialPageRoute(
                  builder: (context) => TelemetryPage(
                    throttle: throttle,
                    roll: roll,
                    pitch: pitch,
                    yaw: yaw,
                    arm: arm,
                    altitude: altitude,
                    batteryVoltage: batteryVoltage,
                    batteryPercent: batteryPercent,
                    packetsPerSecond: packetsPerSecond,
                    compassDirection: compassDirection,
                    isConnected: isConnected,
                    isESP32Connected: isESP32Connected,
                  ),
                ),
              );
            },
          ),
          _buildDrawerItem(
            Icons.settings,
            'Settings',
            () {
              Navigator.pop(context);
              Navigator.push(
                context,
                MaterialPageRoute(
                  builder: (context) => SettingsPage(
                    currentIP: currentDroneIp,
                    currentPort: currentPort,
                    isConnected: isConnected,
                    throttleSensitivity: throttleSensitivity,
                    stickSensitivity: stickSensitivity,
                    onIPChanged: _handleIPChange,
                    onPortChanged: _handlePortChange,
                    onSensitivityChanged: _handleSensitivityChange,
                  ),
                ),
              );
            },
          ),
          _buildDrawerItem(
            Icons.info,
            'About',
            () {
              Navigator.pop(context);
              _showAboutDialog();
            },
          ),
        ],
      ),
    );
  }

  Widget _buildDrawerItem(IconData icon, String title, VoidCallback onTap) {
    return ListTile(
      leading: Icon(icon, color: const Color(0xFF4A5FFF)),
      title: Text(title, style: const TextStyle(color: Colors.white)),
      onTap: onTap,
    );
  }

  void _showAboutDialog() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        backgroundColor: const Color(0xFF262B3F),
        title: const Text(
          'About',
          style: TextStyle(color: Colors.white),
        ),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'UDP Drone Control',
              style: TextStyle(
                color: Colors.white,
                fontSize: 18,
                fontWeight: FontWeight.bold,
              ),
            ),
            const SizedBox(height: 8),
            const Text(
              'Version 4.0 - Safe Flight Edition',
              style: TextStyle(color: Colors.white70),
            ),
            const SizedBox(height: 16),
            const Text(
              '🔒 Safety Features:',
              style: TextStyle(color: Color(0xFF4A5FFF), fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 8),
            _buildFeatureItem('✓ Throttle non-centering (holds position)'),
            _buildFeatureItem('✓ ARM safety check (throttle must be idle)'),
            _buildFeatureItem('✓ Max throttle limited to 95% (1900 PWM)'),
            _buildFeatureItem('✓ Adjustable sensitivity controls'),
            _buildFeatureItem('✓ Emergency stop button'),
            _buildFeatureItem('✓ Connection handshake validation'),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('Close', style: TextStyle(color: Color(0xFF4A5FFF))),
          ),
        ],
      ),
    );
  }

  Widget _buildFeatureItem(String text) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 4),
      child: Text(
        text,
        style: const TextStyle(color: Colors.white70, fontSize: 12),
      ),
    );
  }
}

// ==================== SETTINGS PAGE ====================
class SettingsPage extends StatefulWidget {
  final String currentIP;
  final int currentPort;
  final bool isConnected;
  final double throttleSensitivity;
  final double stickSensitivity;
  final Function(String) onIPChanged;
  final Function(int) onPortChanged;
  final Function(double, double) onSensitivityChanged;

  const SettingsPage({
    super.key,
    required this.currentIP,
    required this.currentPort,
    required this.isConnected,
    required this.throttleSensitivity,
    required this.stickSensitivity,
    required this.onIPChanged,
    required this.onPortChanged,
    required this.onSensitivityChanged,
  });

  @override
  State<SettingsPage> createState() => _SettingsPageState();
}

class _SettingsPageState extends State<SettingsPage> {
  late TextEditingController _ipController;
  late TextEditingController _portController;
  late double _throttleSens;
  late double _stickSens;

  @override
  void initState() {
    super.initState();
    _ipController = TextEditingController(text: widget.currentIP);
    _portController = TextEditingController(text: widget.currentPort.toString());
    _throttleSens = widget.throttleSensitivity;
    _stickSens = widget.stickSensitivity;
  }

  @override
  void dispose() {
    _ipController.dispose();
    _portController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF1A1D2E),
      appBar: AppBar(
        title: const Text('Settings'),
        backgroundColor: const Color(0xFF262B3F),
      ),
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(24),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            _buildSection(
              'Connection Settings',
              Icons.wifi,
              [
                TextField(
                  controller: _ipController,
                  style: const TextStyle(color: Colors.white),
                  decoration: const InputDecoration(
                    labelText: 'Drone IP Address',
                    labelStyle: TextStyle(color: Colors.white70),
                    border: OutlineInputBorder(),
                    enabledBorder: OutlineInputBorder(
                      borderSide: BorderSide(color: Color(0xFF4A5FFF)),
                    ),
                  ),
                ),
                const SizedBox(height: 16),
                TextField(
                  controller: _portController,
                  keyboardType: TextInputType.number,
                  style: const TextStyle(color: Colors.white),
                  decoration: const InputDecoration(
                    labelText: 'Port',
                    labelStyle: TextStyle(color: Colors.white70),
                    border: OutlineInputBorder(),
                    enabledBorder: OutlineInputBorder(
                      borderSide: BorderSide(color: Color(0xFF4A5FFF)),
                    ),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 32),
            _buildSection(
              'Sensitivity Controls',
              Icons.tune,
              [
                const Text(
                  'Throttle Sensitivity',
                  style: TextStyle(color: Colors.white, fontSize: 16),
                ),
                const SizedBox(height: 8),
                Row(
                  children: [
                    Expanded(
                      child: Slider(
                        value: _throttleSens,
                        min: 0.5,
                        max: 2.0,
                        divisions: 30,
                        activeColor: const Color(0xFFFF9800),
                        label: '${(_throttleSens * 100).toInt()}%',
                        onChanged: (value) {
                          setState(() => _throttleSens = value);
                        },
                      ),
                    ),
                    SizedBox(
                      width: 60,
                      child: Text(
                        '${(_throttleSens * 100).toInt()}%',
                        style: const TextStyle(color: Colors.white, fontSize: 16),
                        textAlign: TextAlign.center,
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 16),
                const Text(
                  'Stick Sensitivity (Roll/Pitch/Yaw)',
                  style: TextStyle(color: Colors.white, fontSize: 16),
                ),
                const SizedBox(height: 8),
                Row(
                  children: [
                    Expanded(
                      child: Slider(
                        value: _stickSens,
                        min: 0.5,
                        max: 2.0,
                        divisions: 30,
                        activeColor: const Color(0xFF4A5FFF),
                        label: '${(_stickSens * 100).toInt()}%',
                        onChanged: (value) {
                          setState(() => _stickSens = value);
                        },
                      ),
                    ),
                    SizedBox(
                      width: 60,
                      child: Text(
                        '${(_stickSens * 100).toInt()}%',
                        style: const TextStyle(color: Colors.white, fontSize: 16),
                        textAlign: TextAlign.center,
                      ),
                    ),
                  ],
                ),
              ],
            ),
            const SizedBox(height: 32),
            if (widget.isConnected)
              Container(
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: Colors.orange.withOpacity(0.2),
                  border: Border.all(color: Colors.orange),
                  borderRadius: BorderRadius.circular(8),
                ),
                child: const Row(
                  children: [
                    Icon(Icons.warning, color: Colors.orange),
                    SizedBox(width: 12),
                    Expanded(
                      child: Text(
                        'Disconnect before changing settings',
                        style: TextStyle(color: Colors.orange),
                      ),
                    ),
                  ],
                ),
              ),
            const SizedBox(height: 24),
            SizedBox(
              width: double.infinity,
              height: 50,
              child: ElevatedButton(
                onPressed: widget.isConnected
                    ? null
                    : () {
                        widget.onIPChanged(_ipController.text);
                        widget.onPortChanged(int.tryParse(_portController.text) ?? 8888);
                        widget.onSensitivityChanged(_throttleSens, _stickSens);
                        Navigator.pop(context);
                        ScaffoldMessenger.of(context).showSnackBar(
                          const SnackBar(
                            content: Text('Settings saved'),
                            backgroundColor: Colors.green,
                          ),
                        );
                      },
                style: ElevatedButton.styleFrom(
                  backgroundColor: const Color(0xFF4A5FFF),
                  disabledBackgroundColor: Colors.grey,
                ),
                child: const Text('Save Settings', style: TextStyle(fontSize: 16)),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildSection(String title, IconData icon, List<Widget> children) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            Icon(icon, color: const Color(0xFF4A5FFF), size: 24),
            const SizedBox(width: 8),
            Text(
              title,
              style: const TextStyle(
                color: Colors.white,
                fontSize: 20,
                fontWeight: FontWeight.bold,
              ),
            ),
          ],
        ),
        const SizedBox(height: 16),
        ...children,
      ],
    );
  }
}

// ==================== TELEMETRY PAGE ====================
class TelemetryPage extends StatelessWidget {
  final int throttle;
  final int roll;
  final int pitch;
  final int yaw;
  final int arm;
  final double altitude;
  final double batteryVoltage;
  final double batteryPercent;
  final int packetsPerSecond;
  final double compassDirection;
  final bool isConnected;
  final bool isESP32Connected;

  const TelemetryPage({
    super.key,
    required this.throttle,
    required this.roll,
    required this.pitch,
    required this.yaw,
    required this.arm,
    required this.altitude,
    required this.batteryVoltage,
    required this.batteryPercent,
    required this.packetsPerSecond,
    required this.compassDirection,
    required this.isConnected,
    required this.isESP32Connected,
  });

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF1A1D2E),
      appBar: AppBar(
        title: const Text('Telemetry & Debug'),
        backgroundColor: const Color(0xFF262B3F),
      ),
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            _buildStatusCard(),
            const SizedBox(height: 16),
            _buildPWMCard(),
            const SizedBox(height: 16),
            _buildBatteryCard(),
            const SizedBox(height: 16),
            _buildNetworkCard(),
          ],
        ),
      ),
    );
  }

  Widget _buildStatusCard() {
    return Card(
      color: const Color(0xFF262B3F),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Flight Status',
              style: TextStyle(
                color: Color(0xFF4A5FFF),
                fontSize: 18,
                fontWeight: FontWeight.bold,
              ),
            ),
            const SizedBox(height: 16),
            _buildStatusRow('Connection', isConnected ? 'CONNECTED' : 'DISCONNECTED',
                isConnected ? Colors.green : Colors.red),
            _buildStatusRow('ESP32', isESP32Connected ? 'ONLINE' : 'OFFLINE',
                isESP32Connected ? Colors.green : Colors.orange),
            _buildStatusRow('Motors', arm == 1 ? 'ARMED' : 'DISARMED',
                arm == 1 ? Colors.yellow : Colors.blue),
            _buildStatusRow('Altitude', '${altitude.toStringAsFixed(2)} m', Colors.cyan),
          ],
        ),
      ),
    );
  }

  Widget _buildPWMCard() {
    return Card(
      color: const Color(0xFF262B3F),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'PWM Values (μs)',
              style: TextStyle(
                color: Color(0xFF4A5FFF),
                fontSize: 18,
                fontWeight: FontWeight.bold,
              ),
            ),
            const SizedBox(height: 16),
            _buildPWMBar('Throttle', throttle, Colors.orange),
            const SizedBox(height: 12),
            _buildPWMBar('Roll', roll, Colors.blue),
            const SizedBox(height: 12),
            _buildPWMBar('Pitch', pitch, Colors.green),
            const SizedBox(height: 12),
            _buildPWMBar('Yaw', yaw, Colors.purple),
          ],
        ),
      ),
    );
  }

  Widget _buildBatteryCard() {
    Color batteryColor = batteryPercent > 50
        ? Colors.green
        : batteryPercent > 20
            ? Colors.orange
            : Colors.red;

    return Card(
      color: const Color(0xFF262B3F),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Battery',
              style: TextStyle(
                color: Color(0xFF4A5FFF),
                fontSize: 18,
                fontWeight: FontWeight.bold,
              ),
            ),
            const SizedBox(height: 16),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Text(
                  '${batteryVoltage.toStringAsFixed(2)}V',
                  style: TextStyle(color: batteryColor, fontSize: 24, fontWeight: FontWeight.bold),
                ),
                Text(
                  '${batteryPercent.toStringAsFixed(0)}%',
                  style: TextStyle(color: batteryColor, fontSize: 24, fontWeight: FontWeight.bold),
                ),
              ],
            ),
            const SizedBox(height: 12),
            ClipRRect(
              borderRadius: BorderRadius.circular(8),
              child: LinearProgressIndicator(
                value: batteryPercent / 100,
                minHeight: 20,
                backgroundColor: Colors.grey[800],
                valueColor: AlwaysStoppedAnimation<Color>(batteryColor),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildNetworkCard() {
    return Card(
      color: const Color(0xFF262B3F),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Network',
              style: TextStyle(
                color: Color(0xFF4A5FFF),
                fontSize: 18,
                fontWeight: FontWeight.bold,
              ),
            ),
            const SizedBox(height: 16),
            _buildStatusRow('Packets/sec', packetsPerSecond.toString(), Colors.cyan),
            _buildStatusRow('Compass', '${compassDirection.toStringAsFixed(0)}°', Colors.purple),
          ],
        ),
      ),
    );
  }

  Widget _buildStatusRow(String label, String value, Color color) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label, style: const TextStyle(color: Colors.white70, fontSize: 14)),
          Text(
            value,
            style: TextStyle(color: color, fontSize: 14, fontWeight: FontWeight.bold),
          ),
        ],
      ),
    );
  }

  Widget _buildPWMBar(String label, int value, Color color) {
    double percentage = ((value - 1000) / 1000);
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Text(label, style: const TextStyle(color: Colors.white, fontSize: 14)),
            Text('$value μs', style: TextStyle(color: color, fontSize: 14, fontWeight: FontWeight.bold)),
          ],
        ),
        const SizedBox(height: 4),
        ClipRRect(
          borderRadius: BorderRadius.circular(4),
          child: LinearProgressIndicator(
            value: percentage,
            minHeight: 12,
            backgroundColor: Colors.grey[800],
            valueColor: AlwaysStoppedAnimation<Color>(color),
          ),
        ),
      ],
    );
  }
}