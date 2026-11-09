import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:math';
import 'dart:typed_data';
import 'dart:ui' as ui;
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:fl_chart/fl_chart.dart';
import 'package:intl/intl.dart';
import 'package:http/http.dart' as http;
import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import 'package:path_provider/path_provider.dart';
import 'package:share_plus/share_plus.dart';
import 'package:shared_preferences/shared_preferences.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  runApp(const UVMonitorApp());
}

class UVMonitorApp extends StatelessWidget {
  const UVMonitorApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'ESP32 UV Sensor Calibration & Cloud Hub',
      theme: ThemeData.dark().copyWith(
        scaffoldBackgroundColor: const Color(0xFF0B0F19),
        colorScheme: const ColorScheme.dark(
          primary: Color(0xFF00F5D4),
          secondary: Color(0xFF7B2CBF),
        ),
      ),
      home: const UVMonitorScreen(),
    );
  }
}

class GuvaCalibrationEngine {
  static const List<double> mvThresholds = <double>[
    50.0, 227.0, 318.0, 408.0, 503.0, 606.0,
    696.0, 795.0, 881.0, 976.0, 1079.0, 1170.0
  ];

  static int lookupDiscreteUVI(double mv) {
    if (mv < 50.0) return 0;
    for (int uvi = 11; uvi >= 1; uvi--) {
      if (mv >= mvThresholds[uvi]) return uvi;
    }
    return 0;
  }

  static double calculateCalibratedUVI(double mv) {
    if (mv < 50.0) return 0.0;
    if (mv >= 1170.0) return 11.0 + (mv - 1170.0) / 91.0;
    for (int i = 0; i < 11; i++) {
      final double vLow = mvThresholds[i];
      final double vHigh = mvThresholds[i + 1];
      if (mv >= vLow && mv <= vHigh) {
        return i + (mv - vLow) / (vHigh - vLow);
      }
    }
    return 0.0;
  }

  static double calculateOldLinearUVI(double mv) => mv / 100.0;

  static String getRiskCategory(double uvi) {
    if (uvi < 3.0) return 'Low';
    if (uvi < 6.0) return 'Moderate';
    if (uvi < 8.0) return 'High';
    if (uvi < 11.0) return 'Very high';
    return 'Extreme';
  }
}

class UVRecord {
  final DateTime timestamp;
  final double mvRaw;
  final double mvMedian;
  final double mvKalman;
  final double mvHybrid;
  final double uviCalibrated;
  final double uviLinearOld;
  final int uviLookupInt;
  final String riskCategory;
  final int rssi;

  UVRecord({
    required this.timestamp,
    required this.mvRaw,
    required this.mvMedian,
    required this.mvKalman,
    required this.mvHybrid,
    required this.uviCalibrated,
    required this.uviLinearOld,
    required this.uviLookupInt,
    required this.riskCategory,
    required this.rssi,
  });

  double getMvByFilter(String filterName) {
    switch (filterName) {
      case 'Median Filter':
        return mvMedian;
      case '1D Kalman Filter':
        return mvKalman;
      case 'Hybrid (Median+Kalman)':
      default:
        return mvHybrid;
    }
  }

  double getUviByFilter(String filterName) {
    return GuvaCalibrationEngine.calculateCalibratedUVI(
        getMvByFilter(filterName));
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'ts': timestamp.toIso8601String(),
        'raw': mvRaw,
        'med': mvMedian,
        'kal': mvKalman,
        'hyb': mvHybrid,
        'uvi_cal': uviCalibrated,
        'uvi_lin': uviLinearOld,
        'uvi_int': uviLookupInt,
        'risk': riskCategory,
        'rssi': rssi,
      };

  factory UVRecord.fromJson(Map<String, dynamic> json) {
    final double hyb =
        ((json['hyb'] ?? json['kal'] ?? json['mv'] ?? 0.0) as num).toDouble();
    final double raw = ((json['raw'] ?? hyb) as num).toDouble();
    final double uviCal = ((json['uvi_cal'] ??
            json['uv'] ??
            GuvaCalibrationEngine.calculateCalibratedUVI(hyb)) as num)
        .toDouble();
    return UVRecord(
      timestamp: DateTime.parse(json['ts'].toString()),
      mvRaw: raw,
      mvMedian: ((json['med'] ?? hyb) as num).toDouble(),
      mvKalman: ((json['kal'] ?? hyb) as num).toDouble(),
      mvHybrid: hyb,
      uviCalibrated: uviCal,
      uviLinearOld: ((json['uvi_lin'] ?? (hyb / 100.0)) as num).toDouble(),
      uviLookupInt: ((json['uvi_int'] ??
              GuvaCalibrationEngine.lookupDiscreteUVI(hyb)) as num)
          .toInt(),
      riskCategory: (json['risk'] ??
              GuvaCalibrationEngine.getRiskCategory(uviCal))
          .toString(),
      rssi: ((json['rssi'] ?? 0) as num).toInt(),
    );
  }
}

class HourlyBucketStat {
  final int hour;
  final String timeSlot;
  final double peakUvi;
  final double avgMv;
  final int count;
  final String risk;

  HourlyBucketStat({
    required this.hour,
    required this.timeSlot,
    required this.peakUvi,
    required this.avgMv,
    required this.count,
    required this.risk,
  });
}

class GranularTelemetryRow {
  final String time;
  final String uvData;
  final String voltage;

  GranularTelemetryRow({
    required this.time,
    required this.uvData,
    required this.voltage,
  });
}

class UVMonitorScreen extends StatefulWidget {
  const UVMonitorScreen({super.key});

  @override
  State<UVMonitorScreen> createState() => _UVMonitorScreenState();
}

class _UVMonitorScreenState extends State<UVMonitorScreen> {
  final TextEditingController _ipController =
      TextEditingController(text: '192.168.0.117');
  final GlobalKey _chartBoundaryKey = GlobalKey();

  Timer? _pollTimer;
  MqttServerClient? _mqttClient;
  bool _isConnected = false;
  bool _isMqttCloudConnected = false;
  bool _isSimulating = false;
  String _status = 'Tap Connect (Auto-syncs via HiveMQ Cloud & Local Wi-Fi)';
  final int _selectedPollSeconds = 5;

  String _selectedFilter = 'Hybrid (Median+Kalman)';
  bool _overlayRawOnChart = true;
  final List<String> _filterOptions = <String>[
    'Hybrid (Median+Kalman)',
    '1D Kalman Filter',
    'Median Filter',
  ];

  final List<double> _winBuf = <double>[];
  double _kalX = 0.0, _kalP = 1.0;
  double _hybX = 0.0, _hybP = 1.0;

  UVRecord? _latestRecord;
  int _packetCount = 0;

  final List<UVRecord> _liveRecords = <UVRecord>[];
  List<UVRecord> _allHistoryRecords = <UVRecord>[];

  String _periodMode = 'Date';
  int _selectedYear = DateTime.now().year;
  int _selectedMonth = DateTime.now().month;
  int _selectedWeek = ((DateTime.now().day - 1) ~/ 7) + 1;
  int _selectedDay = DateTime.now().day;
  String _timeGranularity = 'Seconds';

  static const List<String> _monthNames = <String>[
    'Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun',
    'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec'
  ];

  @override
  void initState() {
    super.initState();
    _loadSavedHistory();
    _connectHiveMqCloud();
  }

  @override
  void dispose() {
    _pollTimer?.cancel();
    _mqttClient?.disconnect();
    _ipController.dispose();
    super.dispose();
  }

  Future<void> _loadSavedHistory() async {
    final SharedPreferences prefs = await SharedPreferences.getInstance();
    final List<String>? savedJson = prefs.getStringList('uv_history_logs_v3');
    if (savedJson != null) {
      setState(() {
        _allHistoryRecords = savedJson
            .map((String str) =>
                UVRecord.fromJson(jsonDecode(str) as Map<String, dynamic>))
            .toList();
      });
    }
  }

  Future<void> _saveRecordToHistory(UVRecord record) async {
    _allHistoryRecords.add(record);
    if (_allHistoryRecords.length > 8000) {
      _allHistoryRecords.removeAt(0);
    }
    final SharedPreferences prefs = await SharedPreferences.getInstance();
    final List<String> encoded =
        _allHistoryRecords.map((UVRecord r) => jsonEncode(r.toJson())).toList();
    await prefs.setStringList('uv_history_logs_v3', encoded);
  }

  Future<void> _connectHiveMqCloud() async {
    final String clientId = 'flutter_uv_${Random().nextInt(99999)}';
    _mqttClient = MqttServerClient.withPort('broker.hivemq.com', clientId, 1883);
    _mqttClient!.keepAlivePeriod = 30;
    _mqttClient!.autoReconnect = true;
    _mqttClient!.onConnected = () {
      if (mounted) setState(() => _isMqttCloudConnected = true);
      _mqttClient!.subscribe(
          'venkat_iot/sensors/uv/node_01', MqttQos.atMostOnce);
    };
    _mqttClient!.onDisconnected = () {
      if (mounted) setState(() => _isMqttCloudConnected = false);
    };

    try {
      await _mqttClient!.connect();
      _mqttClient!.updates?.listen((List<MqttReceivedMessage<MqttMessage>> c) {
        if (!_isConnected || _isSimulating) return;
        final MqttPublishMessage recMess = c[0].payload as MqttPublishMessage;
        final String payload =
            MqttPublishPayload.bytesToStringAsString(recMess.payload.message);
        _parseIncomingJson(payload, source: 'HiveMQ Cloud + Grafana Sync');
      });
    } catch (_) {
      if (mounted) setState(() => _isMqttCloudConnected = false);
    }
  }

  void _toggleConnection() {
    _pollTimer?.cancel();
    setState(() {
      _isSimulating = false;
      _isConnected = !_isConnected;
    });

    if (_isConnected) {
      if (!_isMqttCloudConnected) _connectHiveMqCloud();
      setState(() => _status = 'Syncing via HiveMQ Cloud & Local HTTP...');
      _fetchLocalHttp();
      _pollTimer = Timer.periodic(
        Duration(seconds: _selectedPollSeconds),
        (_) => _fetchLocalHttp(),
      );
    } else {
      setState(() => _status = 'Telemetry Paused');
    }
  }

  Future<void> _fetchLocalHttp() async {
    final String ip = _ipController.text.trim();
    if (ip.isEmpty) return;
    try {
      final http.Response response = await http
          .get(Uri.parse('http://$ip/data'))
          .timeout(const Duration(milliseconds: 2200));

      if (response.statusCode == 200) {
        _parseIncomingJson(response.body, source: 'Local Wi-Fi ($ip) + Cloud');
      }
    } catch (_) {
      if (_isMqttCloudConnected && mounted) {
        setState(() => _status = 'Connected remotely via HiveMQ Cloud Broker');
      }
    }
  }

  void _parseIncomingJson(String body, {required String source}) {
    try {
      final Map<String, dynamic> data =
          jsonDecode(body) as Map<String, dynamic>;
      final double raw = double.tryParse(
              (data['mv_raw'] ?? data['sensor_mv'] ?? '0').toString()) ??
          0.0;
      final int rssi = int.tryParse((data['rssi_dbm'] ?? '0').toString()) ?? 0;

      if (data.containsKey('mv_hybrid')) {
        final double med = double.tryParse(data['mv_median'].toString()) ?? raw;
        final double kal = double.tryParse(data['mv_kalman'].toString()) ?? raw;
        final double hyb = double.tryParse(data['mv_hybrid'].toString()) ?? raw;
        _recordSample(raw, med, kal, hyb, rssi, source: source);
      } else {
        _compute3FiltersLocallyAndRecord(raw, rssi, source: source);
      }
    } catch (_) {}
  }

  void _compute3FiltersLocallyAndRecord(double raw, int rssi,
      {required String source}) {
    _winBuf.add(raw);
    if (_winBuf.length > 9) _winBuf.removeAt(0);

    final List<double> sorted = List<double>.from(_winBuf)..sort();
    final double med = sorted[sorted.length ~/ 2];

    if (_packetCount == 0) _kalX = raw;
    _kalP += 0.05;
    final double kG = _kalP / (_kalP + 2.5);
    _kalX += kG * (raw - _kalX);
    _kalP = (1.0 - kG) * _kalP;

    if (_packetCount == 0) _hybX = med;
    _hybP += 0.05;
    final double hG = _hybP / (_hybP + 1.5);
    _hybX += hG * (med - _hybX);
    _hybP = (1.0 - hG) * _hybP;

    _recordSample(raw, med, _kalX, _hybX, rssi, source: source);
  }

  void _recordSample(
    double raw,
    double med,
    double kal,
    double hyb,
    int rssi, {
    required String source,
  }) {
    if (!mounted) return;

    final double uviCal = GuvaCalibrationEngine.calculateCalibratedUVI(hyb);
    final double uviLin = GuvaCalibrationEngine.calculateOldLinearUVI(hyb);
    final int uviInt = GuvaCalibrationEngine.lookupDiscreteUVI(hyb);
    final String risk = GuvaCalibrationEngine.getRiskCategory(uviCal);

    final UVRecord rec = UVRecord(
      timestamp: DateTime.now(),
      mvRaw: raw,
      mvMedian: med,
      mvKalman: kal,
      mvHybrid: hyb,
      uviCalibrated: uviCal,
      uviLinearOld: uviLin,
      uviLookupInt: uviInt,
      riskCategory: risk,
      rssi: rssi,
    );

    setState(() {
      _latestRecord = rec;
      _packetCount++;
      _status = '$source • #$_packetCount';
      _liveRecords.add(rec);
      if (_liveRecords.length > 60) _liveRecords.removeAt(0);
    });

    _saveRecordToHistory(rec);
  }

  void _toggleDemoStream() {
    _pollTimer?.cancel();
    setState(() {
      _isConnected = false;
      _isSimulating = !_isSimulating;
      if (_isSimulating) {
        _generateDemoPoint();
        _pollTimer = Timer.periodic(
          Duration(seconds: _selectedPollSeconds),
          (_) => _generateDemoPoint(),
        );
      } else {
        _status = 'Demo Stopped';
      }
    });
  }

  void _generateDemoPoint() {
    final Random rng = Random();
    final double baseMv = 380.0 + sin(_packetCount * 0.2) * 260.0;
    final double whiteNoise = (rng.nextDouble() - 0.5) * 30.0;
    final double wifiSpike = (rng.nextInt(4) == 0) ? 165.0 : 0.0;
    final double raw = (baseMv + whiteNoise + wifiSpike).clamp(20.0, 1250.0);
    _compute3FiltersLocallyAndRecord(raw, -51,
        source: 'Demo Noise Filter Test');
  }

  double _calcFilterStdDev(String filterName) {
    if (_liveRecords.length < 3) return 0.0;
    final List<double> vals =
        _liveRecords.map((UVRecord r) => r.getMvByFilter(filterName)).toList();
    final List<double> diffs = <double>[];
    for (int i = 1; i < vals.length; i++) {
      diffs.add(vals[i] - vals[i - 1]);
    }
    final double mean =
        diffs.reduce((double a, double b) => a + b) / diffs.length;
    final double variance = diffs
            .map((double d) => pow(d - mean, 2).toDouble())
            .reduce((double a, double b) => a + b) /
        diffs.length;
    return sqrt(variance);
  }

  int _daysInMonth(int year, int month) => DateTime(year, month + 1, 0).day;

  List<UVRecord> _getFilteredRecords() {
    return _allHistoryRecords.where((UVRecord r) {
      final DateTime ts = r.timestamp;
      if (_periodMode == 'Year') {
        return ts.year == _selectedYear;
      } else if (_periodMode == 'Month') {
        return ts.year == _selectedYear && ts.month == _selectedMonth;
      } else if (_periodMode == 'Week') {
        final int wk = ((ts.day - 1) ~/ 7) + 1;
        return ts.year == _selectedYear &&
            ts.month == _selectedMonth &&
            wk == _selectedWeek;
      } else {
        return ts.year == _selectedYear &&
            ts.month == _selectedMonth &&
            ts.day == _selectedDay;
      }
    }).toList();
  }

  List<HourlyBucketStat> _get24HrDescendingList() {
    final Iterable<UVRecord> dayRecords = _allHistoryRecords.where(
        (UVRecord r) =>
            r.timestamp.year == _selectedYear &&
            r.timestamp.month == _selectedMonth &&
            r.timestamp.day == _selectedDay);

    final Map<int, List<UVRecord>> byHour = <int, List<UVRecord>>{
      for (int h = 0; h < 24; h++) h: <UVRecord>[]
    };
    for (final UVRecord r in dayRecords) {
      byHour[r.timestamp.hour]?.add(r);
    }

    final List<HourlyBucketStat> list24 = <HourlyBucketStat>[];
    for (int h = 0; h < 24; h++) {
      final List<UVRecord> items = byHour[h]!;
      double peakUvi = 0.0;
      double avgMv = 0.0;
      if (items.isNotEmpty) {
        for (final UVRecord item in items) {
          final double u = item.getUviByFilter(_selectedFilter);
          if (u > peakUvi) peakUvi = u;
          avgMv += item.getMvByFilter(_selectedFilter);
        }
        avgMv /= items.length;
      }
      list24.add(HourlyBucketStat(
        hour: h,
        timeSlot:
            '${h.toString().padLeft(2, '0')}:00 - ${h.toString().padLeft(2, '0')}:59',
        peakUvi: peakUvi,
        avgMv: avgMv,
        count: items.length,
        risk: GuvaCalibrationEngine.getRiskCategory(peakUvi),
      ));
    }

    list24.sort((HourlyBucketStat a, HourlyBucketStat b) =>
        b.peakUvi.compareTo(a.peakUvi));
    return list24;
  }

  List<GranularTelemetryRow> _getGranularityTableRows() {
    final List<UVRecord> source = _getFilteredRecords();
    if (source.isEmpty) return <GranularTelemetryRow>[];

    final Map<String, List<UVRecord>> bucketed = <String, List<UVRecord>>{};
    for (final UVRecord r in source.reversed) {
      String key;
      if (_timeGranularity == 'Seconds') {
        key = DateFormat('yyyy-MM-dd HH:mm:ss').format(r.timestamp);
      } else if (_timeGranularity == 'Minutes') {
        key = DateFormat('yyyy-MM-dd HH:mm').format(r.timestamp);
      } else if (_timeGranularity == 'Hours') {
        key = DateFormat('yyyy-MM-dd HH:00').format(r.timestamp);
      } else {
        key = DateFormat('yyyy-MM-dd').format(r.timestamp);
      }
      bucketed.putIfAbsent(key, () => <UVRecord>[]).add(r);
    }

    return bucketed.entries.take(100).map((MapEntry<String, List<UVRecord>> e) {
      final List<UVRecord> recs = e.value;
      final double avgMv = recs
              .map((UVRecord r) => r.getMvByFilter(_selectedFilter))
              .reduce((double a, double b) => a + b) /
          recs.length;
      final double avgUvi = GuvaCalibrationEngine.calculateCalibratedUVI(avgMv);
      return GranularTelemetryRow(
        time: e.key,
        uvData:
            '${avgUvi.toStringAsFixed(2)} UVI (${GuvaCalibrationEngine.getRiskCategory(avgUvi)})',
        voltage: '${avgMv.toStringAsFixed(1)} mV',
      );
    }).toList();
  }

  Future<void> _downloadGranularityCsv() async {
    final List<GranularTelemetryRow> rows = _getGranularityTableRows();
    if (rows.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
            content: Text('No data in current selection to download!')),
      );
      return;
    }

    final StringBuffer csv = StringBuffer();
    csv.writeln(
        'Time ($_timeGranularity),UV_Index_Calibrated,Voltage_mV,Filter_Used');
    for (final GranularTelemetryRow r in rows) {
      csv.writeln('${r.time},${r.uvData},${r.voltage},$_selectedFilter');
    }

    final Directory dir = await getTemporaryDirectory();
    final File file = File(
        '${dir.path}/uv_${_timeGranularity.toLowerCase()}_${DateFormat('yyyyMMdd_HHmmss').format(DateTime.now())}.csv');
    await file.writeAsString(csv.toString());
    await Share.shareXFiles(<XFile>[XFile(file.path)],
        text: 'UV Sensor Data ($_timeGranularity Granularity)');
  }

  Future<void> _download3FilterComparisonCsv() async {
    final List<UVRecord> list = _getFilteredRecords().isNotEmpty
        ? _getFilteredRecords()
        : _allHistoryRecords;
    if (list.isEmpty) return;

    final StringBuffer csv = StringBuffer();
    csv.writeln(
        'Timestamp,Raw_mV,Median_mV,Kalman_mV,Hybrid_MedKal_mV,UVI_Calibrated,UVI_Lookup_Int,UVI_Linear_Old,Risk_Category');

    final DateFormat df = DateFormat('yyyy-MM-dd HH:mm:ss');
    for (final UVRecord r in list) {
      csv.writeln(
        '${df.format(r.timestamp)},'
        '${r.mvRaw.toStringAsFixed(2)},'
        '${r.mvMedian.toStringAsFixed(2)},'
        '${r.mvKalman.toStringAsFixed(2)},'
        '${r.mvHybrid.toStringAsFixed(2)},'
        '${r.uviCalibrated.toStringAsFixed(3)},'
        '${r.uviLookupInt},'
        '${r.uviLinearOld.toStringAsFixed(3)},'
        '${r.riskCategory}',
      );
    }

    final Directory dir = await getTemporaryDirectory();
    final File file = File(
        '${dir.path}/uv_3filter_comparison_${DateFormat('yyyyMMdd_HHmmss').format(DateTime.now())}.csv');
    await file.writeAsString(csv.toString());
    await Share.shareXFiles(<XFile>[XFile(file.path)],
        text: '3-Filter UV Comparison Dataset (Hybrid, Kalman, Median)');
  }

  Future<void> _downloadGraphPng() async {
    try {
      final RenderRepaintBoundary? boundary = _chartBoundaryKey.currentContext
          ?.findRenderObject() as RenderRepaintBoundary?;
      if (boundary == null) return;
      final ui.Image image = await boundary.toImage(pixelRatio: 3.0);
      final ByteData? byteData =
          await image.toByteData(format: ui.ImageByteFormat.png);
      if (byteData == null) return;

      final Directory dir = await getTemporaryDirectory();
      final File file = File(
          '${dir.path}/uv_graph_${DateFormat('yyyyMMdd_HHmmss').format(DateTime.now())}.png');
      await file.writeAsBytes(byteData.buffer.asUint8List());
      await Share.shareXFiles(<XFile>[XFile(file.path)],
          text: 'UV Sensor Graph Image ($_selectedFilter)');
    } catch (e) {
      if (!mounted) return;
      ScaffoldMessenger.of(context)
          .showSnackBar(SnackBar(content: Text('Graph Export Error: $e')));
    }
  }

  Future<void> _showPhoneStyleWiFiDialog() async {
    final TextEditingController ssidCtrl = TextEditingController();
    final TextEditingController passCtrl = TextEditingController();
    final TextEditingController influxUrlCtrl = TextEditingController();
    final TextEditingController influxTokCtrl = TextEditingController();
    List<dynamic> scannedNetworks = <dynamic>[];
    bool scanning = true;

    await showDialog<void>(
      context: context,
      builder: (BuildContext ctx) => StatefulBuilder(
        builder: (BuildContext ctx, StateSetter setDialogState) {
          if (scanning) {
            scanning = false;
            final String ip = _ipController.text.trim();
            http
                .get(Uri.parse('http://$ip/scan'))
                .timeout(const Duration(seconds: 4))
                .then((http.Response res) {
              if (res.statusCode == 200) {
                final Map<String, dynamic> data =
                    jsonDecode(res.body) as Map<String, dynamic>;
                setDialogState(() {
                  scannedNetworks =
                      (data['networks'] as List<dynamic>?) ?? <dynamic>[];
                });
              }
            }).catchError((Object _) {});
          }

          return AlertDialog(
            backgroundColor: const Color(0xFF151F32),
            title: const Row(
              children: <Widget>[
                Icon(Icons.wifi_rounded, color: Color(0xFF00F5D4)),
                SizedBox(width: 8),
                Expanded(
                  child: Text('Connect ESP32 to Any Wi-Fi & Cloud',
                      style:
                          TextStyle(fontSize: 15, fontWeight: FontWeight.bold)),
                ),
              ],
            ),
            content: SingleChildScrollView(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  if (scannedNetworks.isNotEmpty) ...<Widget>[
                    const Text('Tap an Available Network Found by ESP32:',
                        style:
                            TextStyle(fontSize: 11, color: Color(0xFF00F5D4))),
                    const SizedBox(height: 6),
                    SizedBox(
                      height: 110,
                      width: double.maxFinite,
                      child: ListView.builder(
                        itemCount: scannedNetworks.length,
                        itemBuilder: (BuildContext _, int i) {
                          final Map<String, dynamic> net =
                              scannedNetworks[i] as Map<String, dynamic>;
                          return ListTile(
                            dense: true,
                            contentPadding: EdgeInsets.zero,
                            leading: const Icon(Icons.network_wifi,
                                color: Colors.white70, size: 18),
                            title: Text('${net['ssid']}',
                                style: const TextStyle(fontSize: 12)),
                            trailing: Text('${net['rssi']} dBm',
                                style: const TextStyle(
                                    fontSize: 10, color: Colors.white54)),
                            onTap: () {
                              setDialogState(() {
                                ssidCtrl.text = net['ssid'].toString();
                              });
                            },
                          );
                        },
                      ),
                    ),
                    const Divider(color: Colors.white24),
                  ],
                  TextField(
                    controller: ssidCtrl,
                    decoration: const InputDecoration(
                      labelText: 'Wi-Fi / Hotspot Name (SSID)',
                      isDense: true,
                      border: OutlineInputBorder(),
                    ),
                  ),
                  const SizedBox(height: 10),
                  TextField(
                    controller: passCtrl,
                    obscureText: true,
                    decoration: const InputDecoration(
                      labelText: 'Wi-Fi Password',
                      isDense: true,
                      border: OutlineInputBorder(),
                    ),
                  ),
                  const SizedBox(height: 12),
                  const Text('Optional InfluxDB Cloud (For Live Grafana):',
                      style: TextStyle(fontSize: 11, color: Colors.white60)),
                  const SizedBox(height: 6),
                  TextField(
                    controller: influxUrlCtrl,
                    decoration: const InputDecoration(
                      labelText: 'InfluxDB Write URL',
                      isDense: true,
                      border: OutlineInputBorder(),
                    ),
                  ),
                  const SizedBox(height: 8),
                  TextField(
                    controller: influxTokCtrl,
                    decoration: const InputDecoration(
                      labelText: 'InfluxDB API Token',
                      isDense: true,
                      border: OutlineInputBorder(),
                    ),
                  ),
                ],
              ),
            ),
            actions: <Widget>[
              TextButton(
                onPressed: () => Navigator.pop(ctx),
                child: const Text('Cancel'),
              ),
              ElevatedButton(
                style: ElevatedButton.styleFrom(
                  backgroundColor: const Color(0xFF00F5D4),
                  foregroundColor: Colors.black,
                ),
                onPressed: () async {
                  final String ip = _ipController.text.trim();
                  final NavigatorState nav = Navigator.of(ctx);
                  final ScaffoldMessengerState messenger =
                      ScaffoldMessenger.of(context);
                  try {
                    await http.post(
                      Uri.parse('http://$ip/wifi'),
                      body: <String, String>{
                        'ssid': ssidCtrl.text.trim(),
                        'pass': passCtrl.text,
                        'influx_url': influxUrlCtrl.text.trim(),
                        'influx_token': influxTokCtrl.text.trim(),
                      },
                    );
                    nav.pop();
                    messenger.showSnackBar(
                      const SnackBar(
                          content: Text(
                              'Saved! ESP32 is connecting to the new Wi-Fi & syncing via HiveMQ Cloud.')),
                    );
                  } catch (e) {
                    messenger.showSnackBar(
                      const SnackBar(
                          content: Text(
                              'Tip: If switching away from old Wi-Fi, connect phone to "ESP32_UV_WiFi_Setup" AP (IP 192.168.4.1) first.')),
                    );
                  }
                },
                child: const Text('Connect ESP32'),
              ),
            ],
          );
        },
      ),
    );
  }

  Color _getRiskColor(double uvi) {
    if (uvi < 3.0) return const Color(0xFF00F5D4);
    if (uvi < 6.0) return const Color(0xFFFEE440);
    if (uvi < 8.0) return const Color(0xFFFF9E00);
    if (uvi < 11.0) return const Color(0xFFFF0055);
    return const Color(0xFFB5179E);
  }

  @override
  Widget build(BuildContext context) {
    final double activeMv =
        _latestRecord?.getMvByFilter(_selectedFilter) ?? 0.0;
    final double activeUvi =
        GuvaCalibrationEngine.calculateCalibratedUVI(activeMv);
    final int discreteUvi = GuvaCalibrationEngine.lookupDiscreteUVI(activeMv);
    final double oldLinearUvi =
        GuvaCalibrationEngine.calculateOldLinearUVI(activeMv);
    final String activeRisk = GuvaCalibrationEngine.getRiskCategory(activeUvi);
    final Color activeColor = _getRiskColor(activeUvi);

    final List<HourlyBucketStat> hourlyRanked = _get24HrDescendingList();
    final List<GranularTelemetryRow> granularRows = _getGranularityTableRows();
    final int currentYear = DateTime.now().year;
    final List<int> yearOptions =
        List<int>.generate((currentYear - 2020) + 1, (int i) => 2020 + i);
    final int maxDays = _daysInMonth(_selectedYear, _selectedMonth);
    if (_selectedDay > maxDays) _selectedDay = maxDays;

    return Scaffold(
      appBar: AppBar(
        backgroundColor: const Color(0xFF131B2E),
        title: const Text(
          'UV Calibration, Filter Lab & Cloud Hub',
          style: TextStyle(fontWeight: FontWeight.w800, fontSize: 15),
        ),
        actions: <Widget>[
          IconButton(
            tooltip: 'Connect ESP32 to Any Wi-Fi / Hotspot',
            icon: const Icon(Icons.wifi_password_rounded,
                color: Color(0xFF00F5D4)),
            onPressed: _showPhoneStyleWiFiDialog,
          ),
          TextButton.icon(
            onPressed: _toggleDemoStream,
            icon: Icon(
              _isSimulating ? Icons.stop_circle : Icons.science_rounded,
              color: _isSimulating
                  ? const Color(0xFFFF0055)
                  : const Color(0xFFFEE440),
            ),
            label: Text(
              _isSimulating ? 'Stop' : 'Demo',
              style: const TextStyle(
                  color: Colors.white, fontWeight: FontWeight.bold),
            ),
          ),
        ],
      ),
      body: SafeArea(
        child: SingleChildScrollView(
          padding: const EdgeInsets.all(12.0),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: <Widget>[
              Container(
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: const Color(0xFF151F32),
                  borderRadius: BorderRadius.circular(14),
                  border: Border.all(color: Colors.white10),
                ),
                child: Column(
                  children: <Widget>[
                    Row(
                      children: <Widget>[
                        Expanded(
                          child: TextField(
                            controller: _ipController,
                            style: const TextStyle(
                                color: Colors.white,
                                fontWeight: FontWeight.bold,
                                fontSize: 13),
                            decoration: InputDecoration(
                              labelText:
                                  'ESP32 Local IP (or 192.168.4.1 Setup)',
                              prefixIcon: const Icon(Icons.router_rounded,
                                  color: Color(0xFF00F5D4), size: 18),
                              filled: true,
                              fillColor: const Color(0xFF0B0F19),
                              border: OutlineInputBorder(
                                borderRadius: BorderRadius.circular(10),
                                borderSide: BorderSide.none,
                              ),
                              isDense: true,
                            ),
                          ),
                        ),
                        const SizedBox(width: 8),
                        ElevatedButton(
                          style: ElevatedButton.styleFrom(
                            backgroundColor: _isConnected
                                ? const Color(0xFFFF0055)
                                : const Color(0xFF00F5D4),
                            foregroundColor:
                                _isConnected ? Colors.white : Colors.black,
                            padding: const EdgeInsets.symmetric(
                                horizontal: 16, vertical: 14),
                          ),
                          onPressed: _toggleConnection,
                          child: Text(
                            _isConnected ? 'Stop' : 'Connect',
                            style: const TextStyle(fontWeight: FontWeight.w900),
                          ),
                        ),
                      ],
                    ),
                    const SizedBox(height: 8),
                    Row(
                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                      children: <Widget>[
                        Expanded(
                          child: Text(_status,
                              style: const TextStyle(
                                  fontSize: 11, color: Colors.white70),
                              overflow: TextOverflow.ellipsis),
                        ),
                        Container(
                          padding: const EdgeInsets.symmetric(
                              horizontal: 8, vertical: 3),
                          decoration: BoxDecoration(
                            color: (_isMqttCloudConnected
                                    ? const Color(0xFF00F5D4)
                                    : Colors.orange)
                                .withOpacity(0.15),
                            borderRadius: BorderRadius.circular(8),
                          ),
                          child: Text(
                            _isMqttCloudConnected
                                ? 'HiveMQ Cloud: ONLINE'
                                : 'HiveMQ: Connecting...',
                            style: TextStyle(
                              fontSize: 10,
                              fontWeight: FontWeight.bold,
                              color: _isMqttCloudConnected
                                  ? const Color(0xFF00F5D4)
                                  : Colors.orangeAccent,
                            ),
                          ),
                        ),
                      ],
                    ),
                    const Divider(color: Colors.white12, height: 16),
                    Row(
                      children: <Widget>[
                        const Icon(Icons.tune_rounded,
                            color: Color(0xFFFEE440), size: 18),
                        const SizedBox(width: 6),
                        const Text('Active Filter:',
                            style: TextStyle(
                                fontSize: 12, fontWeight: FontWeight.bold)),
                        const SizedBox(width: 8),
                        Expanded(
                          child: DropdownButtonHideUnderline(
                            child: DropdownButton<String>(
                              value: _selectedFilter,
                              isExpanded: true,
                              dropdownColor: const Color(0xFF1E293B),
                              items: _filterOptions
                                  .map((String f) => DropdownMenuItem<String>(
                                        value: f,
                                        child: Text(f,
                                            style: const TextStyle(
                                                fontSize: 12,
                                                color: Color(0xFF00F5D4),
                                                fontWeight: FontWeight.bold)),
                                      ))
                                  .toList(),
                              onChanged: (String? val) {
                                if (val != null) {
                                  setState(() => _selectedFilter = val);
                                }
                              },
                            ),
                          ),
                        ),
                        Checkbox(
                          value: _overlayRawOnChart,
                          activeColor: const Color(0xFFFF0055),
                          onChanged: (bool? v) =>
                              setState(() => _overlayRawOnChart = v ?? true),
                        ),
                        const Text('vs Raw', style: TextStyle(fontSize: 11)),
                      ],
                    ),
                  ],
                ),
              ),
              const SizedBox(height: 12),
              Row(
                children: <Widget>[
                  Expanded(
                    child: _metricCard(
                      title: 'Filtered Voltage',
                      value: '${activeMv.toStringAsFixed(1)} mV',
                      sub:
                          'Raw: ${(_latestRecord?.mvRaw ?? 0).toStringAsFixed(1)} mV',
                      accent: const Color(0xFF00BBF9),
                    ),
                  ),
                  const SizedBox(width: 8),
                  Expanded(
                    child: _metricCard(
                      title: 'Calibrated UVI',
                      value: activeUvi.toStringAsFixed(2),
                      sub: 'Lookup Step: $discreteUvi',
                      accent: activeColor,
                    ),
                  ),
                  const SizedBox(width: 8),
                  Expanded(
                    child: _metricCard(
                      title: 'WHO Risk Category',
                      value: activeRisk,
                      sub: 'Old Lin: ${oldLinearUvi.toStringAsFixed(2)}',
                      accent: const Color(0xFFF15BB5),
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 12),
              Container(
                padding: const EdgeInsets.all(10),
                decoration: BoxDecoration(
                  color: const Color(0xFF151F32),
                  borderRadius: BorderRadius.circular(12),
                  border: Border.all(color: Colors.white10),
                ),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: <Widget>[
                    const Text(
                      'Live Filter Noise Jitter Comparison (Lower σ = Cleaner Signal):',
                      style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.bold,
                          color: Color(0xFFFEE440)),
                    ),
                    const SizedBox(height: 6),
                    SingleChildScrollView(
                      scrollDirection: Axis.horizontal,
                      child: Row(
                        children: _filterOptions.map((String fName) {
                          final double sigma = _calcFilterStdDev(fName);
                          final double val =
                              _latestRecord?.getMvByFilter(fName) ?? 0.0;
                          final bool isSelected = _selectedFilter == fName;
                          return Container(
                            margin: const EdgeInsets.only(right: 8),
                            padding: const EdgeInsets.symmetric(
                                horizontal: 10, vertical: 6),
                            decoration: BoxDecoration(
                              color: isSelected
                                  ? const Color(0xFF00F5D4).withOpacity(0.15)
                                  : const Color(0xFF0B0F19),
                              borderRadius: BorderRadius.circular(8),
                              border: Border.all(
                                color: isSelected
                                    ? const Color(0xFF00F5D4)
                                    : Colors.white12,
                              ),
                            ),
                            child: Column(
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: <Widget>[
                                Text(fName,
                                    style: const TextStyle(
                                        fontSize: 10,
                                        fontWeight: FontWeight.bold)),
                                Text(
                                  '${val.toStringAsFixed(1)} mV  (σ=${sigma.toStringAsFixed(2)})',
                                  style: const TextStyle(
                                      fontSize: 10, color: Color(0xFF00F5D4)),
                                ),
                              ],
                            ),
                          );
                        }).toList(),
                      ),
                    ),
                  ],
                ),
              ),
              const SizedBox(height: 14),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: <Widget>[
                  const Text(
                    'Live UV Index Graph (Filtered vs Unfiltered)',
                    style: TextStyle(fontSize: 13, fontWeight: FontWeight.w800),
                  ),
                  TextButton.icon(
                    onPressed: _downloadGraphPng,
                    icon: const Icon(Icons.download_for_offline_rounded,
                        size: 16, color: Color(0xFF00F5D4)),
                    label: const Text('Download Graph PNG',
                        style:
                            TextStyle(fontSize: 11, color: Color(0xFF00F5D4))),
                  ),
                ],
              ),
              RepaintBoundary(
                key: _chartBoundaryKey,
                child: Container(
                  height: 210,
                  padding: const EdgeInsets.fromLTRB(8, 16, 16, 8),
                  decoration: BoxDecoration(
                    color: const Color(0xFF151F32),
                    borderRadius: BorderRadius.circular(14),
                    border: Border.all(color: Colors.white10),
                  ),
                  child: _liveRecords.isEmpty
                      ? const Center(
                          child: Text(
                            'Tap "Connect" or "Demo" to compare Filtered vs Unfiltered UV',
                            style:
                                TextStyle(color: Colors.white54, fontSize: 12),
                          ),
                        )
                      : LineChart(
                          LineChartData(
                            gridData: FlGridData(
                              show: true,
                              drawVerticalLine: false,
                              getDrawingHorizontalLine: (double v) => FlLine(
                                  color: Colors.white.withOpacity(0.06),
                                  strokeWidth: 1),
                            ),
                            borderData: FlBorderData(show: false),
                            titlesData: FlTitlesData(
                              topTitles: const AxisTitles(
                                  sideTitles: SideTitles(showTitles: false)),
                              rightTitles: const AxisTitles(
                                  sideTitles: SideTitles(showTitles: false)),
                              bottomTitles: AxisTitles(
                                sideTitles: SideTitles(
                                  showTitles: true,
                                  reservedSize: 22,
                                  interval: (_liveRecords.length > 5)
                                      ? (_liveRecords.length / 4)
                                          .floorToDouble()
                                      : 1,
                                  getTitlesWidget:
                                      (double val, TitleMeta meta) {
                                    final int idx = val.toInt();
                                    if (idx >= 0 && idx < _liveRecords.length) {
                                      return Padding(
                                        padding:
                                            const EdgeInsets.only(top: 5.0),
                                        child: Text(
                                          DateFormat('HH:mm:ss').format(
                                              _liveRecords[idx].timestamp),
                                          style: const TextStyle(
                                              fontSize: 9,
                                              color: Colors.white60),
                                        ),
                                      );
                                    }
                                    return const SizedBox.shrink();
                                  },
                                ),
                              ),
                            ),
                            lineBarsData: <LineChartBarData>[
                              if (_overlayRawOnChart)
                                LineChartBarData(
                                  spots: _liveRecords
                                      .asMap()
                                      .entries
                                      .map((MapEntry<int, UVRecord> e) => FlSpot(
                                          e.key.toDouble(),
                                          GuvaCalibrationEngine
                                              .calculateCalibratedUVI(
                                                  e.value.mvRaw)))
                                      .toList(),
                                  isCurved: false,
                                  color:
                                      const Color(0xFFFF0055).withOpacity(0.55),
                                  barWidth: 1.5,
                                  dashArray: <int>[4, 4],
                                  dotData: const FlDotData(show: false),
                                ),
                              LineChartBarData(
                                spots: _liveRecords
                                    .asMap()
                                    .entries
                                    .map((MapEntry<int, UVRecord> e) => FlSpot(
                                        e.key.toDouble(),
                                        e.value
                                            .getUviByFilter(_selectedFilter)))
                                    .toList(),
                                isCurved: true,
                                color: activeColor,
                                barWidth: 3.0,
                                dotData: const FlDotData(show: true),
                              ),
                            ],
                          ),
                        ),
                ),
              ),
              const SizedBox(height: 16),
              Container(
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: const Color(0xFF151F32),
                  borderRadius: BorderRadius.circular(14),
                  border: Border.all(color: Colors.white10),
                ),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: <Widget>[
                    const Text(
                      'Select Filter Period (Date / Week 1–5 / Month Jan–Dec / Year 2020–Present)',
                      style:
                          TextStyle(fontSize: 12, fontWeight: FontWeight.w800),
                    ),
                    const SizedBox(height: 8),
                    Row(
                      children: <String>['Date', 'Week', 'Month', 'Year']
                          .map((String mode) {
                        final bool selected = _periodMode == mode;
                        return Expanded(
                          child: Padding(
                            padding:
                                const EdgeInsets.symmetric(horizontal: 2.0),
                            child: ChoiceChip(
                              label: SizedBox(
                                width: double.infinity,
                                child: Text(
                                  mode,
                                  textAlign: TextAlign.center,
                                  style: TextStyle(
                                    fontSize: 11,
                                    fontWeight: FontWeight.bold,
                                    color: selected
                                        ? Colors.black
                                        : Colors.white70,
                                  ),
                                ),
                              ),
                              selected: selected,
                              selectedColor: const Color(0xFF00F5D4),
                              backgroundColor: const Color(0xFF0B0F19),
                              onSelected: (bool _) =>
                                  setState(() => _periodMode = mode),
                            ),
                          ),
                        );
                      }).toList(),
                    ),
                    const SizedBox(height: 10),
                    Wrap(
                      spacing: 8,
                      runSpacing: 8,
                      children: <Widget>[
                        _buildDropdownChip<int>(
                          label: 'Year',
                          value: _selectedYear,
                          items: yearOptions
                              .map((int y) => DropdownMenuItem<int>(
                                  value: y, child: Text('$y')))
                              .toList(),
                          onChanged: (int? val) => setState(
                              () => _selectedYear = val ?? currentYear),
                        ),
                        if (_periodMode != 'Year')
                          _buildDropdownChip<int>(
                            label: 'Month',
                            value: _selectedMonth,
                            items: List<DropdownMenuItem<int>>.generate(
                              12,
                              (int i) => DropdownMenuItem<int>(
                                value: i + 1,
                                child: Text(_monthNames[i]),
                              ),
                            ),
                            onChanged: (int? val) =>
                                setState(() => _selectedMonth = val ?? 1),
                          ),
                        if (_periodMode == 'Week')
                          _buildDropdownChip<int>(
                            label: 'Week',
                            value: _selectedWeek,
                            items: List<DropdownMenuItem<int>>.generate(
                              5,
                              (int i) => DropdownMenuItem<int>(
                                value: i + 1,
                                child: Text('Week ${i + 1}'),
                              ),
                            ),
                            onChanged: (int? val) =>
                                setState(() => _selectedWeek = val ?? 1),
                          ),
                        if (_periodMode == 'Date')
                          _buildDropdownChip<int>(
                            label: 'Date',
                            value: _selectedDay,
                            items: List<DropdownMenuItem<int>>.generate(
                              maxDays,
                              (int i) => DropdownMenuItem<int>(
                                value: i + 1,
                                child: Text(
                                    '${i + 1} ${_monthNames[_selectedMonth - 1]}'),
                              ),
                            ),
                            onChanged: (int? val) =>
                                setState(() => _selectedDay = val ?? 1),
                          ),
                      ],
                    ),
                  ],
                ),
              ),
              const SizedBox(height: 16),
              Text(
                '24-Hour UV Ranking for $_selectedDay ${_monthNames[_selectedMonth - 1]} $_selectedYear (Descending Order)',
                style:
                    const TextStyle(fontSize: 13, fontWeight: FontWeight.w800),
              ),
              const SizedBox(height: 4),
              const Text(
                'Highest UV radiation hour is highlighted dark at the front; ordered from highest to lowest.',
                style: TextStyle(fontSize: 10, color: Colors.white54),
              ),
              const SizedBox(height: 8),
              Container(
                height: 230,
                decoration: BoxDecoration(
                  color: const Color(0xFF151F32),
                  borderRadius: BorderRadius.circular(14),
                  border: Border.all(color: Colors.white10),
                ),
                child: ListView.builder(
                  itemCount: hourlyRanked.length,
                  itemBuilder: (BuildContext ctx, int idx) {
                    final HourlyBucketStat hr = hourlyRanked[idx];
                    final double peakUvi = hr.peakUvi;
                    final bool isHighestHour = idx == 0 && peakUvi > 0;

                    return Container(
                      margin: EdgeInsets.fromLTRB(
                          8, isHighestHour ? 8 : 3, 8, isHighestHour ? 6 : 3),
                      padding: EdgeInsets.symmetric(
                          horizontal: 12, vertical: isHighestHour ? 13 : 9),
                      decoration: BoxDecoration(
                        color: isHighestHour
                            ? const Color(0xFF04060A)
                            : const Color(0xFF1E293B).withOpacity(0.45),
                        borderRadius: BorderRadius.circular(10),
                        border: Border.all(
                          color: isHighestHour
                              ? const Color(0xFFFF0055)
                              : Colors.white10,
                          width: isHighestHour ? 2.0 : 0.8,
                        ),
                        boxShadow: isHighestHour
                            ? <BoxShadow>[
                                BoxShadow(
                                  color:
                                      const Color(0xFFFF0055).withOpacity(0.25),
                                  blurRadius: 8,
                                )
                              ]
                            : <BoxShadow>[],
                      ),
                      child: Row(
                        mainAxisAlignment: MainAxisAlignment.spaceBetween,
                        children: <Widget>[
                          Row(
                            children: <Widget>[
                              Container(
                                padding: const EdgeInsets.symmetric(
                                    horizontal: 7, vertical: 3),
                                decoration: BoxDecoration(
                                  color: isHighestHour
                                      ? const Color(0xFFFF0055)
                                      : Colors.white10,
                                  borderRadius: BorderRadius.circular(6),
                                ),
                                child: Text(
                                  isHighestHour
                                      ? '★ #1 HIGHEST HOUR'
                                      : '#${idx + 1}',
                                  style: TextStyle(
                                    fontSize: 10,
                                    fontWeight: FontWeight.w900,
                                    color: isHighestHour
                                        ? Colors.white
                                        : Colors.white70,
                                  ),
                                ),
                              ),
                              const SizedBox(width: 10),
                              Text(
                                hr.timeSlot,
                                style: TextStyle(
                                  fontWeight: isHighestHour
                                      ? FontWeight.w900
                                      : FontWeight.w600,
                                  fontSize: isHighestHour ? 13 : 12,
                                ),
                              ),
                            ],
                          ),
                          Text(
                            'UVI: ${peakUvi.toStringAsFixed(2)} | ${hr.avgMv.toStringAsFixed(1)} mV',
                            style: TextStyle(
                              fontSize: 11,
                              fontWeight: FontWeight.bold,
                              color: _getRiskColor(peakUvi),
                            ),
                          ),
                        ],
                      ),
                    );
                  },
                ),
              ),
              const SizedBox(height: 16),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: <Widget>[
                  const Text(
                    'Telemetry Data (Time, UV Data, Voltage)',
                    style: TextStyle(fontSize: 13, fontWeight: FontWeight.w800),
                  ),
                  Container(
                    padding: const EdgeInsets.symmetric(horizontal: 8),
                    decoration: BoxDecoration(
                      color: const Color(0xFF151F32),
                      borderRadius: BorderRadius.circular(8),
                      border: Border.all(color: const Color(0xFF00F5D4)),
                    ),
                    child: DropdownButtonHideUnderline(
                      child: DropdownButton<String>(
                        value: _timeGranularity,
                        dropdownColor: const Color(0xFF1E293B),
                        items: <String>['Seconds', 'Minutes', 'Hours', 'Day']
                            .map((String unit) => DropdownMenuItem<String>(
                                  value: unit,
                                  child: Text(unit,
                                      style: const TextStyle(
                                          fontSize: 12,
                                          color: Color(0xFF00F5D4),
                                          fontWeight: FontWeight.bold)),
                                ))
                            .toList(),
                        onChanged: (String? v) {
                          if (v != null) setState(() => _timeGranularity = v);
                        },
                      ),
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 8),
              Container(
                height: 210,
                width: double.infinity,
                decoration: BoxDecoration(
                  color: const Color(0xFF151F32),
                  borderRadius: BorderRadius.circular(14),
                  border: Border.all(color: Colors.white10),
                ),
                child: granularRows.isEmpty
                    ? const Center(
                        child: Text(
                          'No telemetry records for this selection yet.',
                          style: TextStyle(color: Colors.white54, fontSize: 12),
                        ),
                      )
                    : SingleChildScrollView(
                        scrollDirection: Axis.vertical,
                        child: DataTable(
                          columnSpacing: 16,
                          columns: const <DataColumn>[
                            DataColumn(label: Text('Time')),
                            DataColumn(label: Text('UV Data')),
                            DataColumn(label: Text('Voltage')),
                          ],
                          rows: granularRows.map((GranularTelemetryRow row) {
                            return DataRow(cells: <DataCell>[
                              DataCell(Text(row.time,
                                  style: const TextStyle(fontSize: 11))),
                              DataCell(Text(row.uvData,
                                  style: const TextStyle(
                                      fontSize: 11,
                                      fontWeight: FontWeight.bold,
                                      color: Color(0xFF00F5D4)))),
                              DataCell(Text(row.voltage,
                                  style: const TextStyle(
                                      fontSize: 11,
                                      color: Colors.amberAccent))),
                            ]);
                          }).toList(),
                        ),
                      ),
              ),
              const SizedBox(height: 14),
              Row(
                children: <Widget>[
                  Expanded(
                    child: ElevatedButton.icon(
                      style: ElevatedButton.styleFrom(
                        backgroundColor: const Color(0xFF00F5D4),
                        foregroundColor: Colors.black,
                        padding: const EdgeInsets.symmetric(vertical: 12),
                      ),
                      onPressed: _downloadGranularityCsv,
                      icon: const Icon(Icons.table_view_rounded, size: 18),
                      label: Text(
                        'Download $_timeGranularity CSV',
                        style: const TextStyle(
                            fontWeight: FontWeight.w900, fontSize: 11),
                      ),
                    ),
                  ),
                  const SizedBox(width: 8),
                  Expanded(
                    child: ElevatedButton.icon(
                      style: ElevatedButton.styleFrom(
                        backgroundColor: const Color(0xFF7B2CBF),
                        foregroundColor: Colors.white,
                        padding: const EdgeInsets.symmetric(vertical: 12),
                      ),
                      onPressed: _download3FilterComparisonCsv,
                      icon: const Icon(Icons.analytics_rounded, size: 18),
                      label: const Text(
                        '3-Filter Benchmark CSV',
                        style: TextStyle(
                            fontWeight: FontWeight.bold, fontSize: 11),
                      ),
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 24),
            ],
          ),
        ),
      ),
    );
  }

  Widget _metricCard({
    required String title,
    required String value,
    required String sub,
    required Color accent,
  }) {
    return Container(
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(
        color: const Color(0xFF151F32),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: accent.withOpacity(0.4)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          Text(title,
              style: const TextStyle(color: Colors.white60, fontSize: 10)),
          const SizedBox(height: 4),
          Text(value,
              style: TextStyle(
                  color: accent, fontSize: 15, fontWeight: FontWeight.w900)),
          const SizedBox(height: 2),
          Text(sub,
              style: const TextStyle(color: Colors.white54, fontSize: 9)),
        ],
      ),
    );
  }

  Widget _buildDropdownChip<T>({
    required String label,
    required T value,
    required List<DropdownMenuItem<T>> items,
    required ValueChanged<T?> onChanged,
  }) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 2),
      decoration: BoxDecoration(
        color: const Color(0xFF0B0F19),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: Colors.white12),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Text('$label: ',
              style: const TextStyle(color: Colors.white60, fontSize: 11)),
          DropdownButtonHideUnderline(
            child: DropdownButton<T>(
              value: value,
              dropdownColor: const Color(0xFF1E293B),
              style: const TextStyle(
                  color: Color(0xFF00F5D4),
                  fontSize: 12,
                  fontWeight: FontWeight.bold),
              items: items,
              onChanged: onChanged,
            ),
          ),
        ],
      ),
    );
  }
}
