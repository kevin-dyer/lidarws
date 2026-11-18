import { useCallback, useEffect, useRef, useState } from 'react';
import {
  ActivityIndicator,
  KeyboardAvoidingView,
  Platform,
  Pressable,
  ScrollView,
  StyleSheet,
  Text,
  TextInput,
  View,
} from 'react-native';
import { CameraView, useCameraPermissions } from 'expo-camera';
import * as FileSystem from 'expo-file-system/legacy';
import { ArrowLeft, Square } from 'lucide-react-native';
import { PALETTE } from '../theme';
import type { ScannerClient } from '../scanner/ScannerClient';
import type { Point3 } from '../scanner/pointCloud';
import { PointCloudView } from './PointCloudView';
import type { PhoneMount } from '../mapping/inertial';
import { MappingRun, type MappingSnapshot } from '../mapping/run';
import {
  DEFAULT_MAPPING_SETTINGS,
  minimumSweepPeriod,
  validateMappingSettings,
  type MappingSettings,
} from '../mapping/settings';

export interface MappingResult {
  points: Point3[];
  durationSeconds: number;
  settings: MappingSettings;
  videoUri?: string;
  trajectoryJson: string;
}

interface Props {
  scanner: ScannerClient;
  connected: boolean;
  onClose: () => void;
  onFinished: (result: MappingResult) => void;
  onError: (message: string) => void;
}

type Stage = 'form' | 'starting' | 'running' | 'stopping';

const MOUNTS: { value: PhoneMount; label: string }[] = [
  { value: 'portrait', label: 'Upright' },
  { value: 'landscape-left', label: 'Sideways, top left' },
  { value: 'landscape-right', label: 'Sideways, top right' },
];

const STATE_LABEL = {
  initializing: 'Starting map',
  tracking: 'Tracking',
  lost: 'Tracking lost',
} as const;

const formatClock = (seconds: number): string =>
  `${Math.floor(seconds / 60)}:${String(Math.floor(seconds % 60)).padStart(2, '0')}`;

export function MobileMapping({ scanner, connected, onClose, onFinished, onError }: Props) {
  const [stage, setStage] = useState<Stage>('form');
  const [form, setForm] = useState({
    period: String(DEFAULT_MAPPING_SETTINGS.sweepPeriodSeconds),
    min: String(DEFAULT_MAPPING_SETTINGS.minAngleDeg),
    max: String(DEFAULT_MAPPING_SETTINGS.maxAngleDeg),
    range: String(DEFAULT_MAPPING_SETTINGS.maxRangeM),
  });
  const [mount, setMount] = useState<PhoneMount>(DEFAULT_MAPPING_SETTINGS.phoneMount);
  const [desktopMode, setDesktopMode] = useState(false);
  const [desktopHost, setDesktopHost] = useState('');
  const [problem, setProblem] = useState<string | null>(null);
  const [snapshot, setSnapshot] = useState<MappingSnapshot | null>(null);
  const [cameraReady, setCameraReady] = useState(false);
  const [permission, requestPermission] = useCameraPermissions();
  const camera = useRef<CameraView>(null);
  const run = useRef<MappingRun | null>(null);
  const settingsRef = useRef<MappingSettings>(DEFAULT_MAPPING_SETTINGS);

  const parsed = (): MappingSettings => ({
    sweepPeriodSeconds: Number(form.period),
    minAngleDeg: Number(form.min),
    maxAngleDeg: Number(form.max),
    maxRangeM: Number(form.range),
    phoneMount: mount,
  });

  const begin = async (): Promise<void> => {
    const settings = parsed();
    const invalid = validateMappingSettings(settings);
    if (invalid) {
      setProblem(invalid);
      return;
    }
    if (desktopMode) {
      const octets = desktopHost.trim().split('.');
      if (octets.length !== 4 || octets.some((part) => !/^\d{1,3}$/.test(part) || Number(part) > 255)) {
        setProblem('Enter the desktop IPv4 address.');
        return;
      }
    }
    setProblem(null);
    const granted = permission?.granted || (await requestPermission()).granted;
    if (!granted) {
      setProblem('Camera permission is needed while mapping.');
      return;
    }
    settingsRef.current = settings;
    setSnapshot(null);
    setCameraReady(false);
    setStage('starting');
  };

  const finish = useCallback(
    async (discard = false): Promise<void> => {
      const active = run.current;
      if (!active) return;
      run.current = null;
      setStage('stopping');
      try {
        if (discard) {
          await active.abort();
          setStage('form');
          return;
        }
        const output = await active.stop();
        if (output.points.length === 0) {
          onError('No map points were captured. Check the lidar and move the scanner through the scene.');
          setStage('form');
          return;
        }
        onFinished({
          points: output.points,
          durationSeconds: output.durationSeconds,
          settings: settingsRef.current,
          videoUri: output.videoUri,
          trajectoryJson: output.trajectoryJson,
        });
      } catch (reason) {
        onError(reason instanceof Error ? reason.message : 'Could not finish mapping.');
        setStage('form');
      }
    },
    [onError, onFinished],
  );

  // The camera view mounts for the "starting" stage; once it is ready, the run can begin recording.
  useEffect(() => {
    if (stage !== 'starting' || !cameraReady) return undefined;
    let cancelled = false;
    const mapping = new MappingRun(scanner, settingsRef.current, {
      onSnapshot: setSnapshot,
      onError: (error) => {
        onError(error.message);
        void finish(true);
      },
      startVideo: async () => (await camera.current?.recordAsync())?.uri,
      stopVideo: () => camera.current?.stopRecording(),
      desktopHost: desktopMode ? desktopHost.trim() : undefined,
      captureFrame: desktopMode
        ? async () => {
            const photo = await camera.current?.takePictureAsync({ base64: true, quality: 0.3, shutterSound: false });
            if (!photo) throw new Error('Camera is not available.');
            try {
              if (!photo.base64) throw new Error('Camera returned an empty frame.');
              return photo.base64;
            } finally {
              await FileSystem.deleteAsync(photo.uri, { idempotent: true });
            }
          }
        : undefined,
    });
    run.current = mapping;
    mapping
      .start()
      .then(() => {
        if (!cancelled) setStage('running');
      })
      .catch((reason: unknown) => {
        run.current = null;
        if (cancelled) return;
        onError(reason instanceof Error ? reason.message : 'Could not start mapping.');
        setStage('form');
      });
    return () => {
      cancelled = true;
    };
    // The run is created once per start; callbacks are intentionally not dependencies.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [stage, cameraReady, scanner]);

  // Leaving the screen mid-run must stop the stage, lidar and recording.
  useEffect(
    () => () => {
      void run.current?.abort();
    },
    [],
  );

  const minPeriod = minimumSweepPeriod(Number(form.min), Number(form.max));
  const updateField = (key: keyof typeof form) => (value: string) =>
    setForm((current) => ({ ...current, [key]: value }));

  if (stage === 'form') {
    return (
      <KeyboardAvoidingView style={styles.flex} behavior={Platform.OS === 'ios' ? 'padding' : undefined}>
        <View style={styles.header}>
          <Pressable accessibilityRole="button" accessibilityLabel="Back" onPress={onClose} style={styles.back}>
            <ArrowLeft color={PALETTE.ink} size={22} />
          </Pressable>
          <View>
            <Text style={styles.eyebrow}>HAND-HELD SCANNING</Text>
            <Text style={styles.title}>Mobile mapping</Text>
          </View>
        </View>
        <ScrollView contentContainerStyle={styles.form} keyboardShouldPersistTaps="handled">
          <Text style={styles.intro}>
            Attach this phone to the front of the scanner with its rear camera facing forward. While mapping, the tilt
            stage sweeps up and down continuously and you walk the scanner through the scene. The phone builds a
            local map; desktop mode also streams sensor data and does not record local video.
          </Text>

          {Platform.OS === 'android' && (
            <>
              <Text style={styles.label}>Mapping destination</Text>
              <View style={styles.segments}>
                {[{ value: false, label: 'On this phone' }, { value: true, label: 'Desktop FAST-LIVO2 (experimental)' }].map((option) => (
                  <Pressable
                    key={option.label}
                    accessibilityRole="radio"
                    accessibilityState={{ selected: desktopMode === option.value }}
                    onPress={() => setDesktopMode(option.value)}
                    style={[styles.segment, desktopMode === option.value && styles.segmentOn]}
                  >
                    <Text style={[styles.segmentText, desktopMode === option.value && styles.segmentTextOn]}>{option.label}</Text>
                  </Pressable>
                ))}
              </View>
              {desktopMode && (
                <>
                  <Text style={styles.label}>Desktop IPv4 address</Text>
                  <TextInput
                    accessibilityLabel="Desktop IPv4 address"
                    value={desktopHost}
                    onChangeText={setDesktopHost}
                    keyboardType="decimal-pad"
                    autoCapitalize="none"
                    placeholder="192.168.1.50"
                    style={styles.input}
                  />
                  <Text style={styles.note}>
                    Streams camera frames, IMU and timed LiDAR to UDP port 5005. Local video is not recorded.
                    Calibrate the rig before relying on desktop maps.
                  </Text>
                </>
              )}
            </>
          )}

          <Text style={styles.label}>Phone orientation on the scanner</Text>
          <View style={styles.segments}>
            {MOUNTS.map((option) => (
              <Pressable
                key={option.value}
                accessibilityRole="radio"
                accessibilityState={{ selected: mount === option.value }}
                onPress={() => setMount(option.value)}
                style={[styles.segment, mount === option.value && styles.segmentOn]}
              >
                <Text style={[styles.segmentText, mount === option.value && styles.segmentTextOn]}>{option.label}</Text>
              </Pressable>
            ))}
          </View>

          <Field
            label="Seconds per up-and-down sweep"
            value={form.period}
            onChange={updateField('period')}
            hint={`Default 5 s. At least ${Number.isFinite(minPeriod) ? minPeriod.toFixed(1) : '–'} s for these angles.`}
          />
          <View style={styles.row}>
            <View style={styles.half}>
              <Field label="Lowest tilt (°)" value={form.min} onChange={updateField('min')} signed />
            </View>
            <View style={styles.half}>
              <Field label="Highest tilt (°)" value={form.max} onChange={updateField('max')} signed />
            </View>
          </View>
          <Field
            label="Maximum range (m)"
            value={form.range}
            onChange={updateField('range')}
            hint="Points farther than this are ignored."
          />

          {problem ? <Text style={styles.problem}>{problem}</Text> : null}
          <Text style={styles.note}>
            Hold the scanner still for the first sweep so the map can be seeded, then move slowly and steadily.
          </Text>

          <Pressable
            accessibilityRole="button"
            disabled={!connected}
            onPress={() => void begin()}
            style={[styles.primary, !connected && styles.disabled]}
          >
            <Text style={styles.primaryText}>{connected ? 'Start mapping' : 'Connect the scanner first'}</Text>
          </Pressable>
        </ScrollView>
      </KeyboardAvoidingView>
    );
  }

  const running = stage === 'running';
  return (
    <View style={styles.flex}>
      <View style={styles.liveHeader}>
        <View style={styles.cameraBox}>
          <CameraView
            ref={camera}
            style={StyleSheet.absoluteFill}
            facing="back"
            mode={desktopMode ? 'picture' : 'video'}
            pictureSize={desktopMode ? '640x480' : undefined}
            animateShutter={!desktopMode}
            mute
            videoStabilizationMode="off"
            onCameraReady={() => setCameraReady(true)}
            onMountError={(error) => {
              onError(`Camera could not start: ${error.message}`);
              if (run.current) void finish(true);
              else setStage('form');
            }}
          />
          {running && <View style={styles.recDot} />}
        </View>
        <View style={styles.liveStats}>
          <Text style={styles.statLine}>{snapshot ? STATE_LABEL[snapshot.stats.state] : 'Starting scanner…'}</Text>
          <Text style={styles.statSub}>
            {snapshot ? `${formatClock(snapshot.elapsedSeconds)} · ${snapshot.stats.mapPoints.toLocaleString()} points` : ' '}
          </Text>
          <Text style={styles.statSub}>{snapshot ? `${snapshot.stats.distanceM.toFixed(1)} m travelled` : ' '}</Text>
        </View>
      </View>

      <View style={styles.cloud}>
        <PointCloudView points={snapshot?.points ?? []} fill zoom={1.4} fullscreenButton={false} />
        {snapshot && snapshot.holdStillSeconds > 0 && (
          <View style={styles.banner}>
            <Text style={styles.bannerText}>Hold still · {snapshot.holdStillSeconds}s</Text>
          </View>
        )}
        {snapshot?.stats.state === 'lost' && (
          <View style={[styles.banner, styles.bannerWarn]}>
            <Text style={styles.bannerText}>Lost tracking – hold still while retrying</Text>
            <Text style={styles.bannerText}>
              Matches: {snapshot.stats.lastMatches} · RMSE:{' '}
              {Number.isFinite(snapshot.stats.lastRmse) ? `${snapshot.stats.lastRmse.toFixed(3)} m` : 'n/a'}
            </Text>
          </View>
        )}
      </View>

      <View style={styles.stopBar}>
        <Pressable
          accessibilityRole="button"
          accessibilityLabel="Stop mapping"
          disabled={!running}
          onPress={() => void finish()}
          style={[styles.stop, !running && styles.disabled]}
        >
          {stage === 'stopping' || stage === 'starting' ? (
            <ActivityIndicator color={PALETTE.white} />
          ) : (
            <Square color={PALETTE.white} fill={PALETTE.white} size={18} />
          )}
          <Text style={styles.stopText}>
            {stage === 'starting' ? 'Homing and starting…' : stage === 'stopping' ? 'Finishing…' : 'Stop mapping'}
          </Text>
        </Pressable>
      </View>
    </View>
  );
}

function Field({
  label,
  value,
  onChange,
  hint,
  signed = false,
}: {
  label: string;
  value: string;
  onChange: (value: string) => void;
  hint?: string;
  signed?: boolean;
}) {
  return (
    <View style={styles.field}>
      <Text style={styles.label}>{label}</Text>
      <TextInput
        accessibilityLabel={label}
        value={value}
        onChangeText={onChange}
        keyboardType={signed && Platform.OS === 'ios' ? 'numbers-and-punctuation' : 'decimal-pad'}
        style={styles.input}
      />
      {hint ? <Text style={styles.hint}>{hint}</Text> : null}
    </View>
  );
}

const styles = StyleSheet.create({
  flex: { flex: 1, backgroundColor: PALETTE.paper },
  header: { flexDirection: 'row', alignItems: 'center', gap: 12, paddingHorizontal: 20, paddingVertical: 14 },
  back: { width: 42, height: 42, borderRadius: 21, backgroundColor: PALETTE.white, alignItems: 'center', justifyContent: 'center' },
  eyebrow: { color: PALETTE.moss, fontSize: 11, fontWeight: '700', letterSpacing: 1.2 },
  title: { color: PALETTE.ink, fontSize: 24, fontWeight: '700' },
  form: { padding: 20, gap: 14, paddingBottom: 40 },
  intro: { color: PALETTE.muted, fontSize: 14, lineHeight: 20 },
  label: { color: PALETTE.ink, fontSize: 14, fontWeight: '600' },
  segments: { gap: 8 },
  segment: { borderWidth: 1, borderColor: PALETTE.line, borderRadius: 12, padding: 12, backgroundColor: PALETTE.white },
  segmentOn: { backgroundColor: PALETTE.forest, borderColor: PALETTE.forest },
  segmentText: { color: PALETTE.ink, fontSize: 14 },
  segmentTextOn: { color: PALETTE.white, fontWeight: '600' },
  field: { gap: 6 },
  row: { flexDirection: 'row', gap: 12 },
  half: { flex: 1 },
  input: { borderWidth: 1, borderColor: PALETTE.line, borderRadius: 12, backgroundColor: PALETTE.white, paddingHorizontal: 14, paddingVertical: 11, fontSize: 16, color: PALETTE.ink },
  hint: { color: PALETTE.muted, fontSize: 12 },
  problem: { color: PALETTE.danger, fontSize: 14 },
  note: { color: PALETTE.muted, fontSize: 13, lineHeight: 18 },
  primary: { backgroundColor: PALETTE.forest, borderRadius: 16, paddingVertical: 16, alignItems: 'center', marginTop: 6 },
  primaryText: { color: PALETTE.lime, fontSize: 16, fontWeight: '700' },
  disabled: { opacity: 0.5 },
  liveHeader: { flexDirection: 'row', alignItems: 'center', gap: 14, padding: 16 },
  cameraBox: { width: 96, height: 128, borderRadius: 14, overflow: 'hidden', backgroundColor: '#000' },
  recDot: { position: 'absolute', top: 8, right: 8, width: 10, height: 10, borderRadius: 5, backgroundColor: '#E5484D' },
  liveStats: { flex: 1, gap: 4 },
  statLine: { color: PALETTE.ink, fontSize: 18, fontWeight: '700' },
  statSub: { color: PALETTE.muted, fontSize: 14 },
  cloud: { flex: 1, marginHorizontal: 12, borderRadius: 18, overflow: 'hidden', backgroundColor: PALETTE.white },
  banner: { position: 'absolute', top: 10, alignSelf: 'center', backgroundColor: PALETTE.forest, borderRadius: 14, paddingHorizontal: 14, paddingVertical: 7 },
  bannerWarn: { backgroundColor: PALETTE.danger },
  bannerText: { color: PALETTE.white, fontSize: 13, fontWeight: '600' },
  stopBar: { padding: 16 },
  stop: { backgroundColor: PALETTE.danger, borderRadius: 18, paddingVertical: 18, flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 10 },
  stopText: { color: PALETTE.white, fontSize: 17, fontWeight: '700' },
});
