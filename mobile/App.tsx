import { useEffect, useMemo, useState } from 'react';
import type { ReactNode } from 'react';
import {
  ActivityIndicator,
  Alert,
  FlatList,
  Image,
  KeyboardAvoidingView,
  Platform,
  Pressable,
  ScrollView,
  StatusBar,
  StyleSheet,
  Text,
  View,
  useWindowDimensions,
} from 'react-native';
import { SafeAreaProvider, SafeAreaView } from 'react-native-safe-area-context';
import Slider from '@react-native-community/slider';
import * as ImagePicker from 'expo-image-picker';
import * as Sharing from 'expo-sharing';
import Svg, { Circle } from 'react-native-svg';
import {
  ArrowLeft,
  Camera,
  Check,
  ChevronRight,
  Clock3,
  Compass,
  FolderOpen,
  Menu,
  Radio,
  ScanLine,
  Share2,
  Trash2,
  Wifi,
} from 'lucide-react-native';
import { SetupWizard } from './src/components/SetupWizard';
import { requestLocalNetworkAccess } from './src/scanner/permissions';
import { getLastScannerIp } from './src/storage/scannerAddress';
import { PALETTE } from './src/theme';
import { PointCloudView } from './src/components/PointCloudView';
import { AngleSelector } from './src/components/AngleSelector';
import { ScanCancelledError, ScannerClient, type ScanProgress, type ScanSettings } from './src/scanner/ScannerClient';
import type { Point3 } from './src/scanner/pointCloud';
import { ScanCarousel } from './src/components/ScanCarousel';
import { deleteScan, listScans, loadScanPoints, saveScan, type SavedScan } from './src/storage/scans';

type Screen = 'home' | 'setup' | 'scan' | 'scanning' | 'finished' | 'gallery' | 'detail';


const INITIAL_SCAN: ScanSettings = {
  startAngle: -45,
  stopAngle: 45,
  durationSeconds: 30,
};

function AppButton({
  title,
  onPress,
  kind = 'dark',
  icon,
  disabled = false,
}: {
  title: string;
  onPress: () => void;
  kind?: 'dark' | 'light' | 'outline' | 'danger';
  icon?: ReactNode;
  disabled?: boolean;
}) {
  return (
    <Pressable
      accessibilityRole="button"
      disabled={disabled}
      onPress={onPress}
      style={({ pressed }) => [
        styles.button,
        styles[`button_${kind}`],
        disabled && styles.buttonDisabled,
        pressed && !disabled && styles.buttonPressed,
      ]}
    >
      {icon}
      <Text style={[styles.buttonText, kind === 'light' && styles.buttonTextDark]}>{title}</Text>
      {kind === 'light' && <ChevronRight color={PALETTE.forest} size={19} />}
    </Pressable>
  );
}

const ZOOM = 1.6;

function PageHeader({
  title,
  eyebrow,
  onBack,
  action,
}: {
  title: string;
  eyebrow?: string;
  onBack: () => void;
  action?: ReactNode;
}) {
  return (
    <View style={styles.pageHeader}>
      <Pressable accessibilityRole="button" accessibilityLabel="Go back" onPress={onBack} style={styles.backButton}>
        <ArrowLeft size={21} color={PALETTE.ink} />
      </Pressable>
      <View style={styles.pageHeading}>
        {eyebrow ? <Text style={styles.eyebrow}>{eyebrow}</Text> : null}
        <Text style={styles.pageTitle}>{title}</Text>
      </View>
      {action ?? <View style={styles.headerActionPlaceholder} />}
    </View>
  );
}

function formatDate(value: string): string {
  return new Date(value).toLocaleDateString(undefined, {
    month: 'short',
    day: 'numeric',
    year: 'numeric',
  });
}

const knownAddresses = async (): Promise<string[]> => {
  const ip = await getLastScannerIp().catch(() => null);
  return ip ? [ip] : [];
};

export default function App() {
  const [screen, setScreen] = useState<Screen>('home');
  const [deviceAddress, setDeviceAddress] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [scanSettings, setScanSettings] = useState<ScanSettings>(INITIAL_SCAN);
  const [scanProgress, setScanProgress] = useState<ScanProgress>({
    phase: 'preparing',
    percent: 0,
    secondsRemaining: 0,
    status: 'Preparing scanner…',
  });
  const [scanPoints, setScanPoints] = useState<Point3[]>([]);
  const [livePoints, setLivePoints] = useState<Point3[]>([]);
  const [scanPhoto, setScanPhoto] = useState<string | undefined>();
  const [scans, setScans] = useState<SavedScan[]>([]);
  const [loadingScans, setLoadingScans] = useState(false);
  const [saving, setSaving] = useState(false);
  const [busy, setBusy] = useState(false);
  const [selectedScan, setSelectedScan] = useState<SavedScan | null>(null);
  const [selectedPoints, setSelectedPoints] = useState<Point3[]>([]);
  const [selecting, setSelecting] = useState(false);
  const [selectedIds, setSelectedIds] = useState<string[]>([]);
  const { width: screenWidth } = useWindowDimensions();
  const [draggingAngle, setDraggingAngle] = useState(false);
  const [lidarOk, setLidarOk] = useState(true);
  const [photoLoading, setPhotoLoading] = useState(false);

  const scanner = useMemo(
    () =>
      new ScannerClient(
        (address) => setDeviceAddress(address),
        (message) => setError(message),
      ),
    [],
  );

  useEffect(() => {
    void requestLocalNetworkAccess()
      .then(() => scanner.open())
      .then(async () => scanner.discover(4500, await knownAddresses()))
      .catch(() => {});
    return () => scanner.close();
  }, [scanner]);

  useEffect(() => {
    if (screen !== 'scanning') return undefined;
    setLivePoints([]);
    let stillOnScreen = true;
    void scanner
      .startScan(scanSettings, (progress) => {
        if (stillOnScreen) setScanProgress(progress);
      }, (livePoints) => {
        if (stillOnScreen) setLivePoints(livePoints);
      })
      .then((result) => {
        if (!stillOnScreen) return;
        setScanPoints(result.points);
        setScanSettings((current) => ({ ...current, durationSeconds: result.durationSeconds }));
        setScreen('finished');
      })
      .catch((reason: unknown) => {
        if (!stillOnScreen || reason instanceof ScanCancelledError) return;
        setError(reason instanceof Error ? reason.message : 'The scan could not be completed.');
        setScreen('scan');
      });
    return () => {
      stillOnScreen = false;
    };
  }, [screen, scanner, scanSettings]);

  useEffect(() => {
    if (!deviceAddress || screen === 'scanning' || screen === 'setup') return undefined;
    let misses = 0;
    let cancelled = false;
    const timer = setInterval(() => {
      void scanner.checkConnection().then((state) => {
        if (cancelled) return;
        if (state === 'offline') {
          misses += 1;
          if (misses >= 2) setDeviceAddress(null);
          return;
        }
        misses = 0;
        setLidarOk(state === 'ok');
      }).catch(() => {});
    }, 4000);
    return () => {
      cancelled = true;
      clearInterval(timer);
    };
  }, [scanner, deviceAddress, screen]);

  useEffect(() => {
    if (screen !== 'gallery' && screen !== 'home') return;
    if (screen === 'gallery') {
      setLoadingScans(true);
      setSelecting(false);
      setSelectedIds([]);
    }
    void listScans()
      .then(setScans)
      .catch((reason: unknown) => setError(reason instanceof Error ? reason.message : 'Could not load saved scans.'))
      .finally(() => setLoadingScans(false));
  }, [screen]);

  const openScan = (scan: SavedScan): void => {
    setSelectedScan(scan);
    setSelectedPoints([]);
    setScreen('detail');
    void loadScanPoints(scan, 30000)
      .then(setSelectedPoints)
      .catch(() => setError('Could not load this scan.'));
  };

  const toggleSelected = (id: string): void => {
    setSelectedIds((current) => (current.includes(id) ? current.filter((item) => item !== id) : [...current, id]));
  };

  const shareSelected = async (): Promise<void> => {
    for (const scan of scans.filter((item) => selectedIds.includes(item.id))) {
      await shareSavedScan(scan);
    }
  };

  const deleteSelected = (): void => {
    const targets = scans.filter((item) => selectedIds.includes(item.id));
    Alert.alert(`Delete ${targets.length} scan${targets.length === 1 ? '' : 's'}?`, 'They will be removed from this device.', [
      { text: 'Cancel', style: 'cancel' },
      {
        text: 'Delete',
        style: 'destructive',
        onPress: () => {
          void Promise.all(targets.map((scan) => deleteScan(scan)))
            .then(() => {
              setScans((current) => current.filter((scan) => !selectedIds.includes(scan.id)));
              setSelectedIds([]);
              setSelecting(false);
            })
            .catch((reason: unknown) =>
              setError(reason instanceof Error ? reason.message : 'Could not delete the selected scans.'),
            );
        },
      },
    ]);
  };

  const refreshDevice = async (): Promise<void> => {
    setBusy(true);
    setError(null);
    try {
      await requestLocalNetworkAccess();
      await scanner.discover(4500, await knownAddresses());
    } catch (reason) {
      setDeviceAddress(null);
      setError(reason instanceof Error ? reason.message : 'Scanner discovery failed.');
    } finally {
      setBusy(false);
    }
  };

  const takeScanPhoto = async (): Promise<void> => {
    try {
      const permission = await ImagePicker.requestCameraPermissionsAsync();
      if (!permission.granted) {
        throw new Error('Camera permission is required to add a scan photo.');
      }
      setPhotoLoading(true);
      const result = await ImagePicker.launchCameraAsync({ quality: 0.82 });
      if (result.canceled) setPhotoLoading(false);
      else setScanPhoto(result.assets[0]?.uri);
    } catch (reason) {
      setPhotoLoading(false);
      setError(reason instanceof Error ? reason.message : 'Could not open the camera.');
    }
  };

  const finishSave = async (): Promise<void> => {
    setSaving(true);
    setError(null);
    try {
      await saveScan(scanPoints, scanSettings, scanPhoto);
      setScanPoints([]);
      setScanPhoto(undefined);
      setScreen('home');
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : 'Could not save the point cloud.');
    } finally {
      setSaving(false);
    }
  };

  const discardScan = (): void => {
    Alert.alert('Delete this scan?', 'The captured point cloud and reference photo will be discarded.', [
      { text: 'Keep scan', style: 'cancel' },
      {
        text: 'Delete',
        style: 'destructive',
        onPress: () => {
          setScanPoints([]);
          setScanPhoto(undefined);
          setScreen('home');
        },
      },
    ]);
  };

  const shareSavedScan = async (scan: SavedScan): Promise<void> => {
    try {
      if (!(await Sharing.isAvailableAsync())) {
        throw new Error('The system share sheet is not available on this device.');
      }
      await Sharing.shareAsync(scan.fileUri, {
        dialogTitle: 'Share point cloud',
        mimeType: 'application/octet-stream',
        UTI: 'public.data',
      });
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : 'Could not share this scan.');
    }
  };

  const confirmDeleteSavedScan = (scan: SavedScan): void => {
    Alert.alert('Delete saved scan?', `${scan.name} will be removed from this device.`, [
      { text: 'Cancel', style: 'cancel' },
      {
        text: 'Delete',
        style: 'destructive',
        onPress: () => {
          void deleteScan(scan)
            .then(() => setScans((current) => current.filter((saved) => saved.id !== scan.id)))
            .catch((reason: unknown) =>
              setError(reason instanceof Error ? reason.message : 'Could not delete this scan.'),
            );
        },
      },
    ]);
  };

  const selectAngle = (startAngle: number, stopAngle: number): void => {
    setScanSettings((current) => ({ ...current, startAngle, stopAngle }));
  };

  const durationMinimum = Math.ceil(
    (scanSettings.stopAngle - scanSettings.startAngle) / 10,
  );
  const effectiveDuration = Math.max(scanSettings.durationSeconds, durationMinimum);
  const tiltSpeed = (scanSettings.stopAngle - scanSettings.startAngle) / effectiveDuration;

  return (
    <SafeAreaProvider>
      <SafeAreaView style={styles.safeArea}>
        <StatusBar barStyle="dark-content" backgroundColor={PALETTE.paper} />
        <KeyboardAvoidingView
          style={styles.flex}
          behavior={Platform.OS === 'ios' ? 'padding' : undefined}
        >
        {error ? (
          <Pressable accessibilityRole="button" onPress={() => setError(null)} style={styles.errorBanner}>
            <Text style={styles.errorText}>{error}</Text>
            <Text style={styles.dismissError}>×</Text>
          </Pressable>
        ) : null}

        {screen === 'home' && (
          <ScrollView contentContainerStyle={styles.homePage} showsVerticalScrollIndicator={false}>
            <View style={styles.homeTopRow}>
              <View style={styles.brand}>
                <View style={styles.brandMark}><ScanLine color={PALETTE.lime} size={19} /></View>
                <View>
                  <Text style={styles.brandName}>3D LIDAR SCANNER</Text>
                  <Text style={styles.brandCaption}>FIELD SCANNER</Text>
                </View>
              </View>
              <Pressable
                accessibilityRole="button"
                accessibilityLabel="Scanner settings"
                onPress={() => setScreen('setup')}
                style={styles.iconButton}
              >
                <Menu color={PALETTE.ink} size={22} />
              </Pressable>
            </View>

            {scans.length > 0 ? (
              <ScanCarousel scans={scans} onOpen={openScan} />
            ) : (
              <View style={styles.heroCopy}>
                <Text style={styles.heroKicker}>YOUR WORLD, IN 3D</Text>
                <Text style={styles.heroTitle}>Capture the{'\n'}shape of a place.</Text>
                <Text style={styles.heroDescription}>Set up your scanner and turn a moment into a point cloud.</Text>
              </View>
            )}

            <View style={styles.deviceCard}>
              <View style={styles.deviceIcon}><Radio size={21} color={PALETTE.forest} /></View>
              <View style={styles.deviceInfo}>
                <Text style={styles.cardOverline}>SCANNER</Text>
                <Text style={styles.deviceTitle}>{!deviceAddress ? 'Scanner not connected' : lidarOk ? 'Lidar connected' : 'Lidar not detected'}</Text>
                <Text style={styles.deviceSubline}>
                  {deviceAddress ? (lidarOk ? `Local device · ${deviceAddress}` : 'Check the lidar cable and power') : 'Connect on home Wi-Fi or use its access point'}
                </Text>
              </View>
              <Pressable
                accessibilityRole="button"
                accessibilityLabel={deviceAddress ? 'Refresh connection' : 'Find scanner'}
                onPress={() => void refreshDevice()}
                style={styles.refreshButton}
              >
                {busy ? <ActivityIndicator size="small" color={PALETTE.forest} /> : <Wifi size={18} color={PALETTE.forest} />}
              </Pressable>
            </View>

            <View style={styles.homeActions}>
              <Text style={styles.sectionTitle}>Ready when you are</Text>
              <AppButton
                title="Start a scan"
                kind="light"
                icon={<Compass color={PALETTE.forest} size={19} />}
                onPress={() => {
                  setError(null);
                  setScanSettings(INITIAL_SCAN);
                  setScreen('scan');
                }}
                disabled={!deviceAddress}
              />
              {!deviceAddress && (
                <Pressable onPress={() => setScreen('setup')} style={styles.connectHint}>
                  <Text style={styles.connectHintText}>Set up scanner connection</Text>
                  <ChevronRight color={PALETTE.moss} size={16} />
                </Pressable>
              )}
              <Pressable
                accessibilityRole="button"
                onPress={() => setScreen('gallery')}
                style={styles.galleryAction}
              >
                <View style={styles.galleryIcon}><FolderOpen color={PALETTE.forest} size={20} /></View>
                <View style={styles.galleryActionText}>
                  <Text style={styles.galleryTitle}>Your Scans</Text>
                  <Text style={styles.gallerySubline}>Your saved point clouds, all in one place</Text>
                </View>
                <ChevronRight color={PALETTE.muted} size={20} />
              </Pressable>
            </View>

            <View style={styles.homeFooter}>
              <View style={styles.footerDot} />
              <Text style={styles.footerText}>Scans are saved privately on this device</Text>
            </View>
          </ScrollView>
        )}

        {screen === 'setup' && (
          <SetupWizard
            scanner={scanner}
            onClose={() => setScreen('home')}
            onConnected={(address) => setDeviceAddress(address)}
          />
        )}

        {screen === 'scan' && (
          <View style={styles.flex}>
            <PageHeader title="New 3D scan" eyebrow="CAPTURE" onBack={() => setScreen('home')} />
            <ScrollView
              contentContainerStyle={styles.scanPageContent}
              showsVerticalScrollIndicator={false}
              scrollEnabled={!draggingAngle}
            >
              <Text style={styles.introText}>Choose the tilt range and capture time for this scene.</Text>
              <View style={styles.scanCard}>
                <View style={styles.cardTitleRow}>
                  <Text style={styles.sectionTitle}>Tilt range</Text>
                  <Text style={styles.dragLabel}>DRAG TO ADJUST</Text>
                </View>
                <AngleSelector
                  startAngle={scanSettings.startAngle}
                  stopAngle={scanSettings.stopAngle}
                  onChange={selectAngle}
                  onDragChange={setDraggingAngle}
                />
                <View style={styles.angleReadouts}>
                  <View style={styles.angleReadout}>
                    <Text style={styles.readoutLabel}>START</Text>
                    <Text style={styles.readoutValue}>{scanSettings.startAngle}°</Text>
                  </View>
                  <View style={styles.angleRangeDivider} />
                  <View style={styles.angleReadout}>
                    <Text style={styles.readoutLabel}>STOP</Text>
                    <Text style={styles.readoutValue}>{scanSettings.stopAngle}°</Text>
                  </View>
                </View>
              </View>

              <View style={styles.scanCard}>
                <View style={styles.durationHeading}>
                  <View>
                    <Text style={styles.sectionTitle}>Scan duration</Text>
                    <Text style={styles.deviceSubline}>Motor speed adjusts to your chosen time</Text>
                  </View>
                  <View style={styles.durationBadge}>
                    <Text style={styles.durationValue}>{effectiveDuration}</Text>
                    <Text style={styles.durationUnit}>SEC</Text>
                  </View>
                </View>
                <Slider
                  accessibilityLabel="Scan duration"
                  minimumValue={5}
                  maximumValue={120}
                  step={1}
                  value={scanSettings.durationSeconds}
                  minimumTrackTintColor={PALETTE.forest}
                  maximumTrackTintColor="#DFE7E2"
                  thumbTintColor={PALETTE.forest}
                  onValueChange={(durationSeconds) =>
                    setScanSettings((current) => ({ ...current, durationSeconds }))
                  }
                  style={styles.slider}
                />
                <View style={styles.sliderLabels}>
                  <Text style={styles.sliderLabel}>5 SEC</Text>
                  <Text style={styles.sliderLabel}>120 SEC</Text>
                </View>
                <View style={styles.speedNote}>
                  <Clock3 size={15} color={PALETTE.moss} />
                  <Text style={styles.speedNoteText}>
                    {durationMinimum > 5 && scanSettings.durationSeconds < durationMinimum
                      ? `Tilt safety limit: minimum ${durationMinimum}s at these angles`
                      : `Tilt speed ${tiltSpeed.toFixed(1)}°/s · capped at 10°/s`}
                  </Text>
                </View>
              </View>

              <Pressable
                accessibilityRole="checkbox"
                accessibilityState={{ checked: scanSettings.homeBeforeScan !== false }}
                onPress={() =>
                  setScanSettings((current) => ({ ...current, homeBeforeScan: current.homeBeforeScan === false }))
                }
                style={styles.checkRow}
              >
                <View style={[styles.checkBox, scanSettings.homeBeforeScan !== false && styles.checkBoxOn]}>
                  {scanSettings.homeBeforeScan !== false && <Check size={15} color={PALETTE.white} />}
                </View>
                <View style={styles.flex}>
                  <Text style={styles.sectionTitle}>Home before scan</Text>
                  <Text style={styles.deviceSubline}>Re-find the tilt stage zero position first</Text>
                </View>
              </Pressable>
            </ScrollView>
            <View style={styles.bottomBar}>
              <AppButton
                title="Scan"
                icon={<ScanLine color={PALETTE.lime} size={19} />}
                onPress={() => {
                  setError(null);
                  setScanPhoto(undefined);
                  setScanProgress({
                    phase: 'preparing',
                    percent: 0,
                    secondsRemaining: 0,
                    status: scanSettings.homeBeforeScan === false ? 'Moving to the start angle…' : 'Homing tilt stage…',
                  });
                  setScreen('scanning');
                }}
                disabled={!deviceAddress}
              />
              {!deviceAddress && <Text style={styles.bottomHint}>Connect to your scanner to begin.</Text>}
            </View>
          </View>
        )}

        {screen === 'scanning' && (
          <View style={styles.scanningPage}>
            {scanProgress.phase === 'preparing' && (
              <View style={styles.scanningTop}>
                <View style={styles.liveBadge}><View style={styles.liveDot} /><Text style={styles.liveText}>LIVE CAPTURE</Text></View>
                <Text style={styles.scanningTitle}>Scanning</Text>
                <Text style={styles.scanningSubtitle}>Keep the scanner steady while it captures the scene.</Text>
              </View>
            )}
            {scanProgress.phase === 'preparing' ? (
              <View style={styles.progressWrap}>
                <ActivityIndicator size="large" color={PALETTE.forest} />
                <Text style={styles.homingTitle}>{scanProgress.status}</Text>
                <Text style={styles.progressStatus}>The scan starts once the tilt stage is in position.</Text>
              </View>
            ) : (
              <View style={styles.cloudWrap}>
                <PointCloudView points={livePoints} fill zoom={ZOOM} />
                <View style={styles.progressOverlay}>
                  <View style={styles.progressRing}>
                    <SvgRing progress={scanProgress.percent} />
                    <View style={styles.progressCenter}>
                      <Text style={styles.secondsValue}>{scanProgress.secondsRemaining}</Text>
                      <Text style={styles.secondsLabel}>SEC</Text>
                    </View>
                  </View>
                </View>
                <Text style={styles.progressPercent}>{scanProgress.percent}% COMPLETE</Text>
              </View>
            )}
            <View style={styles.bottomBar}>
              <AppButton
                title="Cancel scan"
                kind="outline"
                onPress={() => {
                  scanner.cancelScan();
                  setScreen('home');
                }}
              />
              <Text style={styles.bottomHint}>The tilt stage returns to home when stopped.</Text>
            </View>
          </View>
        )}

        {screen === 'finished' && (
          <View style={styles.flex}>
            <PageHeader
              title="Scan complete"
              eyebrow="CAPTURE FINISHED"
              onBack={() => setScreen('home')}
              action={<View style={styles.completeIcon}><Check size={27} color={PALETTE.forest} /></View>}
            />
            <ScrollView contentContainerStyle={styles.finishedContent}>
              <View style={[styles.finishedCloud, { width: screenWidth, height: screenWidth * 1.05 }]}>
                <PointCloudView points={scanPoints} fill zoom={ZOOM} height={screenWidth * 1.05} />
              </View>
              <Text style={styles.cloudCaption}>
                {scanPoints.length.toLocaleString()} points captured · {effectiveDuration} second sweep
              </Text>
              <Pressable accessibilityRole="button" onPress={() => void takeScanPhoto()} style={styles.photoCard}>
                {scanPhoto ? (
                  <Image
                    source={{ uri: scanPhoto }}
                    style={styles.photoPreview}
                    onLoadEnd={() => setPhotoLoading(false)}
                  />
                ) : (
                  <View style={styles.photoPlaceholder}>
                    <View style={styles.photoIcon}><Camera color={PALETTE.forest} size={21} /></View>
                    <Text style={styles.photoTitle}>Add a reference photo</Text>
                    <Text style={styles.photoSubline}>Optional · taken with your camera</Text>
                  </View>
                )}
                {scanPhoto && <Text style={styles.replacePhoto}>TAP TO RETAKE</Text>}
                {photoLoading && (
                  <View style={styles.photoLoading}>
                    <ActivityIndicator size="large" color={PALETTE.forest} />
                    <Text style={styles.photoSubline}>Processing photo…</Text>
                  </View>
                )}
              </Pressable>
              <View style={styles.finishedActions}>
                <AppButton
                  title={saving ? 'Saving point cloud…' : 'Save point cloud'}
                  icon={saving ? <ActivityIndicator color={PALETTE.lime} /> : <FolderOpen color={PALETTE.lime} size={19} />}
                  onPress={() => void finishSave()}
                  disabled={saving}
                />
                <AppButton title="Delete scan" kind="outline" icon={<Trash2 color={PALETTE.white} size={18} />} onPress={discardScan} />
                <Text style={styles.formNote}>Saved as a standard .ply point cloud for post-processing and sharing.</Text>
              </View>
            </ScrollView>
          </View>
        )}

        {screen === 'gallery' && (
          <View style={styles.flex}>
            <PageHeader
              title="Your scans"
              eyebrow="LIBRARY"
              onBack={() => setScreen('home')}
              action={
                scans.length > 0 ? (
                  <Pressable
                    accessibilityRole="button"
                    onPress={() => {
                      setSelecting((current) => !current);
                      setSelectedIds([]);
                    }}
                  >
                    <Text style={styles.headerLink}>{selecting ? 'Done' : 'Select'}</Text>
                  </Pressable>
                ) : (
                  <View style={styles.scanCountBadge}><Text style={styles.scanCount}>{scans.length}</Text></View>
                )
              }
            />
            {selecting && (
              <View style={styles.selectBar}>
                <View style={styles.selectBarButton}>
                  <AppButton
                    title={`Share${selectedIds.length ? ` (${selectedIds.length})` : ''}`}
                    icon={<Share2 color={PALETTE.lime} size={18} />}
                    onPress={() => void shareSelected()}
                    disabled={selectedIds.length === 0}
                  />
                </View>
                <View style={styles.selectBarButton}>
                  <AppButton
                    title={`Delete${selectedIds.length ? ` (${selectedIds.length})` : ''}`}
                    kind="outline"
                    icon={<Trash2 color={PALETTE.white} size={18} />}
                    onPress={deleteSelected}
                    disabled={selectedIds.length === 0}
                  />
                </View>
              </View>
            )}
            {loadingScans ? (
              <View style={styles.emptyState}><ActivityIndicator color={PALETTE.forest} /></View>
            ) : scans.length === 0 ? (
              <View style={styles.emptyState}>
                <View style={styles.emptyIcon}><FolderOpen size={25} color={PALETTE.forest} /></View>
                <Text style={styles.emptyTitle}>No scans yet</Text>
                <Text style={styles.emptyText}>Saved point clouds will appear here, ready to share.</Text>
              </View>
            ) : (
              <FlatList
                contentContainerStyle={styles.galleryList}
                data={scans}
                keyExtractor={(item) => item.id}
                extraData={[selecting, selectedIds]}
                renderItem={({ item }) => (
                  <Pressable
                    accessibilityRole="button"
                    onPress={() => (selecting ? toggleSelected(item.id) : openScan(item))}
                    onLongPress={() => {
                      setSelecting(true);
                      toggleSelected(item.id);
                    }}
                    style={styles.savedScanCard}
                  >
                    {selecting && (
                      <View style={[styles.selectBox, selectedIds.includes(item.id) && styles.selectBoxOn]}>
                        {selectedIds.includes(item.id) && <Check size={14} color={PALETTE.white} />}
                      </View>
                    )}
                    <View style={styles.thumb}>
                      {item.photoUri ? (
                        <Image source={{ uri: item.photoUri }} style={styles.thumbImage} />
                      ) : (
                        <ScanLine color={PALETTE.forest} size={20} />
                      )}
                    </View>
                    <View style={styles.savedScanInfo}>
                      <Text style={styles.savedScanName} numberOfLines={1}>{item.name}</Text>
                      <Text style={styles.savedScanDetails}>
                        {item.pointCount.toLocaleString()} points · {formatDate(item.createdAt)}
                      </Text>
                      <Text style={styles.savedScanFile} numberOfLines={1}>{item.id}_raw_cloud.ply</Text>
                    </View>
                    {!selecting && (
                      <View style={styles.savedScanActions}>
                        <Pressable
                          accessibilityRole="button"
                          accessibilityLabel="Share point cloud"
                          onPress={() => void shareSavedScan(item)}
                          style={styles.smallAction}
                        >
                          <Share2 color={PALETTE.forest} size={18} />
                        </Pressable>
                        <Pressable
                          accessibilityRole="button"
                          accessibilityLabel="Delete point cloud"
                          onPress={() => confirmDeleteSavedScan(item)}
                          style={styles.smallAction}
                        >
                          <Trash2 color={PALETTE.danger} size={18} />
                        </Pressable>
                      </View>
                    )}
                  </Pressable>
                )}
              />
            )}
            <View style={styles.galleryFootnote}>
              <Text style={styles.formNote}>Share opens the system sheet for email, AirDrop, nearby sharing, and other apps.</Text>
            </View>
          </View>
        )}
        {screen === 'detail' && selectedScan && (
          <View style={styles.flex}>
            <PageHeader title="Scan details" eyebrow="SAVED SCAN" onBack={() => setScreen('gallery')} />
            <ScrollView contentContainerStyle={styles.finishedContent}>
              <View style={[styles.finishedCloud, { width: screenWidth, height: screenWidth * 1.05, marginTop: 0 }]}>
                <PointCloudView points={selectedPoints} fill zoom={ZOOM} height={screenWidth * 1.05} />
                <View pointerEvents="none" style={styles.detailCaption}>
                  <Text style={styles.detailDate}>{formatDate(selectedScan.createdAt)}</Text>
                  <Text style={styles.detailMeta}>{selectedScan.pointCount.toLocaleString()} points</Text>
                  <Text style={styles.detailMeta}>{selectedScan.durationSeconds} second sweep</Text>
                </View>
                <View style={styles.detailActions}>
                  <Pressable accessibilityRole="button" accessibilityLabel="Share scan" onPress={() => void shareSavedScan(selectedScan)} style={styles.detailIconButton}>
                    <Share2 color={PALETTE.white} size={19} />
                  </Pressable>
                  <Pressable
                    accessibilityRole="button"
                    accessibilityLabel="Delete scan"
                    style={styles.detailIconButton}
                    onPress={() => {
                      const scan = selectedScan;
                      Alert.alert('Delete saved scan?', `${scan.name} will be removed from this device.`, [
                        { text: 'Cancel', style: 'cancel' },
                        {
                          text: 'Delete',
                          style: 'destructive',
                          onPress: () => {
                            void deleteScan(scan)
                              .then(() => {
                                setScans((current) => current.filter((saved) => saved.id !== scan.id));
                                setScreen('gallery');
                              })
                              .catch((reason: unknown) =>
                                setError(reason instanceof Error ? reason.message : 'Could not delete this scan.'),
                              );
                          },
                        },
                      ]);
                    }}
                  >
                    <Trash2 color={PALETTE.white} size={19} />
                  </Pressable>
                </View>
              </View>
              {selectedScan.photoUri ? (
                <View style={styles.photoCard}>
                  <Image source={{ uri: selectedScan.photoUri }} style={styles.photoPreview} />
                </View>
              ) : null}
            </ScrollView>
          </View>
        )}
        </KeyboardAvoidingView>
      </SafeAreaView>
    </SafeAreaProvider>
  );
}

function SvgRing({ progress }: { progress: number }) {
  const radius = 110;
  const circumference = 2 * Math.PI * radius;
  return (
    <View style={StyleSheet.absoluteFill}>
      <Svg width="100%" height="100%" viewBox="0 0 240 240">
        <Circle cx="120" cy="120" r={radius} stroke="#E4EBE7" strokeWidth={8} fill="none" />
        <Circle
          cx="120"
          cy="120"
          r={radius}
          stroke={PALETTE.forest}
          strokeWidth={8}
          strokeLinecap="round"
          strokeDasharray={`${circumference} ${circumference}`}
          strokeDashoffset={circumference * (1 - progress / 100)}
          fill="none"
          rotation={-90}
          origin="120, 120"
        />
      </Svg>
    </View>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: PALETTE.paper },
  flex: { flex: 1 },
  homePage: { flexGrow: 1, paddingHorizontal: 24, paddingBottom: 25 },
  homeTopRow: { height: 58, flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between' },
  brand: { flexDirection: 'row', alignItems: 'center', gap: 10 },
  brandMark: { width: 37, height: 37, borderRadius: 13, backgroundColor: PALETTE.forest, alignItems: 'center', justifyContent: 'center' },
  brandName: { color: PALETTE.ink, fontSize: 12, fontWeight: '800', letterSpacing: 1.4 },
  brandCaption: { color: PALETTE.muted, fontSize: 9, fontWeight: '600', letterSpacing: 1.1, marginTop: 2 },
  iconButton: { width: 42, height: 42, borderRadius: 14, backgroundColor: PALETTE.white, alignItems: 'center', justifyContent: 'center' },
  heroCopy: { paddingTop: 36, paddingBottom: 27 },
  heroKicker: { color: PALETTE.moss, fontWeight: '800', fontSize: 10, letterSpacing: 1.8, marginBottom: 12 },
  heroTitle: { color: PALETTE.ink, fontSize: 37, lineHeight: 43, letterSpacing: -1.2, fontWeight: '700' },
  heroDescription: { color: PALETTE.muted, fontSize: 14, lineHeight: 21, marginTop: 12, maxWidth: 300 },
  deviceCard: { backgroundColor: PALETTE.white, borderRadius: 20, padding: 15, flexDirection: 'row', alignItems: 'center', gap: 12, borderWidth: 1, borderColor: '#EDF1EE' },
  deviceIcon: { height: 44, width: 44, borderRadius: 15, backgroundColor: PALETTE.softGreen, alignItems: 'center', justifyContent: 'center' },
  deviceInfo: { flex: 1 },
  cardOverline: { color: PALETTE.moss, fontSize: 9, fontWeight: '800', letterSpacing: 1.3, marginBottom: 4 },
  deviceTitle: { color: PALETTE.ink, fontSize: 14, fontWeight: '700' },
  deviceSubline: { color: PALETTE.muted, fontSize: 11, lineHeight: 16, marginTop: 4 },
  refreshButton: { width: 36, height: 36, alignItems: 'center', justifyContent: 'center' },
  homeActions: { marginTop: 32, gap: 12 },
  sectionTitle: { color: PALETTE.ink, fontSize: 16, fontWeight: '700' },
  button: { minHeight: 56, borderRadius: 17, paddingHorizontal: 18, flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 10 },
  button_dark: { backgroundColor: PALETTE.forest },
  button_light: { backgroundColor: PALETTE.lime },
  button_outline: { backgroundColor: '#4B5563' },
  button_danger: { backgroundColor: '#F8EAE8' },
  buttonDisabled: { opacity: 0.45 },
  buttonPressed: { opacity: 0.86, transform: [{ scale: 0.99 }] },
  buttonText: { color: PALETTE.white, fontSize: 15, fontWeight: '700', letterSpacing: 0.1 },
  buttonTextDark: { color: PALETTE.forest },
  connectHint: { flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 5, paddingVertical: 3 },
  connectHintText: { color: PALETTE.moss, fontSize: 12, fontWeight: '600' },
  galleryAction: { backgroundColor: PALETTE.white, borderWidth: 1, borderColor: '#E9EFEB', minHeight: 78, borderRadius: 19, padding: 14, flexDirection: 'row', alignItems: 'center', gap: 12, marginTop: 7 },
  galleryIcon: { width: 42, height: 42, borderRadius: 14, backgroundColor: PALETTE.softGreen, alignItems: 'center', justifyContent: 'center' },
  galleryActionText: { flex: 1 },
  galleryTitle: { color: PALETTE.ink, fontWeight: '700', fontSize: 14 },
  gallerySubline: { color: PALETTE.muted, fontSize: 11, marginTop: 4 },
  homeFooter: { marginTop: 'auto', paddingTop: 25, flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 7 },
  footerDot: { width: 6, height: 6, borderRadius: 3, backgroundColor: PALETTE.moss },
  footerText: { color: PALETTE.muted, fontSize: 10, fontWeight: '500' },
  pageContent: { paddingHorizontal: 22, paddingBottom: 30 },
  pageHeader: { minHeight: 74, paddingHorizontal: 20, flexDirection: 'row', alignItems: 'center', gap: 13 },
  backButton: { width: 40, height: 40, borderRadius: 14, backgroundColor: PALETTE.white, alignItems: 'center', justifyContent: 'center' },
  pageHeading: { flex: 1 },
  eyebrow: { color: PALETTE.moss, fontSize: 9, fontWeight: '800', letterSpacing: 1.4, marginBottom: 3 },
  pageTitle: { color: PALETTE.ink, fontSize: 20, fontWeight: '700', letterSpacing: -0.3 },
  headerActionPlaceholder: { width: 40 },
  setupStatus: { backgroundColor: PALETTE.white, borderRadius: 19, padding: 15, flexDirection: 'row', alignItems: 'center', gap: 12, marginBottom: 14 },
  setupStatusIcon: { width: 43, height: 43, borderRadius: 15, backgroundColor: PALETTE.softGreen, alignItems: 'center', justifyContent: 'center' },
  setupStatusIconConnected: { backgroundColor: '#E7F3CE' },
  formCard: { backgroundColor: PALETTE.white, padding: 17, borderRadius: 20, marginTop: 18, borderWidth: 1, borderColor: '#E9EFEB' },
  formHeading: { flexDirection: 'row', gap: 11, alignItems: 'center', marginBottom: 19 },
  formIcon: { height: 37, width: 37, borderRadius: 13, backgroundColor: PALETTE.softGreen, alignItems: 'center', justifyContent: 'center' },
  inputLabel: { color: PALETTE.muted, fontSize: 9, fontWeight: '800', letterSpacing: 1.2, marginBottom: 7, marginTop: 12 },
  textInput: { height: 48, backgroundColor: '#F6F8F6', borderRadius: 12, borderWidth: 1, borderColor: '#E7EDE9', paddingHorizontal: 13, color: PALETTE.ink, fontSize: 14 },
  formNote: { color: PALETTE.muted, fontSize: 11, lineHeight: 16, textAlign: 'center', marginTop: 11 },
  helperBox: { flexDirection: 'row', gap: 9, alignItems: 'center', paddingHorizontal: 5, paddingTop: 17 },
  helperText: { flex: 1, color: PALETTE.muted, fontSize: 11, lineHeight: 16 },
  scanPageContent: { paddingHorizontal: 20, paddingTop: 4, paddingBottom: 22 },
  introText: { color: PALETTE.muted, fontSize: 13, marginHorizontal: 4, marginBottom: 15 },
  scanCard: { backgroundColor: PALETTE.white, borderRadius: 21, paddingHorizontal: 16, paddingVertical: 17, marginBottom: 14, borderWidth: 1, borderColor: '#E9EFEB', alignItems: 'center' },
  cardTitleRow: { width: '100%', flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center', marginBottom: 1 },
  dragLabel: { color: PALETTE.moss, fontSize: 8, fontWeight: '800', letterSpacing: 1 },
  angleReadouts: { width: '100%', borderTopWidth: 1, borderColor: '#EDF1EE', paddingTop: 13, marginTop: -4, flexDirection: 'row', justifyContent: 'center', alignItems: 'center' },
  angleReadout: { minWidth: 90, alignItems: 'center' },
  readoutLabel: { color: PALETTE.muted, fontSize: 8, fontWeight: '800', letterSpacing: 1.2 },
  readoutValue: { color: PALETTE.ink, fontSize: 17, fontWeight: '700', marginTop: 4 },
  angleRangeDivider: { width: 1, height: 31, backgroundColor: PALETTE.line },
  durationHeading: { width: '100%', flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center' },
  durationBadge: { height: 47, minWidth: 60, borderRadius: 14, backgroundColor: PALETTE.softGreen, alignItems: 'center', justifyContent: 'center', flexDirection: 'row', gap: 3, paddingHorizontal: 8 },
  durationValue: { color: PALETTE.forest, fontWeight: '800', fontSize: 17 },
  durationUnit: { color: PALETTE.moss, fontWeight: '800', fontSize: 8, marginTop: 5 },
  slider: { width: '100%', height: 42, marginTop: 13 },
  sliderLabels: { width: '100%', flexDirection: 'row', justifyContent: 'space-between', marginTop: -3 },
  sliderLabel: { color: PALETTE.muted, fontSize: 9, fontWeight: '700', letterSpacing: 0.7 },
  speedNote: { alignSelf: 'flex-start', flexDirection: 'row', alignItems: 'center', gap: 7, marginTop: 14, backgroundColor: '#F5F8F5', paddingVertical: 8, paddingHorizontal: 10, borderRadius: 10 },
  speedNoteText: { color: PALETTE.muted, fontSize: 10, fontWeight: '500' },
  bottomBar: { paddingHorizontal: 22, paddingTop: 12, paddingBottom: 15, backgroundColor: PALETTE.paper },
  bottomHint: { textAlign: 'center', color: PALETTE.muted, fontSize: 10, marginTop: 9 },
  scanningPage: { flex: 1, paddingHorizontal: 24, alignItems: 'center' },
  scanningTop: { alignItems: 'center', paddingTop: 46 },
  liveBadge: { flexDirection: 'row', alignItems: 'center', gap: 7, backgroundColor: PALETTE.softGreen, borderRadius: 20, paddingHorizontal: 11, paddingVertical: 7 },
  liveDot: { width: 6, height: 6, borderRadius: 3, backgroundColor: '#72A977' },
  liveText: { color: PALETTE.forest, fontSize: 9, letterSpacing: 1, fontWeight: '800' },
  scanningTitle: { color: PALETTE.ink, fontSize: 30, fontWeight: '700', letterSpacing: -0.6, marginTop: 19 },
  scanningSubtitle: { color: PALETTE.muted, fontSize: 12, textAlign: 'center', marginTop: 8, maxWidth: 260, lineHeight: 18 },
  progressWrap: { alignItems: 'center', justifyContent: 'center', flex: 1, width: '100%', minHeight: 310 },
  progressRing: { width: 104, height: 104, alignItems: 'center', justifyContent: 'center' },
  cloudWrap: { flex: 1, alignSelf: 'stretch', marginHorizontal: -24, paddingBottom: 6 },
  progressOverlay: { position: 'absolute', top: 8, right: 16, backgroundColor: 'rgba(255,255,255,0.9)', borderRadius: 60, padding: 4 },
  homingTitle: { color: PALETTE.ink, fontSize: 20, fontWeight: '700', marginTop: 22 },
  checkRow: { flexDirection: 'row', alignItems: 'center', gap: 14, backgroundColor: PALETTE.white, borderRadius: 20, padding: 18, marginTop: 14 },
  checkBox: { width: 26, height: 26, borderRadius: 8, borderWidth: 2, borderColor: '#B8C9C0', alignItems: 'center', justifyContent: 'center' },
  checkBoxOn: { backgroundColor: PALETTE.forest, borderColor: PALETTE.forest },
  photoLoading: { ...StyleSheet.absoluteFillObject, backgroundColor: 'rgba(247,249,247,0.85)', alignItems: 'center', justifyContent: 'center' },
  progressCenter: { alignItems: 'center', justifyContent: 'center' },
  secondsValue: { color: PALETTE.ink, fontSize: 30, lineHeight: 34, fontWeight: '600', letterSpacing: -2 },
  secondsLabel: { color: PALETTE.muted, fontSize: 9, fontWeight: '800', letterSpacing: 1.5, marginTop: 3 },
  progressPercent: { color: PALETTE.forest, fontSize: 10, letterSpacing: 1.5, fontWeight: '800', marginTop: 22 },
  progressStatus: { color: PALETTE.muted, fontSize: 12, marginTop: 8 },
  finishedContent: { flexGrow: 1, paddingHorizontal: 23, paddingTop: 17, paddingBottom: 25 },
  completeIcon: { width: 44, height: 44, borderRadius: 18, backgroundColor: '#E5F1D0', alignItems: 'center', justifyContent: 'center' },
  finishedTitle: { color: PALETTE.ink, fontSize: 31, lineHeight: 36, fontWeight: '700', letterSpacing: -0.7, marginTop: 16 },
  detailCaption: { position: 'absolute', left: 24, bottom: 16 },
  detailDate: { color: '#FFFFFF', fontSize: 16, fontWeight: '700' },
  detailMeta: { color: 'rgba(255,255,255,0.75)', fontSize: 12, marginTop: 2 },
  detailActions: { position: 'absolute', right: 24, top: 16, flexDirection: 'row', gap: 10 },
  detailIconButton: { width: 42, height: 42, borderRadius: 21, backgroundColor: 'rgba(75,85,99,0.85)', alignItems: 'center', justifyContent: 'center' },
  cloudCaption: { color: PALETTE.muted, fontSize: 12, marginTop: 10, textAlign: 'center' },
  finishedDescription: { color: PALETTE.muted, fontSize: 12, marginTop: 8 },
  finishedCloud: { marginHorizontal: -23, marginTop: 18, overflow: 'hidden' },
  detailCloud: { marginTop: 4 },
  thumb: { width: 54, height: 54, borderRadius: 14, backgroundColor: PALETTE.softGreen, overflow: 'hidden', alignItems: 'center', justifyContent: 'center' },
  thumbImage: { width: '100%', height: '100%' },
  selectBox: { width: 24, height: 24, borderRadius: 8, borderWidth: 2, borderColor: '#B8C9C0', alignItems: 'center', justifyContent: 'center' },
  selectBoxOn: { backgroundColor: PALETTE.forest, borderColor: PALETTE.forest },
  selectBar: { flexDirection: 'row', gap: 10, paddingHorizontal: 18, paddingTop: 8 },
  selectBarButton: { flex: 1 },
  headerLink: { color: PALETTE.forest, fontWeight: '700', fontSize: 13, paddingHorizontal: 4 },
  photoCard: { marginTop: 23, borderWidth: 1, borderColor: '#DDE7E0', borderStyle: 'dashed', borderRadius: 19, minHeight: 147, overflow: 'hidden', backgroundColor: '#F7F9F7' },
  photoPlaceholder: { minHeight: 147, alignItems: 'center', justifyContent: 'center' },
  photoIcon: { width: 43, height: 43, borderRadius: 15, backgroundColor: PALETTE.softGreen, alignItems: 'center', justifyContent: 'center' },
  photoTitle: { color: PALETTE.ink, fontSize: 13, fontWeight: '700', marginTop: 10 },
  photoSubline: { color: PALETTE.muted, fontSize: 10, marginTop: 4 },
  photoPreview: { height: 190, width: '100%', resizeMode: 'cover' },
  replacePhoto: { position: 'absolute', right: 10, bottom: 10, color: PALETTE.white, fontSize: 8, fontWeight: '800', letterSpacing: 1, backgroundColor: '#17332CCC', borderRadius: 8, paddingVertical: 6, paddingHorizontal: 8 },
  finishedActions: { marginTop: 19, gap: 11 },
  scanCountBadge: { width: 34, height: 34, borderRadius: 12, backgroundColor: PALETTE.softGreen, alignItems: 'center', justifyContent: 'center' },
  scanCount: { color: PALETTE.forest, fontSize: 13, fontWeight: '800' },
  galleryList: { paddingHorizontal: 18, paddingTop: 9, paddingBottom: 12, gap: 11 },
  savedScanCard: { backgroundColor: PALETTE.white, borderRadius: 17, padding: 13, flexDirection: 'row', alignItems: 'center', gap: 11, borderWidth: 1, borderColor: '#E9EFEB' },
  savedScanIcon: { width: 42, height: 42, borderRadius: 14, backgroundColor: PALETTE.softGreen, alignItems: 'center', justifyContent: 'center' },
  savedScanInfo: { flex: 1 },
  savedScanName: { color: PALETTE.ink, fontSize: 12, fontWeight: '700' },
  savedScanDetails: { color: PALETTE.muted, fontSize: 10, marginTop: 4 },
  savedScanFile: { color: '#91A097', fontSize: 9, marginTop: 4 },
  savedScanActions: { flexDirection: 'row', gap: 3 },
  smallAction: { height: 34, width: 34, alignItems: 'center', justifyContent: 'center' },
  emptyState: { flex: 1, alignItems: 'center', justifyContent: 'center', paddingHorizontal: 50 },
  emptyIcon: { width: 58, height: 58, borderRadius: 20, backgroundColor: PALETTE.softGreen, alignItems: 'center', justifyContent: 'center' },
  emptyTitle: { color: PALETTE.ink, fontSize: 18, fontWeight: '700', marginTop: 16 },
  emptyText: { color: PALETTE.muted, textAlign: 'center', fontSize: 12, lineHeight: 18, marginTop: 7 },
  galleryFootnote: { paddingHorizontal: 25, paddingTop: 8, paddingBottom: 15 },
  errorBanner: { marginHorizontal: 14, marginTop: 5, marginBottom: 2, minHeight: 38, paddingHorizontal: 12, borderRadius: 12, backgroundColor: '#F8EAE8', flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between' },
  errorText: { flex: 1, color: PALETTE.danger, fontSize: 11, lineHeight: 15, paddingVertical: 8 },
  dismissError: { color: PALETTE.danger, fontSize: 22, marginLeft: 10, paddingHorizontal: 4 },
});
