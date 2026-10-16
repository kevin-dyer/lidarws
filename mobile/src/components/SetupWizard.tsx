import { useEffect, useRef, useState } from 'react';
import {
  ActivityIndicator,
  KeyboardAvoidingView,
  Linking,
  Platform,
  Pressable,
  ScrollView,
  StyleSheet,
  Text,
  TextInput,
  View,
} from 'react-native';
import { ArrowLeft, Check, X } from 'lucide-react-native';
import WifiManager from 'react-native-wifi-reborn';
import type { ScannerClient } from '../scanner/ScannerClient';
import { requestLocalNetworkAccess, requestWifiPermissions } from '../scanner/permissions';
import { loadScannerConfig, saveScannerConfig } from '../storage/appData';
import { getLastScannerIp, setLastScannerIp } from '../storage/scannerAddress';
import { PALETTE } from '../theme';

type Phase = 'prepare' | 'join' | 'home' | 'return' | 'done';
type CheckState = 'running' | 'ok' | 'fail';
interface Check {
  label: string;
  state: CheckState;
  detail?: string;
}

const PHASES: Phase[] = ['prepare', 'join', 'home', 'return', 'done'];
const TITLES: Record<Phase, string> = {
  prepare: 'Get ready',
  join: 'Connect to the scanner',
  home: 'Choose your home Wi-Fi',
  return: 'Back to your home Wi-Fi',
  done: 'All set',
};
const SCANNER_AP_ADDRESS = '192.168.4.1';

const sleep = (ms: number): Promise<void> => new Promise((resolve) => setTimeout(resolve, ms));
const clean = (ssid: string): string => ssid.replace(/^"|"$/g, '');
const messageOf = (reason: unknown, fallback: string): string => (reason instanceof Error ? reason.message : fallback);

interface Props {
  scanner: ScannerClient;
  onClose: () => void;
  onConnected: (address: string) => void;
}

export function SetupWizard({ scanner, onClose, onConnected }: Props) {
  const [phase, setPhase] = useState<Phase>('prepare');
  const [checks, setChecks] = useState<Check[]>([]);
  const [busy, setBusy] = useState(false);
  const [apSsid, setApSsid] = useState('TiltLidar-Bridge');
  const [apPassword, setApPassword] = useState('');
  const [homeSsid, setHomeSsid] = useState('');
  const [homePassword, setHomePassword] = useState('');
  const [finalAddress, setFinalAddress] = useState<string | null>(null);
  const boardHomeIp = useRef<string | null>(null);
  const [failedHint, setFailedHint] = useState<string | null>(null);

  const goTo = (next: Phase): void => {
    setChecks([]);
    setFailedHint(null);
    setPhase(next);
  };

  // Runs steps in order, showing each one's progress and stopping at the first failure.
  const runChecks = async (steps: Array<{ label: string; run: () => Promise<string | void> }>): Promise<boolean> => {
    setBusy(true);
    setFailedHint(null);
    setChecks(steps.map((step) => ({ label: step.label, state: 'running' as CheckState })).slice(0, 1));
    const update = (index: number, patch: Partial<Check>): void =>
      setChecks((current) => current.map((check, i) => (i === index ? { ...check, ...patch } : check)));
    try {
      for (let index = 0; index < steps.length; index += 1) {
        const step = steps[index];
        if (index > 0) setChecks((current) => [...current, { label: step.label, state: 'running' }]);
        try {
          const detail = await step.run();
          update(index, { state: 'ok', detail: detail || undefined });
        } catch (reason) {
          update(index, { state: 'fail', detail: messageOf(reason, 'Something went wrong.') });
          return false;
        }
      }
      return true;
    } finally {
      setBusy(false);
    }
  };

  const prepare = async (): Promise<void> => {
    const ok = await runChecks([
      {
        label: 'Permissions granted',
        run: async () => {
          await requestWifiPermissions();
          await requestLocalNetworkAccess();
        },
      },
      {
        label: 'Detect your current Wi-Fi',
        run: async () => {
          try {
            const current = clean(await WifiManager.getCurrentWifiSSID());
            if (current && current !== '<unknown ssid>' && !current.startsWith('TiltLidar')) {
              setHomeSsid((existing) => existing || current);
              return current;
            }
          } catch {
            // Reading the SSID is a convenience only.
          }
          return 'Not detected — you can type it later';
        },
      },
    ]);
    if (ok) {
      await sleep(700);
      goTo('join');
    }
  };

  const verifyScanner = async (): Promise<string> => {
    scanner.resetSocket();
    if (!(await scanner.probe(SCANNER_AP_ADDRESS, 6000))) {
      throw new Error(
        'The scanner did not answer. Make sure it is powered on (wait ~15 s after power-up) and the phone is on its Wi-Fi.',
      );
    }
    return `Scanner answered at ${SCANNER_AP_ADDRESS}`;
  };

  const joinAutomatically = async (): Promise<void> => {
    const ok = await runChecks([
      {
        label: `Join “${apSsid.trim()}”`,
        run: async () => {
          try {
            await WifiManager.connectToProtectedSSID(apSsid.trim(), apPassword, false, false);
          } catch (reason) {
            throw new Error(
              `${messageOf(reason, 'Could not join.')} Check the name and password, or join it yourself in Wi-Fi settings and tap “I’m already connected”.`,
            );
          }
          await sleep(1500);
        },
      },
      { label: 'Scanner responds', run: verifyScanner },
    ]);
    if (ok) {
      await sleep(700);
      goTo('home');
    }
  };

  const alreadyConnected = async (): Promise<void> => {
    const ok = await runChecks([{ label: 'Scanner responds', run: verifyScanner }]);
    if (ok) {
      await sleep(700);
      goTo('home');
    }
  };

  const sendHomeWifi = async (): Promise<void> => {
    boardHomeIp.current = null;
    const ok = await runChecks([
      {
        label: 'Send Wi-Fi details to scanner',
        run: async () => {
          await scanner.provisionHomeWifi(homeSsid.trim(), homePassword);
        },
      },
      {
        label: `Scanner joins “${homeSsid.trim()}”`,
        run: async () => {
          let answered = false;
          for (let attempt = 0; attempt < 12; attempt += 1) {
            await sleep(2000);
            try {
              const status = await scanner.status();
              answered = true;
              if (status.joined) {
                boardHomeIp.current = status.ip;
                return `Connected, address ${status.ip}`;
              }
            } catch {
              // The scanner may briefly drop its own Wi-Fi while it switches channels.
            }
          }
          if (answered) {
            throw new Error(
              'The scanner could not join that network. Check the password, and use a 2.4 GHz network (the scanner cannot use 5 GHz).',
            );
          }
          return 'Could not confirm — the phone left the scanner Wi-Fi. Continuing.';
        },
      },
    ]);
    if (ok) {
      void saveScannerConfig({ homeSsid: homeSsid.trim(), apSsid: apSsid.trim(), apPassword }).catch(() => {});
      setHomePassword('');
      await sleep(900);
      goTo('return');
    }
  };

  const returnHome = async (): Promise<void> => {
    const target = homeSsid.trim();
    const ok = await runChecks([
      {
        label: `Phone back on “${target}”`,
        run: async () => {
          await (Platform.OS === 'ios' ? WifiManager.disconnectFromSSID(apSsid.trim()) : WifiManager.disconnect()).catch(
            () => {},
          );
          for (let attempt = 0; attempt < 15; attempt += 1) {
            await sleep(1500);
            try {
              if (clean(await WifiManager.getCurrentWifiSSID()) === target) return;
            } catch {
              await sleep(3000);
              return 'Could not read the Wi-Fi name; continuing';
            }
          }
          throw new Error(
            `The phone is not on “${target}”. Join it in Wi-Fi settings, come back, and tap Retry.`,
          );
        },
      },
      {
        label: 'Find scanner on home Wi-Fi',
        run: async () => {
          const hints = boardHomeIp.current ? [boardHomeIp.current] : [];
          scanner.resetSocket();
          for (let attempt = 0; attempt < 3; attempt += 1) {
            try {
              const address = await scanner.discover(4000, hints);
              setFinalAddress(address);
              return `Found at ${address}`;
            } catch {
              scanner.resetSocket();
            }
          }
          setFailedHint(
            'Both devices must be on the same network. Guest networks, “client isolation” and some mesh routers block this. You can still use the scanner’s own Wi-Fi in the field.',
          );
          throw new Error('The scanner did not answer on your home Wi-Fi.');
        },
      },
    ]);
    if (ok) {
      if (boardHomeIp.current) void setLastScannerIp(boardHomeIp.current).catch(() => {});
      await sleep(700);
      goTo('done');
    }
  };

  const finish = (): void => {
    if (finalAddress) onConnected(finalAddress);
    onClose();
  };

  const phaseIndex = PHASES.indexOf(phase);
  const apReady = apSsid.trim().length > 0 && apPassword.length >= 8;
  const homeReady = homeSsid.trim().length > 0 && (homePassword.length === 0 || homePassword.length >= 8);
  const hasFailure = checks.some((check) => check.state === 'fail');

  return (
    <KeyboardAvoidingView style={styles.flex} behavior={Platform.OS === 'ios' ? 'padding' : undefined}>
      <View style={styles.header}>
        <Pressable accessibilityLabel="Close setup" onPress={onClose} style={styles.backButton}>
          <ArrowLeft color={PALETTE.ink} size={22} />
        </Pressable>
        <View>
          <Text style={styles.title}>{TITLES[phase]}</Text>
          <Text style={styles.eyebrow}>STEP {Math.min(phaseIndex + 1, 4)} OF 4</Text>
        </View>
      </View>
      <View style={styles.progressRow}>
        {PHASES.slice(0, 4).map((item, index) => (
          <View key={item} style={[styles.progressBar, index <= phaseIndex && styles.progressBarActive]} />
        ))}
      </View>

      <ScrollView contentContainerStyle={styles.content} keyboardShouldPersistTaps="handled">
        {phase === 'prepare' && (
          <>
            <Text style={styles.lead}>We’ll connect your scanner to your home Wi-Fi in a few quick steps, checking each one as we go.</Text>
            <View style={styles.card}>
              <Text style={styles.bullet}>1. Power on the scanner and wait about 15 seconds.</Text>
              <Text style={styles.bullet}>2. Make sure your phone is on your home Wi-Fi right now (2.4 GHz works best).</Text>
              <Text style={styles.bullet}>3. Allow the Wi-Fi / nearby-devices permissions when asked.</Text>
            </View>
            <Checks checks={checks} />
            <Button title={hasFailure ? 'Retry' : 'Start'} busy={busy} onPress={() => void prepare()} />
          </>
        )}

        {phase === 'join' && (
          <>
            <Text style={styles.lead}>
              Your phone joins the scanner’s own Wi-Fi so it can hand over your home network details.
            </Text>
            <View style={styles.card}>
              <Text style={styles.label}>SCANNER WI-FI NAME</Text>
              <TextInput autoCapitalize="none" autoCorrect={false} onChangeText={setApSsid} style={styles.input} value={apSsid} />
              <Text style={styles.label}>SCANNER WI-FI PASSWORD</Text>
              <TextInput
                autoCapitalize="none"
                autoCorrect={false}
                onChangeText={setApPassword}
                placeholder="Set in wifi_config.h (8+ characters)"
                placeholderTextColor="#9AA79F"
                secureTextEntry
                style={styles.input}
                value={apPassword}
              />
            </View>
            <Checks checks={checks} />
            <Button title={hasFailure ? 'Retry' : 'Join scanner Wi-Fi'} busy={busy} disabled={!apReady} onPress={() => void joinAutomatically()} />
            <Button
              title="I’m already connected to the scanner Wi-Fi"
              kind="outline"
              busy={busy}
              onPress={() => void alreadyConnected()}
            />
            {hasFailure && <Button title="Open Wi-Fi settings" kind="outline" onPress={() => void Linking.openSettings()} />}
          </>
        )}

        {phase === 'home' && (
          <>
            <Text style={styles.lead}>
              Enter the network the scanner should use at home. It remembers it, and falls back to its own Wi-Fi when this one isn’t available.
            </Text>
            <View style={styles.card}>
              <Text style={styles.label}>HOME WI-FI NAME</Text>
              <TextInput
                autoCapitalize="none"
                autoCorrect={false}
                onChangeText={setHomeSsid}
                placeholder="Your home Wi-Fi"
                placeholderTextColor="#9AA79F"
                style={styles.input}
                value={homeSsid}
              />
              <Text style={styles.label}>HOME WI-FI PASSWORD</Text>
              <TextInput
                autoCapitalize="none"
                autoCorrect={false}
                onChangeText={setHomePassword}
                placeholder="Password (8+ characters)"
                placeholderTextColor="#9AA79F"
                secureTextEntry
                style={styles.input}
                value={homePassword}
              />
              <Text style={styles.note}>Must be a 2.4 GHz network. The password is stored on the scanner only.</Text>
            </View>
            <Checks checks={checks} />
            <Button title={hasFailure ? 'Retry' : 'Send to scanner'} busy={busy} disabled={!homeReady} onPress={() => void sendHomeWifi()} />
          </>
        )}

        {phase === 'return' && (
          <>
            <Text style={styles.lead}>
              Now your phone returns to “{homeSsid.trim()}” and looks for the scanner there.
            </Text>
            <Checks checks={checks} />
            {failedHint && <Text style={styles.hint}>{failedHint}</Text>}
            <Button title={hasFailure ? 'Retry' : 'Reconnect and search'} busy={busy} onPress={() => void returnHome()} />
            {hasFailure && <Button title="Open Wi-Fi settings" kind="outline" onPress={() => void Linking.openSettings()} />}
            {hasFailure && (
              <Button
                title="Skip — I’ll use the scanner’s Wi-Fi"
                kind="outline"
                onPress={() => {
                  onClose();
                }}
              />
            )}
          </>
        )}

        {phase === 'done' && (
          <>
            <View style={styles.doneIcon}>
              <Check color={PALETTE.forest} size={34} />
            </View>
            <Text style={styles.doneTitle}>Scanner connected</Text>
            <Text style={styles.lead}>
              Found at {finalAddress} on “{homeSsid.trim()}”. Away from home, join the scanner’s own Wi-Fi from this setup.
            </Text>
            <Button title="Finish" onPress={finish} />
          </>
        )}
      </ScrollView>
    </KeyboardAvoidingView>
  );
}

type ConnectPhase = 'power' | 'network' | 'ap';

/** Walks the user through reconnecting to a scanner that was already set up. */
export function ConnectWizard({ scanner, onClose, onConnected }: Props) {
  const [phase, setPhase] = useState<ConnectPhase>('power');
  const [checks, setChecks] = useState<Check[]>([]);
  const [busy, setBusy] = useState(false);
  const [homeSsid, setHomeSsid] = useState<string | undefined>();
  const [apSsid, setApSsid] = useState('TiltLidar-Bridge');
  const [apPassword, setApPassword] = useState('');

  useEffect(() => {
    void loadScannerConfig()
      .then((config) => {
        if (!config) return;
        setHomeSsid(config.homeSsid);
        if (config.apSsid) setApSsid(config.apSsid);
        if (config.apPassword) setApPassword(config.apPassword);
      })
      .catch(() => {});
  }, []);

  const steps = homeSsid ? (['power', 'network', 'ap'] as ConnectPhase[]) : (['power', 'ap'] as ConnectPhase[]);
  const stepIndex = steps.indexOf(phase);
  const failed = checks.some((check) => check.state === 'fail');

  const run = async (label: string, work: () => Promise<string | void>): Promise<boolean> => {
    setBusy(true);
    setChecks([{ label, state: 'running' }]);
    try {
      const detail = await work();
      setChecks([{ label, state: 'ok', detail: detail || undefined }]);
      return true;
    } catch (reason) {
      setChecks([{ label, state: 'fail', detail: messageOf(reason, 'Something went wrong.') }]);
      return false;
    } finally {
      setBusy(false);
    }
  };

  const connected = async (address: string): Promise<void> => {
    await sleep(600);
    onConnected(address);
    onClose();
  };

  const findScanner = async (): Promise<string> => {
    scanner.resetSocket();
    const last = await getLastScannerIp().catch(() => null);
    return scanner.discover(5000, last ? [last] : []);
  };

  const checkPower = async (): Promise<void> => {
    let address = '';
    const ok = await run('Looking for the scanner', async () => {
      address = await findScanner();
      return `Found at ${address}`;
    });
    if (ok) await connected(address);
  };

  const checkNetwork = async (): Promise<void> => {
    let address = '';
    const ok = await run(`Phone on “${homeSsid}”`, async () => {
      await requestWifiPermissions().catch(() => {});
      let current = '';
      try {
        current = clean(await WifiManager.getCurrentWifiSSID());
      } catch {
        // Reading the SSID is best effort; searching below is the real test.
      }
      if (current && current !== '<unknown ssid>' && current !== homeSsid) {
        throw new Error(`The phone is on “${current}”. Switch to “${homeSsid}” in Wi-Fi settings, then try again.`);
      }
      address = await findScanner();
      return `Scanner found at ${address}`;
    });
    if (ok) await connected(address);
  };

  const joinScanner = async (): Promise<void> => {
    const ok = await run(`Join “${apSsid.trim()}”`, async () => {
      await requestWifiPermissions();
      try {
        await WifiManager.connectToProtectedSSID(apSsid.trim(), apPassword, false, false);
      } catch (reason) {
        throw new Error(
          `${messageOf(reason, 'Could not join.')} Check the password, or join it in Wi-Fi settings and tap Try again.`,
        );
      }
      await sleep(1500);
      scanner.resetSocket();
      if (!(await scanner.probe(SCANNER_AP_ADDRESS, 6000))) {
        throw new Error('Joined the Wi-Fi, but the scanner did not answer. Wait a few seconds and try again.');
      }
      void saveScannerConfig({ apSsid: apSsid.trim(), apPassword }).catch(() => {});
      return 'Connected to the scanner';
    });
    if (ok) await connected(SCANNER_AP_ADDRESS);
  };

  const next = (): void => {
    setChecks([]);
    setPhase(steps[Math.min(stepIndex + 1, steps.length - 1)]);
  };

  const titles: Record<ConnectPhase, string> = {
    power: 'Power and range',
    network: 'Same Wi-Fi network',
    ap: 'Use the scanner’s Wi-Fi',
  };

  return (
    <KeyboardAvoidingView style={styles.flex} behavior={Platform.OS === 'ios' ? 'padding' : undefined}>
      <View style={styles.header}>
        <Pressable accessibilityLabel="Close" onPress={onClose} style={styles.backButton}>
          <ArrowLeft color={PALETTE.ink} size={22} />
        </Pressable>
        <View>
          <Text style={styles.title}>{titles[phase]}</Text>
          <Text style={styles.eyebrow}>CONNECT · STEP {stepIndex + 1} OF {steps.length}</Text>
        </View>
      </View>
      <View style={styles.progressRow}>
        {steps.map((item, index) => (
          <View key={item} style={[styles.progressBar, index <= stepIndex && styles.progressBarActive]} />
        ))}
      </View>
      <ScrollView contentContainerStyle={styles.content} keyboardShouldPersistTaps="handled">
        {phase === 'power' && (
          <>
            <Text style={styles.lead}>First, make sure the scanner is powered on and within range of your Wi-Fi router or your phone.</Text>
            <View style={styles.card}>
              <Text style={styles.bullet}>1. Check that the scanner has power.</Text>
              <Text style={styles.bullet}>2. Wait about 15 seconds after power-up.</Text>
              <Text style={styles.bullet}>3. Move closer to the scanner or router if you are far away.</Text>
            </View>
            <Checks checks={checks} />
            <Button title={failed ? 'Try again' : 'Check connection'} busy={busy} onPress={() => void checkPower()} />
            {failed && <Button title="Still not connecting — next step" kind="outline" onPress={next} />}
          </>
        )}

        {phase === 'network' && (
          <>
            <Text style={styles.lead}>
              Your scanner was set up on “{homeSsid}”. Your phone needs to be on the same network.
            </Text>
            <Checks checks={checks} />
            <Button title={failed ? 'Try again' : 'Check network'} busy={busy} onPress={() => void checkNetwork()} />
            {failed && <Button title="Open Wi-Fi settings" kind="outline" onPress={() => void Linking.openSettings()} />}
            {failed && <Button title="Still not connecting — next step" kind="outline" onPress={next} />}
          </>
        )}

        {phase === 'ap' && (
          <>
            <Text style={styles.lead}>
              Away from home, or if home Wi-Fi is unavailable, your phone can join the scanner’s own Wi-Fi directly.
            </Text>
            <View style={styles.card}>
              <Text style={styles.label}>SCANNER WI-FI NAME</Text>
              <TextInput autoCapitalize="none" autoCorrect={false} onChangeText={setApSsid} style={styles.input} value={apSsid} />
              <Text style={styles.label}>SCANNER WI-FI PASSWORD</Text>
              <TextInput
                autoCapitalize="none"
                autoCorrect={false}
                onChangeText={setApPassword}
                placeholder="Set in wifi_config.h (8+ characters)"
                placeholderTextColor="#9AA79F"
                secureTextEntry
                style={styles.input}
                value={apPassword}
              />
            </View>
            <Checks checks={checks} />
            <Button
              title={failed ? 'Try again' : 'Connect to scanner Wi-Fi'}
              busy={busy}
              disabled={apSsid.trim().length === 0 || apPassword.length < 8}
              onPress={() => void joinScanner()}
            />
            {failed && <Button title="Open Wi-Fi settings" kind="outline" onPress={() => void Linking.openSettings()} />}
          </>
        )}
      </ScrollView>
    </KeyboardAvoidingView>
  );
}

function Checks({ checks }: { checks: Check[] }) {
  if (checks.length === 0) return null;
  return (
    <View style={styles.card}>
      {checks.map((check) => (
        <View key={check.label} style={styles.checkRow}>
          <View style={[styles.checkIcon, check.state === 'ok' && styles.checkOk, check.state === 'fail' && styles.checkFail]}>
            {check.state === 'running' && <ActivityIndicator color={PALETTE.forest} size="small" />}
            {check.state === 'ok' && <Check color={PALETTE.forest} size={16} />}
            {check.state === 'fail' && <X color={PALETTE.white} size={16} />}
          </View>
          <View style={styles.flex}>
            <Text style={styles.checkLabel}>{check.label}</Text>
            {check.detail ? (
              <Text style={[styles.checkDetail, check.state === 'fail' && styles.checkDetailFail]}>{check.detail}</Text>
            ) : null}
          </View>
        </View>
      ))}
    </View>
  );
}

function Button({
  title,
  onPress,
  busy = false,
  disabled = false,
  kind = 'solid',
}: {
  title: string;
  onPress: () => void;
  busy?: boolean;
  disabled?: boolean;
  kind?: 'solid' | 'outline';
}) {
  const off = busy || disabled;
  return (
    <Pressable
      accessibilityRole="button"
      disabled={off}
      onPress={onPress}
      style={[styles.button, kind === 'outline' && styles.buttonOutline, off && styles.buttonOff]}
    >
      {busy && kind === 'solid' ? <ActivityIndicator color={PALETTE.white} /> : null}
      <Text style={[styles.buttonText, kind === 'outline' && styles.buttonTextOutline]}>{title}</Text>
    </Pressable>
  );
}

const styles = StyleSheet.create({
  flex: { flex: 1 },
  header: { flexDirection: 'row', alignItems: 'center', gap: 14, paddingHorizontal: 20, paddingTop: 12 },
  backButton: {
    width: 44,
    height: 44,
    borderRadius: 22,
    backgroundColor: PALETTE.white,
    alignItems: 'center',
    justifyContent: 'center',
    borderWidth: 1,
    borderColor: PALETTE.line,
  },
  eyebrow: { color: PALETTE.muted, fontSize: 9, fontWeight: '600', letterSpacing: 1.1, marginTop: 2 },
  title: { color: PALETTE.ink, fontSize: 12, fontWeight: '800', letterSpacing: 1.4, textTransform: 'uppercase' },
  progressRow: { flexDirection: 'row', gap: 6, paddingHorizontal: 20, paddingTop: 14 },
  progressBar: { flex: 1, height: 4, borderRadius: 2, backgroundColor: PALETTE.line },
  progressBarActive: { backgroundColor: PALETTE.forest },
  content: { padding: 20, gap: 16 },
  lead: { color: PALETTE.muted, fontSize: 15, lineHeight: 22 },
  card: {
    backgroundColor: PALETTE.white,
    borderRadius: 20,
    padding: 18,
    gap: 10,
    borderWidth: 1,
    borderColor: PALETTE.line,
  },
  bullet: { color: PALETTE.ink, fontSize: 15, lineHeight: 22 },
  label: { color: PALETTE.muted, fontSize: 11, fontWeight: '700', letterSpacing: 1, marginTop: 4 },
  input: {
    backgroundColor: PALETTE.paper,
    borderRadius: 14,
    paddingHorizontal: 14,
    paddingVertical: 13,
    fontSize: 16,
    color: PALETTE.ink,
  },
  note: { color: PALETTE.muted, fontSize: 12, lineHeight: 18 },
  hint: { color: PALETTE.muted, fontSize: 13, lineHeight: 19 },
  checkRow: { flexDirection: 'row', alignItems: 'flex-start', gap: 12 },
  checkIcon: {
    width: 26,
    height: 26,
    borderRadius: 13,
    backgroundColor: PALETTE.softGreen,
    alignItems: 'center',
    justifyContent: 'center',
  },
  checkOk: { backgroundColor: PALETTE.lime },
  checkFail: { backgroundColor: PALETTE.danger },
  checkLabel: { color: PALETTE.ink, fontSize: 15, fontWeight: '600', paddingTop: 3 },
  checkDetail: { color: PALETTE.muted, fontSize: 13, lineHeight: 18, marginTop: 2 },
  checkDetailFail: { color: PALETTE.danger },
  button: {
    minHeight: 54,
    borderRadius: 27,
    backgroundColor: PALETTE.forest,
    alignItems: 'center',
    justifyContent: 'center',
    flexDirection: 'row',
    gap: 10,
    paddingHorizontal: 20,
  },
  buttonOutline: { backgroundColor: PALETTE.white, borderWidth: 1.5, borderColor: PALETTE.forest },
  buttonOff: { opacity: 0.45 },
  buttonText: { color: PALETTE.white, fontSize: 16, fontWeight: '700' },
  buttonTextOutline: { color: PALETTE.forest },
  doneIcon: {
    alignSelf: 'center',
    width: 76,
    height: 76,
    borderRadius: 38,
    backgroundColor: PALETTE.lime,
    alignItems: 'center',
    justifyContent: 'center',
    marginTop: 20,
  },
  doneTitle: { color: PALETTE.ink, fontSize: 24, fontWeight: '700', textAlign: 'center' },
});
