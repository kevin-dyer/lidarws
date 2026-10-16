import AsyncStorage from '@react-native-async-storage/async-storage';
import { listScans, deleteScan } from './scans';

const PROFILE_KEY = 'profile.v1';
const CONFIG_KEY = 'scanner.config.v1';

export interface Profile {
  username: string;
  email: string;
}

export interface ScannerConfig {
  homeSsid?: string;
  apSsid?: string;
  apPassword?: string;
}

export async function loadProfile(): Promise<Profile> {
  const raw = await AsyncStorage.getItem(PROFILE_KEY);
  return raw ? { username: '', email: '', ...(JSON.parse(raw) as Partial<Profile>) } : { username: '', email: '' };
}

export async function saveProfile(profile: Profile): Promise<void> {
  await AsyncStorage.setItem(PROFILE_KEY, JSON.stringify(profile));
}

export async function loadScannerConfig(): Promise<ScannerConfig | null> {
  const raw = await AsyncStorage.getItem(CONFIG_KEY);
  return raw ? (JSON.parse(raw) as ScannerConfig) : null;
}

export async function saveScannerConfig(patch: ScannerConfig): Promise<void> {
  const current = (await loadScannerConfig()) ?? {};
  await AsyncStorage.setItem(CONFIG_KEY, JSON.stringify({ ...current, ...patch }));
}

/** Permanently removes every scan file, the scanner configuration and the profile. */
export async function deleteAllData(): Promise<void> {
  const scans = await listScans();
  await Promise.all(scans.map((scan) => deleteScan(scan).catch(() => {})));
  await AsyncStorage.clear();
}
