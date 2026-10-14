import AsyncStorage from '@react-native-async-storage/async-storage';

const KEY = 'scanner.lastHomeIp';

export async function getLastScannerIp(): Promise<string | null> {
  return AsyncStorage.getItem(KEY);
}

export async function setLastScannerIp(ip: string): Promise<void> {
  await AsyncStorage.setItem(KEY, ip);
}
