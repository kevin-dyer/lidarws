import { PermissionsAndroid, Platform } from 'react-native';

type AndroidPermission = Parameters<typeof PermissionsAndroid.request>[0];

// On Android 16+ this may be granted at install time or via the "Nearby devices"
// toggle, and the platform can report it as denied either way, so never block on it.
// Real connectivity problems surface in the scanner checks that follow.
export async function requestLocalNetworkAccess(): Promise<void> {
  if (Platform.OS !== 'android' || Number(Platform.Version) < 36) return;
  const permission = 'android.permission.ACCESS_LOCAL_NETWORK' as AndroidPermission;
  try {
    if (!(await PermissionsAndroid.check(permission))) {
      await PermissionsAndroid.request(permission);
    }
  } catch {
    // Ignored on purpose.
  }
}

export async function requestWifiPermissions(): Promise<void> {
  if (Platform.OS !== 'android') return;
  const permissions: AndroidPermission[] = [PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION];
  if (Number(Platform.Version) >= 33) {
    permissions.push(PermissionsAndroid.PERMISSIONS.NEARBY_WIFI_DEVICES);
  }
  const results = await PermissionsAndroid.requestMultiple(permissions);
  if (permissions.some((permission) => results[permission] !== PermissionsAndroid.RESULTS.GRANTED)) {
    throw new Error('Location / nearby-devices permission is required to join Wi-Fi networks. Allow it in app settings.');
  }
}
