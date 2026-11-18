import AsyncStorage from '@react-native-async-storage/async-storage';
import JSZip from 'jszip';
import * as FileSystem from 'expo-file-system/legacy';
import type { Point3 } from '../scanner/pointCloud';
import { encodeAsciiPly, parseAsciiPly } from '../scanner/pointCloud';

const SCANS_KEY = 'tilt-lidar-scans-v1';

export interface SavedScan {
  id: string;
  name: string;
  fileUri: string;
  photoUri?: string;
  createdAt: string;
  pointCount: number;
  durationSeconds: number;
  startAngle: number;
  stopAngle: number;
  groupId?: string;
  /** Present for hand-held mapping scans: the recorded forward video and the estimated sensor trajectory. */
  videoUri?: string;
  trajectoryUri?: string;
}

/** Extra files saved next to a mobile-mapping point cloud. */
export interface MappingAssets {
  videoSourceUri?: string;
  trajectoryJson: string;
}

async function readScans(): Promise<SavedScan[]> {
  const stored = await AsyncStorage.getItem(SCANS_KEY);
  return stored ? (JSON.parse(stored) as SavedScan[]) : [];
}

async function writeScans(scans: SavedScan[]): Promise<void> {
  await AsyncStorage.setItem(SCANS_KEY, JSON.stringify(scans));
}

export async function listScans(): Promise<SavedScan[]> {
  return (await readScans()).sort((left, right) =>
    right.createdAt.localeCompare(left.createdAt),
  );
}

export async function saveScan(
  points: Point3[],
  settings: { durationSeconds: number; startAngle: number; stopAngle: number },
  photoSourceUri?: string,
  options: { name?: string; groupId?: string; anchorId?: string; mapping?: MappingAssets } = {},
): Promise<SavedScan> {
  const directory = FileSystem.documentDirectory;
  if (!directory) {
    throw new Error('The app document directory is not available.');
  }
  const id = `tilt_scan_${new Date().toISOString().replace(/[-:.TZ]/g, '')}`;
  const fileUri = `${directory}${id}_raw_cloud.ply`;
  const photoUri = photoSourceUri ? `${directory}${id}_photo.jpg` : undefined;
  await FileSystem.writeAsStringAsync(fileUri, encodeAsciiPly(points), {
    encoding: FileSystem.EncodingType.UTF8,
  });
  if (photoSourceUri && photoUri) {
    await FileSystem.copyAsync({ from: photoSourceUri, to: photoUri });
  }

  const videoUri = options.mapping?.videoSourceUri ? `${directory}${id}_video.mp4` : undefined;
  const trajectoryUri = options.mapping ? `${directory}${id}_trajectory.json` : undefined;
  try {
    if (options.mapping && trajectoryUri) {
      await FileSystem.writeAsStringAsync(trajectoryUri, options.mapping.trajectoryJson, {
        encoding: FileSystem.EncodingType.UTF8,
      });
      if (videoUri && options.mapping.videoSourceUri) {
        await FileSystem.copyAsync({ from: options.mapping.videoSourceUri, to: videoUri });
      }
    }
  } catch (error) {
    await Promise.all(
      [fileUri, photoUri, videoUri, trajectoryUri].map((uri) =>
        uri ? FileSystem.deleteAsync(uri, { idempotent: true }) : Promise.resolve(),
      ),
    );
    throw error;
  }

  const scan: SavedScan = {
    id,
    name: options.name?.trim() || new Date().toLocaleString(),
    fileUri,
    photoUri,
    createdAt: new Date().toISOString(),
    pointCount: points.length,
    durationSeconds: settings.durationSeconds,
    startAngle: settings.startAngle,
    stopAngle: settings.stopAngle,
    videoUri,
    trajectoryUri,
  };
  try {
    const existing = await readScans();
    if (options.groupId) {
      scan.groupId = options.groupId;
      const anchor = options.anchorId ? existing.find((saved) => saved.id === options.anchorId) : undefined;
      if (anchor && !anchor.groupId) anchor.groupId = options.groupId;
    }
    await writeScans([scan, ...existing]);
  } catch (error) {
    await FileSystem.deleteAsync(fileUri, { idempotent: true });
    for (const uri of [photoUri, videoUri, trajectoryUri]) {
      if (uri) await FileSystem.deleteAsync(uri, { idempotent: true });
    }
    throw error;
  }
  return scan;
}

export async function renameScan(id: string, name: string): Promise<void> {
  const trimmed = name.trim();
  if (!trimmed) return;
  await writeScans((await readScans()).map((saved) => (saved.id === id ? { ...saved, name: trimmed } : saved)));
}

/** Sets (or, with undefined, clears) the group of every listed scan. */
export async function setScansGroup(ids: string[], groupId: string | undefined): Promise<void> {
  await writeScans(
    (await readScans()).map((saved) => {
      if (!ids.includes(saved.id)) return saved;
      const { groupId: _old, ...rest } = saved;
      return groupId ? { ...rest, groupId } : rest;
    }),
  );
}

/** Deletes a scan's photo file and forgets it. */
export async function removeScanPhoto(scan: SavedScan): Promise<void> {
  if (scan.photoUri) await FileSystem.deleteAsync(scan.photoUri, { idempotent: true });
  await writeScans(
    (await readScans()).map((saved) => {
      if (saved.id !== scan.id) return saved;
      const { photoUri: _photo, ...rest } = saved;
      return rest;
    }),
  );
}

export async function deleteScan(scan: SavedScan): Promise<void> {
  await FileSystem.deleteAsync(scan.fileUri, { idempotent: true });
  for (const uri of [scan.photoUri, scan.videoUri, scan.trajectoryUri]) {
    if (uri) await FileSystem.deleteAsync(uri, { idempotent: true });
  }
  await writeScans((await readScans()).filter((saved) => saved.id !== scan.id));
}

const pointCache = new Map<string, Point3[]>();

/** Loads a saved scan's points, decimated to at most `max` for on-screen display. */
export async function loadScanPoints(scan: SavedScan, max = 20000): Promise<Point3[]> {
  const key = `${scan.id}:${max}`;
  const cached = pointCache.get(key);
  if (cached) return cached;
  const text = await FileSystem.readAsStringAsync(scan.fileUri, { encoding: FileSystem.EncodingType.UTF8 });
  const points = parseAsciiPly(text, max);
  pointCache.set(key, points);
  return points;
}

const safeName = (value: string): string => value.replace(/[^\w.-]+/g, '_').replace(/^_+|_+$/g, '') || 'scan';

/** Files (point cloud plus photo, if any) to hand to the email composer. */
export function shareableFiles(scans: SavedScan[]): string[] {
  return scans.flatMap((scan) => (scan.photoUri ? [scan.fileUri, scan.photoUri] : [scan.fileUri]));
}

/**
 * Returns a single file to share. A lone scan without a photo is shared as-is; anything else is
 * bundled into a zip because the system share sheet takes one file at a time.
 */
export async function buildShareFile(scans: SavedScan[]): Promise<string> {
  if (scans.length === 1 && !scans[0].photoUri) return scans[0].fileUri;
  const zip = new JSZip();
  const used = new Set<string>();
  for (const scan of scans) {
    let base = safeName(scan.name);
    while (used.has(base)) base = `${base}_${scan.id.slice(-4)}`;
    used.add(base);
    const folder = scans.length > 1 ? (zip.folder(base) as JSZip) : zip;
    const ply = await FileSystem.readAsStringAsync(scan.fileUri);
    folder.file(`${base}.ply`, ply);
    if (scan.photoUri) {
      const photo = await FileSystem.readAsStringAsync(scan.photoUri, { encoding: FileSystem.EncodingType.Base64 });
      folder.file(`${base}.jpg`, photo, { base64: true });
    }
  }
  const data = await zip.generateAsync({ type: 'base64' });
  const target = `${FileSystem.cacheDirectory}${scans.length > 1 ? 'scans' : safeName(scans[0].name)}.zip`;
  await FileSystem.writeAsStringAsync(target, data, { encoding: FileSystem.EncodingType.Base64 });
  return target;
}
