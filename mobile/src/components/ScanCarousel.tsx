import { useEffect, useRef, useState } from 'react';
import { Animated, FlatList, Image, Pressable, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import { PALETTE } from '../theme';
import { loadScanPoints, type SavedScan } from '../storage/scans';
import type { Point3 } from '../scanner/pointCloud';
import { PointCloudView } from './PointCloudView';

const CARD_HEIGHT = 250;
const FADE_INTERVAL_MS = 4000;
const ADVANCE_INTERVAL_MS = 6000;

function formatDate(value: string): string {
  return new Date(value).toLocaleDateString(undefined, { month: 'short', day: 'numeric', year: 'numeric' });
}

function ScanCard({ scan, width, onPress }: { scan: SavedScan; width: number; onPress: () => void }) {
  const [points, setPoints] = useState<Point3[]>([]);
  const photoOpacity = useRef(new Animated.Value(0)).current;

  useEffect(() => {
    let active = true;
    void loadScanPoints(scan, 8000).then((loaded) => {
      if (active) setPoints(loaded);
    }).catch(() => {});
    return () => {
      active = false;
    };
  }, [scan]);

  useEffect(() => {
    if (!scan.photoUri) return undefined;
    let showPhoto = false;
    const timer = setInterval(() => {
      showPhoto = !showPhoto;
      Animated.timing(photoOpacity, { toValue: showPhoto ? 1 : 0, duration: 900, useNativeDriver: true }).start();
    }, FADE_INTERVAL_MS);
    return () => clearInterval(timer);
  }, [scan.photoUri, photoOpacity]);

  return (
    <Pressable accessibilityRole="button" onPress={onPress} style={[styles.card, { width }]}>
      <PointCloudView points={points} fill height={CARD_HEIGHT} zoom={1.6} interactive={false} rounded />
      {scan.photoUri ? (
        <Animated.View pointerEvents="none" style={[StyleSheet.absoluteFill, { opacity: photoOpacity }]}>
          <Image source={{ uri: scan.photoUri }} style={styles.photo} />
        </Animated.View>
      ) : null}
      <View pointerEvents="none" style={styles.caption}>
        <Text style={styles.date}>{formatDate(scan.createdAt)}</Text>
        <Text style={styles.points}>{scan.pointCount.toLocaleString()} points</Text>
      </View>
    </Pressable>
  );
}

export function ScanCarousel({ scans, onOpen }: { scans: SavedScan[]; onOpen: (scan: SavedScan) => void }) {
  const { width } = useWindowDimensions();
  const cardWidth = width - 48;
  const [index, setIndex] = useState(0);
  const listRef = useRef<FlatList<SavedScan>>(null);
  const indexRef = useRef(0);
  const touching = useRef(false);
  indexRef.current = index;

  useEffect(() => {
    if (scans.length < 2) return undefined;
    const timer = setInterval(() => {
      if (touching.current) return;
      const next = (indexRef.current + 1) % scans.length;
      listRef.current?.scrollToOffset({ offset: next * (cardWidth + 12), animated: true });
      setIndex(next);
    }, ADVANCE_INTERVAL_MS);
    return () => clearInterval(timer);
  }, [scans.length, cardWidth]);
  return (
    <View style={styles.wrap}>
      <FlatList
        ref={listRef}
        onScrollBeginDrag={() => { touching.current = true; }}
        horizontal
        pagingEnabled={false}
        snapToInterval={cardWidth + 12}
        decelerationRate="fast"
        showsHorizontalScrollIndicator={false}
        data={scans}
        keyExtractor={(item) => item.id}
        contentContainerStyle={styles.list}
        onMomentumScrollEnd={(event) => { touching.current = false; setIndex(Math.round(event.nativeEvent.contentOffset.x / (cardWidth + 12))); }}
        renderItem={({ item }) => <ScanCard scan={item} width={cardWidth} onPress={() => onOpen(item)} />}
      />
      {scans.length > 1 && (
        <View style={styles.dots}>
          {scans.map((scan, position) => (
            <View key={scan.id} style={[styles.dot, position === index && styles.dotActive]} />
          ))}
        </View>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  wrap: { marginHorizontal: -24, paddingTop: 22, paddingBottom: 18 },
  list: { paddingHorizontal: 24, gap: 12 },
  card: { height: CARD_HEIGHT, borderRadius: 24, overflow: 'hidden', backgroundColor: '#0B1220' },
  photo: { width: '100%', height: '100%', resizeMode: 'cover' },
  caption: { position: 'absolute', left: 16, bottom: 14 },
  date: { color: '#FFFFFF', fontSize: 15, fontWeight: '700' },
  points: { color: 'rgba(255,255,255,0.75)', fontSize: 12, marginTop: 2 },
  dots: { flexDirection: 'row', justifyContent: 'center', gap: 6, marginTop: 12 },
  dot: { width: 6, height: 6, borderRadius: 3, backgroundColor: '#C9D5CE' },
  dotActive: { width: 18, backgroundColor: PALETTE.forest },
});
