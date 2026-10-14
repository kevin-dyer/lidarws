import React, { useEffect, useMemo, useRef, useState } from 'react';
import { PanResponder, StyleSheet, View } from 'react-native';
import { Canvas, Points, vec } from '@shopify/react-native-skia';
import type { Point3 } from '../scanner/pointCloud';
import { decimate, HEIGHT_BANDS, projectPoints } from '../scanner/projection';

const BAND_COLORS = ['#3B82F6', '#06B6D4', '#10B981', '#84CC16', '#F59E0B', '#EF4444'];
const MAX_DISPLAY_POINTS = 15000;

interface Props {
  points: readonly Point3[];
  size?: number;
  fill?: boolean;
  zoom?: number;
  height?: number;
  interactive?: boolean;
  rounded?: boolean;
  autoRotate?: boolean;
}

export function PointCloudView({ points, size = 260, fill = false, zoom = 1, height, autoRotate = true, interactive = true, rounded = false }: Props) {
  const [box, setBox] = useState({ width: size, height: size });
  const [yaw, setYaw] = useState(0.4);
  const [pitch, setPitch] = useState(0.35);
  const dragging = useRef(false);
  const last = useRef({ x: 0, y: 0 });

  useEffect(() => {
    if (!autoRotate) return undefined;
    const timer = setInterval(() => {
      if (!dragging.current) setYaw((value) => value + 0.03);
    }, 100);
    return () => clearInterval(timer);
  }, [autoRotate]);

  const pan = useMemo(
    () =>
      PanResponder.create({
        onStartShouldSetPanResponder: () => true,
        onPanResponderGrant: (_, g) => {
          dragging.current = true;
          last.current = { x: g.x0, y: g.y0 };
        },
        onPanResponderMove: (_, g) => {
          const dx = g.moveX - last.current.x;
          const dy = g.moveY - last.current.y;
          last.current = { x: g.moveX, y: g.moveY };
          setYaw((v) => v + dx * 0.01);
          setPitch((v) => Math.max(-1.4, Math.min(1.4, v + dy * 0.01)));
        },
        onPanResponderRelease: () => {
          dragging.current = false;
        },
        onPanResponderTerminate: () => {
          dragging.current = false;
        },
      }),
    [],
  );

  const sample = useMemo(() => decimate(points, MAX_DISPLAY_POINTS), [points]);
  const radius = useMemo(() => {
    let max = 1;
    for (const p of sample) max = Math.max(max, Math.hypot(p[0], p[1], p[2]));
    return max;
  }, [sample]);

  const bands = useMemo(() => {
    const projected = projectPoints(sample, yaw, pitch, radius);
    const halfW = box.width / 2;
    const halfH = box.height / 2;
    const fit = Math.min(halfW, halfH) * 0.92 * zoom;
    const groups: ReturnType<typeof vec>[][] = Array.from({ length: HEIGHT_BANDS }, () => []);
    for (const p of projected) groups[p.band].push(vec(halfW + p.x * fit, halfH - p.y * fit));
    return groups;
  }, [sample, yaw, pitch, radius, box, zoom]);

  return (
    <View
      style={[styles.box, rounded && styles.rounded, fill ? styles.fill : { width: size, height: height ?? size }, fill && height ? { height, flex: 0 } : null]}
      onLayout={(event) => {
        const { width, height } = event.nativeEvent.layout;
        if (width > 0 && height > 0) setBox({ width, height });
      }}
      pointerEvents={interactive ? 'auto' : 'none'}
      {...(interactive ? pan.panHandlers : {})}
    >
      <Canvas style={{ width: box.width, height: box.height }}>
        {bands.map((group, index) => (
          <Points
            key={index}
            points={group}
            mode="points"
            color={BAND_COLORS[index]}
            style="stroke"
            strokeWidth={2}
            strokeCap="round"
          />
        ))}
      </Canvas>
    </View>
  );
}

const styles = StyleSheet.create({
  fill: { alignSelf: 'stretch', flex: 1 },
  rounded: { borderRadius: 24 },
  box: { backgroundColor: '#0B1220', overflow: 'hidden' },
});
