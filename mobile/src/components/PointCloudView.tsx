import React, { useEffect, useMemo, useRef, useState } from 'react';
import { Modal, PanResponder, Pressable, StyleSheet, View, useWindowDimensions } from 'react-native';
import type { GestureResponderEvent } from 'react-native';
import { Maximize2, Minimize2 } from 'lucide-react-native';
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
  fullscreenButton?: boolean;
}

// Viewed from the scanner's own position, looking out at the scene (yaw 0 looks back at the scanner).
const HEAD_ON_YAW = Math.PI;
const HEAD_ON_PITCH = 0.12;
const MIN_SCALE = 0.4;
const MAX_SCALE = 12;

const touchesOf = (event: GestureResponderEvent) => {
  const touches = event.nativeEvent.touches;
  return touches.length >= 2
    ? { count: 2, cx: (touches[0].pageX + touches[1].pageX) / 2, cy: (touches[0].pageY + touches[1].pageY) / 2,
        dist: Math.hypot(touches[0].pageX - touches[1].pageX, touches[0].pageY - touches[1].pageY) }
    : { count: touches.length, cx: 0, cy: 0, dist: 0 };
};

export function PointCloudView(props: Props) {
  const [full, setFull] = useState(false);
  const { width: winW, height: winH } = useWindowDimensions();
  const showButton = props.fullscreenButton ?? (props.interactive ?? true);
  return (
    <>
      <CloudCanvas {...props} onFullscreen={showButton ? () => setFull(true) : undefined} />
      <Modal visible={full} animationType="fade" onRequestClose={() => setFull(false)} statusBarTranslucent>
        <View style={styles.fullBackdrop}>
          <View
            style={[
              styles.fullRotated,
              { width: winH, height: winW, left: (winW - winH) / 2, top: (winH - winW) / 2 },
            ]}
          >
            <CloudCanvas
              points={props.points}
              fill
              zoom={props.zoom}
              autoRotate={false}
              rotated
              onClose={() => setFull(false)}
            />
          </View>
        </View>
      </Modal>
    </>
  );
}

interface CanvasProps extends Props {
  rotated?: boolean;
  onFullscreen?: () => void;
  onClose?: () => void;
}

function CloudCanvas({
  points, size = 260, fill = false, zoom = 1, height, autoRotate = false, interactive = true,
  rounded = false, rotated = false, onFullscreen, onClose,
}: CanvasProps) {
  const [box, setBox] = useState({ width: size, height: size });
  const [yaw, setYaw] = useState(autoRotate ? 0.4 : HEAD_ON_YAW);
  const [pitch, setPitch] = useState(autoRotate ? 0.35 : HEAD_ON_PITCH);
  const [scale, setScale] = useState(1);
  const [offset, setOffset] = useState({ x: 0, y: 0 });
  const dragging = useRef(false);
  const last = useRef({ x: 0, y: 0 });
  const pinch = useRef<{ dist: number; cx: number; cy: number } | null>(null);

  useEffect(() => {
    if (!autoRotate) return undefined;
    const timer = setInterval(() => {
      if (!dragging.current) setYaw((value) => value + 0.03);
    }, 100);
    return () => clearInterval(timer);
  }, [autoRotate]);

  // The fullscreen view is the page turned 90° clockwise, so screen motion maps to view motion.
  const toView = (dx: number, dy: number) => (rotated ? { x: dy, y: -dx } : { x: dx, y: dy });

  const pan = useMemo(
    () =>
      PanResponder.create({
        onStartShouldSetPanResponder: () => true,
        onPanResponderTerminationRequest: () => false,
        onPanResponderGrant: (event, g) => {
          dragging.current = true;
          last.current = { x: g.x0, y: g.y0 };
          pinch.current = null;
        },
        onPanResponderMove: (event, g) => {
          const touch = touchesOf(event);
          if (touch.count >= 2) {
            const previous = pinch.current;
            pinch.current = { dist: touch.dist, cx: touch.cx, cy: touch.cy };
            if (previous && previous.dist > 0) {
              const ratio = touch.dist / previous.dist;
              setScale((value) => Math.max(MIN_SCALE, Math.min(MAX_SCALE, value * ratio)));
              const move = toView(touch.cx - previous.cx, touch.cy - previous.cy);
              setOffset((value) => ({ x: value.x + move.x, y: value.y + move.y }));
            }
            last.current = { x: g.moveX, y: g.moveY };
            return;
          }
          if (pinch.current) {
            // Back to one finger: restart the orbit from the current position.
            pinch.current = null;
            last.current = { x: g.moveX, y: g.moveY };
            return;
          }
          const move = toView(g.moveX - last.current.x, g.moveY - last.current.y);
          last.current = { x: g.moveX, y: g.moveY };
          setYaw((v) => v + move.x * 0.01);
          setPitch((v) => Math.max(-1.4, Math.min(1.4, v + move.y * 0.01)));
        },
        onPanResponderRelease: () => {
          dragging.current = false;
          pinch.current = null;
        },
        onPanResponderTerminate: () => {
          dragging.current = false;
          pinch.current = null;
        },
      }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [rotated],
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
    const fit = Math.min(halfW, halfH) * 0.92 * zoom * scale;
    const groups: ReturnType<typeof vec>[][] = Array.from({ length: HEIGHT_BANDS }, () => []);
    for (const p of projected) groups[p.band].push(vec(halfW + offset.x + p.x * fit, halfH + offset.y - p.y * fit));
    return groups;
  }, [sample, yaw, pitch, radius, box, zoom, scale, offset]);

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
      {(onFullscreen || onClose) && (
        <Pressable
          accessibilityRole="button"
          accessibilityLabel={onClose ? 'Exit full screen' : 'Full screen'}
          onPress={onClose ?? onFullscreen}
          style={[styles.fullButton, onClose && styles.closeButton]}
          hitSlop={12}
        >
          {onClose ? <Minimize2 size={22} color="#FFFFFF" /> : <Maximize2 size={18} color="#FFFFFF" />}
        </Pressable>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  fill: { alignSelf: 'stretch', flex: 1 },
  rounded: { borderRadius: 24 },
  fullButton: { position: 'absolute', top: 12, right: 12, width: 38, height: 38, borderRadius: 19, backgroundColor: 'rgba(255,255,255,0.18)', alignItems: 'center', justifyContent: 'center' },
  closeButton: { top: 16, right: undefined, left: 64, width: 48, height: 48, borderRadius: 24, backgroundColor: 'rgba(255,255,255,0.3)' },
  fullBackdrop: { flex: 1, backgroundColor: '#0B1220' },
  fullRotated: { position: 'absolute', transform: [{ rotate: '90deg' }] },
  box: { backgroundColor: '#0B1220', overflow: 'hidden' },
});
