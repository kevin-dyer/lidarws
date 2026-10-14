import { useRef, useState } from 'react';
import { StyleSheet, Text, View } from 'react-native';

interface Props {
  value: number;
  displayValue?: number;
  min: number;
  max: number;
  onChange: (value: number) => void;
  onDragChange?: (dragging: boolean) => void;
  color: string;
}

const THUMB = 26;

export function DurationSlider({ value, displayValue, min, max, onChange, onDragChange, color }: Props) {
  const [width, setWidth] = useState(0);
  const [active, setActive] = useState(false);
  const widthRef = useRef(0);
  const span = Math.max(1, width - THUMB);
  const fraction = (value - min) / (max - min);
  const x = fraction * span;

  const update = (locationX: number): void => {
    const usable = Math.max(1, widthRef.current - THUMB);
    const f = Math.max(0, Math.min(1, (locationX - THUMB / 2) / usable));
    onChange(Math.round(min + f * (max - min)));
  };
  const setDrag = (dragging: boolean): void => {
    setActive(dragging);
    onDragChange?.(dragging);
  };

  return (
    <View
      accessibilityRole="adjustable"
      accessibilityLabel="Scan duration"
      style={styles.wrap}
      onLayout={(event) => {
        widthRef.current = event.nativeEvent.layout.width;
        setWidth(event.nativeEvent.layout.width);
      }}
      onStartShouldSetResponder={() => true}
      onMoveShouldSetResponder={() => true}
      onResponderGrant={(event) => {
        setDrag(true);
        update(event.nativeEvent.locationX);
      }}
      onResponderMove={(event) => update(event.nativeEvent.locationX)}
      onResponderRelease={() => setDrag(false)}
      onResponderTerminate={() => setDrag(false)}
      onResponderTerminationRequest={() => false}
    >
      <View pointerEvents="none" style={styles.track} />
      <View pointerEvents="none" style={[styles.fill, { width: x + THUMB / 2, backgroundColor: color }]} />
      <View pointerEvents="none" style={[styles.thumb, { left: x, backgroundColor: color }]} />
      {active && (
        <View pointerEvents="none" style={[styles.tooltip, { left: x + THUMB / 2 - 28, backgroundColor: color }]}>
          <Text style={styles.tooltipText}>{displayValue ?? value}s</Text>
        </View>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  wrap: { height: 72, width: '100%', alignSelf: 'stretch', marginTop: 4 },
  track: { position: 'absolute', left: 0, right: 0, top: 40, height: 3, borderRadius: 2, backgroundColor: '#DFE7E2' },
  fill: { position: 'absolute', left: 0, top: 40, height: 3, borderRadius: 2 },
  thumb: { position: 'absolute', top: 29, width: THUMB, height: THUMB, borderRadius: THUMB / 2, borderWidth: 3, borderColor: '#FFFFFF', elevation: 3 },
  tooltip: { position: 'absolute', top: 0, width: 56, height: 30, borderRadius: 10, alignItems: 'center', justifyContent: 'center' },
  tooltipText: { color: '#FFFFFF', fontSize: 14, fontWeight: '700' },
});
