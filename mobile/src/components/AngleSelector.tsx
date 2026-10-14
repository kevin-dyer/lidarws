import { useRef } from 'react';
import { View, useWindowDimensions } from 'react-native';
import Svg, { Circle, Line, Path, Text as SvgText } from 'react-native-svg';

interface AngleSelectorProps {
  startAngle: number;
  stopAngle: number;
  onChange: (startAngle: number, stopAngle: number) => void;
  onDragChange?: (dragging: boolean) => void;
}

type Handle = 'start' | 'stop';

export function AngleSelector({ startAngle, stopAngle, onChange, onDragChange }: AngleSelectorProps) {
  const { width } = useWindowDimensions();
  const width_ = Math.min(width - 72, 330);
  const height = 300;
  const centerX = 28;
  const centerY = height / 2;
  const radius = Math.min(width_ - centerX - 22, (height / 2 - 16) / Math.sin((60 * Math.PI) / 180));
  const activeHandle = useRef<Handle>('start');

  const position = (angle: number, r = radius): [number, number] => [
    centerX + Math.cos((angle * Math.PI) / 180) * r,
    centerY + Math.sin((angle * Math.PI) / 180) * r,
  ];
  const [startX, startY] = position(startAngle);
  const [stopX, stopY] = position(stopAngle);
  const arcStart = position(-60);
  const arcEnd = position(60);
  const arcPath = `M ${arcStart[0]} ${arcStart[1]} A ${radius} ${radius} 0 0 1 ${arcEnd[0]} ${arcEnd[1]}`;

  const angleAt = (x: number, y: number): number =>
    Math.round(Math.max(-45, Math.min(45, (Math.atan2(y - centerY, x - centerX) * 180) / Math.PI)));
  const update = (x: number, y: number): void => {
    const angle = angleAt(x, y);
    if (activeHandle.current === 'start') {
      onChange(Math.min(angle, stopAngle - 1), stopAngle);
    } else {
      onChange(startAngle, Math.max(angle, startAngle + 1));
    }
  };

  return (
    <View
      accessibilityLabel="Drag the angle handles to set the scan start and stop angles"
      onStartShouldSetResponder={() => true}
      onMoveShouldSetResponder={() => true}
      onResponderGrant={(event) => {
        onDragChange?.(true);
        const { locationX: x, locationY: y } = event.nativeEvent;
        const startDistance = Math.hypot(x - startX, y - startY);
        const stopDistance = Math.hypot(x - stopX, y - stopY);
        activeHandle.current = startDistance <= stopDistance ? 'start' : 'stop';
        update(x, y);
      }}
      onResponderMove={(event) => update(event.nativeEvent.locationX, event.nativeEvent.locationY)}
      onResponderRelease={() => onDragChange?.(false)}
      onResponderTerminate={() => onDragChange?.(false)}
      onResponderTerminationRequest={() => false}
    >
      <Svg width={width_} height={height} viewBox={`0 0 ${width_} ${height}`}>
        <Path d={arcPath} fill="none" stroke="#E5EBE8" strokeWidth={2} />
        {[-45, -30, -15, 0, 15, 30, 45].map((angle) => {
          const inner = position(angle, radius - 8);
          const tick = position(angle);
          return (
            <Line
              key={angle}
              x1={inner[0]}
              y1={inner[1]}
              x2={tick[0]}
              y2={tick[1]}
              stroke={angle === 0 ? '#B8C9C0' : '#D8E2DC'}
              strokeWidth={angle % 30 === 0 ? 2 : 1}
            />
          );
        })}
        <Line x1={centerX} y1={centerY} x2={centerX + radius + 6} y2={centerY} stroke="#E5EBE8" strokeWidth={1} strokeDasharray="4 4" />
        <Line x1={centerX} y1={centerY} x2={startX} y2={startY} stroke="#6C9B86" strokeWidth={3} />
        <Line x1={centerX} y1={centerY} x2={stopX} y2={stopY} stroke="#174D43" strokeWidth={3} />
        <Circle cx={centerX} cy={centerY} r={6} fill="#174D43" />
        <Circle cx={startX} cy={startY} r={13} fill="#DDECE4" stroke="#6C9B86" strokeWidth={3} />
        <Circle cx={stopX} cy={stopY} r={13} fill="#174D43" stroke="#FFFFFF" strokeWidth={3} />
        <SvgText x={centerX + radius + 4} y={centerY + 4} fill="#7B8A82" fontSize="11" textAnchor="end" dx={-18}>0°</SvgText>
        <SvgText x={position(-45, radius * 0.72)[0]} y={position(-45, radius * 0.72)[1] - 6} fill="#7B8A82" fontSize="11">−45°</SvgText>
        <SvgText x={position(45, radius * 0.72)[0]} y={position(45, radius * 0.72)[1] + 16} fill="#7B8A82" fontSize="11">+45°</SvgText>
      </Svg>
    </View>
  );
}
