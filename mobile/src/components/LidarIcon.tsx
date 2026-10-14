import Svg, { Circle, Path } from 'react-native-svg';

export function LidarIcon({ color = '#FFFFFF', size = 24 }: { color?: string; size?: number }) {
  const arc = (r: number, side: 1 | -1) => {
    const dx = r * 0.8;
    const dy = r * 0.6;
    return `M ${12 + side * dx} ${12 - dy} A ${r} ${r} 0 0 ${side === 1 ? 1 : 0} ${12 + side * dx} ${12 + dy}`;
  };
  return (
    <Svg width={size} height={size} viewBox="0 0 24 24" fill="none">
      <Circle cx="12" cy="12" r="2.6" stroke={color} strokeWidth={1.8} />
      {[5.5, 8.5, 11.5].flatMap((r) => [1, -1].map((side) => (
        <Path key={`${r}${side}`} d={arc(r, side as 1 | -1)} stroke={color} strokeWidth={1.8} strokeLinecap="round" />
      )))}
    </Svg>
  );
}
