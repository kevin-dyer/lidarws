import { useEffect, useState } from 'react';
import { StyleSheet, TextInput, View } from 'react-native';
import { Pencil } from 'lucide-react-native';
import { PALETTE } from '../theme';

interface Props {
  value: string;
  placeholder?: string;
  onChangeText?: (value: string) => void;
  onCommit?: (value: string) => void;
}

export function NameField({ value, placeholder = 'Name this scan', onChangeText, onCommit }: Props) {
  const [text, setText] = useState(value);
  useEffect(() => setText(value), [value]);
  return (
    <View style={styles.row}>
      <TextInput
        value={text}
        placeholder={placeholder}
        placeholderTextColor="#98A59E"
        maxLength={60}
        returnKeyType="done"
        style={styles.input}
        onChangeText={(next) => {
          setText(next);
          onChangeText?.(next);
        }}
        onEndEditing={() => onCommit?.(text)}
      />
      <Pencil size={17} color={PALETTE.muted} />
    </View>
  );
}

const styles = StyleSheet.create({
  row: { flexDirection: 'row', alignItems: 'center', gap: 10, backgroundColor: PALETTE.white, borderRadius: 16, borderWidth: 1, borderColor: '#E9EFEB', paddingHorizontal: 16, marginTop: 14 },
  input: { flex: 1, height: 50, color: PALETTE.ink, fontSize: 16, fontWeight: '600' },
});
