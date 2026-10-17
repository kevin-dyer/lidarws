import { Modal, Pressable, ScrollView, StyleSheet, Text, View } from 'react-native';
import { Check, FolderPlus, Layers, X } from 'lucide-react-native';
import { PALETTE } from '../theme';

export interface GroupOption {
  id: string;
  title: string;
  count: number;
}

export type GroupChoice = { type: 'none' } | { type: 'new' } | { type: 'existing'; id: string };

interface Props {
  visible: boolean;
  title?: string;
  groups: GroupOption[];
  /** Currently chosen group id; undefined means no group. */
  current?: string;
  onPick: (choice: GroupChoice) => void;
  onClose: () => void;
}

export function GroupPicker({ visible, title = 'Choose a group', groups, current, onPick, onClose }: Props) {
  const choose = (choice: GroupChoice): void => {
    onPick(choice);
    onClose();
  };
  return (
    <Modal visible={visible} transparent animationType="fade" onRequestClose={onClose}>
      <Pressable accessibilityLabel="Close" style={styles.backdrop} onPress={onClose}>
        <Pressable style={styles.sheet} onPress={() => {}}>
          <View style={styles.header}>
            <Text style={styles.title}>{title}</Text>
            <Pressable accessibilityRole="button" accessibilityLabel="Close" onPress={onClose} hitSlop={10}>
              <X size={20} color={PALETTE.muted} />
            </Pressable>
          </View>
          <ScrollView style={styles.list}>
            <Pressable accessibilityRole="button" style={styles.row} onPress={() => choose({ type: 'new' })}>
              <FolderPlus size={19} color={PALETTE.forest} />
              <Text style={styles.rowText}>Start a new group</Text>
            </Pressable>
            <Pressable accessibilityRole="button" style={styles.row} onPress={() => choose({ type: 'none' })}>
              <View style={styles.rowIcon} />
              <Text style={styles.rowText}>No group</Text>
              {current === undefined && <Check size={18} color={PALETTE.forest} />}
            </Pressable>
            {groups.map((group) => (
              <Pressable
                key={group.id}
                accessibilityRole="button"
                style={styles.row}
                onPress={() => choose({ type: 'existing', id: group.id })}
              >
                <Layers size={19} color={PALETTE.forest} />
                <View style={styles.flex}>
                  <Text style={styles.rowText} numberOfLines={1}>{group.title}</Text>
                  <Text style={styles.rowSub}>{group.count} scan{group.count === 1 ? '' : 's'}</Text>
                </View>
                {current === group.id && <Check size={18} color={PALETTE.forest} />}
              </Pressable>
            ))}
          </ScrollView>
        </Pressable>
      </Pressable>
    </Modal>
  );
}

const styles = StyleSheet.create({
  backdrop: { flex: 1, backgroundColor: 'rgba(23,51,44,0.45)', justifyContent: 'center', padding: 24 },
  sheet: { backgroundColor: PALETTE.white, borderRadius: 22, paddingVertical: 8, maxHeight: '70%' },
  header: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center', paddingHorizontal: 20, paddingVertical: 14 },
  title: { color: PALETTE.ink, fontSize: 17, fontWeight: '700' },
  list: { flexGrow: 0 },
  row: { flexDirection: 'row', alignItems: 'center', gap: 14, paddingHorizontal: 20, paddingVertical: 14 },
  rowIcon: { width: 19 },
  rowText: { flex: 1, color: PALETTE.ink, fontSize: 15, fontWeight: '600' },
  rowSub: { color: PALETTE.muted, fontSize: 11, marginTop: 2 },
  flex: { flex: 1 },
});
