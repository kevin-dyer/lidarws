const REORDER_WAIT_MS = 100;
const MAX_REORDER_PACKETS = 32;

/** Reorders a bounded burst without ever passing an incomplete byte stream to the LiDAR parser. */
export class TimedPacketOrder<T> {
  private expected: number | null = null;
  private readonly pending = new Map<number, { packet: T }>();
  private timer: ReturnType<typeof setTimeout> | undefined;
  private closed = false;

  constructor(
    private readonly deliver: (packet: T) => void,
    private readonly onError: (error: Error) => void,
  ) {}

  push(sequence: number, packet: T): void {
    if (this.closed) return;
    if (this.expected === null) this.expected = sequence;
    const distance = (sequence - this.expected) >>> 0;
    if (distance >= 0x80000000) return;
    if (distance >= MAX_REORDER_PACKETS) {
      this.fail(sequence);
      return;
    }
    if (!this.pending.has(sequence)) this.pending.set(sequence, { packet });
    let advanced = false;
    while (true) {
      const current = this.pending.get(this.expected);
      if (!current) break;
      this.pending.delete(this.expected);
      this.expected = (this.expected + 1) >>> 0;
      advanced = true;
      this.deliver(current.packet);
      if (this.closed) return;
    }
    if (advanced && this.timer) {
      clearTimeout(this.timer);
      this.timer = undefined;
    }
    if (this.pending.size > 0 && !this.timer) {
      this.timer = setTimeout(() => {
        const first = this.pending.keys().next();
        if (!first.done) this.fail(first.value);
      }, REORDER_WAIT_MS);
    }
  }

  private fail(received: number): void {
    const expected = this.expected;
    this.close();
    this.onError(new Error(
      `Timed LiDAR packet ${expected} was missing (received ${received}). Check the scanner Wi-Fi connection.`,
    ));
  }

  close(): void {
    this.closed = true;
    if (this.timer) clearTimeout(this.timer);
    this.timer = undefined;
    this.pending.clear();
  }
}
