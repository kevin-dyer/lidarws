const WRAP_US = 0x100000000;
const UART_BYTE_US = 10_000_000 / 115200;

/** Relates the ESP32's wrapping microsecond counter to phone epoch seconds. */
export class BoardClock {
  private elapsedUs = 0;

  constructor(private anchorUs: number, private readonly anchorEpochSeconds: number) {}

  toEpochSeconds(counterUs: number): number {
    const difference = ((counterUs - this.anchorUs + WRAP_US / 2) % WRAP_US + WRAP_US) % WRAP_US - WRAP_US / 2;
    this.anchorUs = counterUs;
    this.elapsedUs += difference;
    return this.anchorEpochSeconds + this.elapsedUs / 1_000_000;
  }

  byteTimes(lastByteUs: number, count: number): number[] {
    const last = this.toEpochSeconds(lastByteUs);
    return Array.from({ length: count }, (_, index) => last - ((count - 1 - index) * UART_BYTE_US) / 1_000_000);
  }
}
