declare module 'react-native-udp' {
  interface UdpSocket {
    bind(port: number, address?: string, callback?: () => void): void;
    close(): void;
    on(event: 'message', listener: (message: Uint8Array, info: { address: string; port: number }) => void): this;
    on(event: 'error', listener: (error: Error) => void): this;
    send(
      data: Uint8Array,
      offset: number,
      length: number,
      port: number,
      address: string,
      callback?: (error?: Error | null) => void,
    ): void;
    setBroadcast(enabled: boolean): void;
  }

  const dgram: {
    createSocket(type: 'udp4'): UdpSocket;
  };

  export default dgram;
}
