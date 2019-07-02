class ClockSocket extends EventEmitter2 {
  constructor(addr) {
    super();
    this.addr = addr;
  }

  open() {
    this.socket = new WebSocket(this.addr);

    this.socket.onopen = (ev) => {
      this.emit('open', ev);
    };

    this.socket.onerror = (ev) => {
      this.emit('error', ev);
    };

    this.socket.onclose = (ev) => {
      this.emit('close', ev);
    };

    this.socket.onmessage = (ev) => {
      let t = ev.timeStamp;
      let data = Number(ev.data);
      this.emit('incoming', data, t);
    };
  }

  send(data) {
    if (this.socket.readyState != WebSocket.OPEN) {
      return;
    }
    let msg = data.toString();
    this.socket.send(msg);
  }
}
