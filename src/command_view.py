"""Manual command console for the active acquisition connection."""
from datetime import datetime, timezone, timedelta
import tkinter as tk
from tkinter import ttk, filedialog
import weakref

from command_transport import encode_command, ENDINGS
from network_source import NetworkWorker, validate_endpoint


class CommandView(object):
    def __init__(self, parent, app):
        self.app, self.parent = weakref.proxy(app), parent
        self._worker = None
        self.history = []
        self.mode_var = tk.StringVar(master=parent, value='文本 ASCII')
        self.ending_var = tk.StringVar(master=parent, value='CRLF')
        self.target_var = tk.StringVar(master=parent, value='peer')
        self.target_host_var = tk.StringVar(master=parent, value='127.0.0.1')
        self.target_port_var = tk.StringVar(master=parent, value='3001')
        self.rx_var = tk.StringVar(master=parent, value='文本应答（过滤 IMU）')
        self.status_var = tk.StringVar(master=parent, value='连接串口 / 网口后手动发送命令')
        self.connection_var = tk.StringVar(master=parent)
        header = tk.Frame(parent, bg='white', padx=12, pady=8)
        header.pack(fill='x')
        tk.Label(header, text='命令发送 · 设备应答', bg='white', fg=app.INK,
                 font=('Microsoft YaHei', 12, 'bold')).pack(anchor='w')
        tk.Label(header, textvariable=self.connection_var, bg='white', fg=app.MUTED,
                 anchor='w', font=('Consolas', 10)).pack(fill='x', pady=(5, 0))
        tk.Label(header, text='复用当前连接；文本按所选结束符发送，HEX 不追加结束符。写入成功后请查看设备应答。',
                 bg='white', fg=app.MUTED, anchor='w', font=('Microsoft YaHei', 9)).pack(fill='x', pady=(4, 0))

        self.udp_row = tk.Frame(parent, bg='white', padx=12, pady=6)
        tk.Label(self.udp_row, text='UDP 目标', bg='white', fg=app.INK).pack(side='left')
        for caption, value in (('跟随已锁定发送端', 'peer'), ('指定目标', 'custom')):
            tk.Radiobutton(self.udp_row, text=caption, variable=self.target_var, value=value,
                           command=self.refresh, bg='white').pack(side='left', padx=4)
        self.target_host = ttk.Entry(self.udp_row, textvariable=self.target_host_var, width=15)
        self.target_host.pack(side='left', padx=4)
        tk.Label(self.udp_row, text='端口', bg='white').pack(side='left')
        self.target_port = ttk.Entry(self.udp_row, textvariable=self.target_port_var, width=6)
        self.target_port.pack(side='left', padx=4)

        self.input_frame = tk.Frame(parent, bg='white', padx=12, pady=8)
        self.input_frame.pack(fill='x', pady=4)
        controls = tk.Frame(self.input_frame, bg='white')
        controls.pack(fill='x', pady=(0, 6))
        tk.Label(controls, text='命令格式', bg='white', fg=app.INK).pack(side='left', padx=(0, 6))
        mode = ttk.Combobox(controls, textvariable=self.mode_var, values=('文本 ASCII', 'HEX 字节'),
                            state='readonly', width=12)
        mode.pack(side='left')
        mode.bind('<<ComboboxSelected>>', lambda e: self.refresh())
        tk.Label(controls, text='结束符', bg='white', fg=app.INK).pack(side='left', padx=(12, 6))
        self.ending_box = ttk.Combobox(controls, textvariable=self.ending_var, values=tuple(ENDINGS),
                                      state='readonly', width=7)
        self.ending_box.pack(side='left')
        self.send_button = ttk.Button(controls, text='发送命令', style='Accent.TButton', command=self.send)
        self.send_button.pack(side='right')
        tk.Label(controls, text='Ctrl + Enter', bg='white', fg=app.MUTED).pack(side='right', padx=8)
        self.input = tk.Text(self.input_frame, height=4, font=('Consolas', 11), undo=True,
                             relief='solid', borderwidth=1, wrap='word')
        self.input.pack(fill='x')
        self.input.bind('<Control-Return>', lambda e: (self.send(), 'break')[1])
        historybar = tk.Frame(self.input_frame, bg='white')
        historybar.pack(fill='x', pady=(6, 0))
        tk.Label(historybar, text='最近命令', bg='white', fg=app.INK).pack(side='left', padx=(0, 6))
        self.history_box = ttk.Combobox(historybar, state='readonly', width=45)
        self.history_box.pack(side='left', fill='x', expand=True)
        self.history_box.bind('<<ComboboxSelected>>', lambda e: self.load_history())
        ttk.Button(historybar, text='清空输入', command=lambda: self.input.delete('1.0', 'end')).pack(side='left', padx=6)
        tk.Label(parent, textvariable=self.status_var, anchor='w', bg=app.BG, fg=app.INK,
                 font=('Microsoft YaHei', 9), padx=12, pady=5).pack(fill='x')

        logbar = tk.Frame(parent, bg='white', padx=12, pady=6)
        logbar.pack(fill='x')
        tk.Label(logbar, text='发送 / 接收记录', bg='white', fg=app.INK).pack(side='left')
        rx = ttk.Combobox(logbar, textvariable=self.rx_var, state='readonly', width=24,
                          values=('文本应答（过滤 IMU）', 'HEX 预览（每包最多256B）', '暂停接收显示'))
        rx.pack(side='left', padx=12)
        rx.bind('<<ComboboxSelected>>', lambda e: self.refresh())
        ttk.Button(logbar, text='保存日志…', command=self.save_log).pack(side='right')
        ttk.Button(logbar, text='清空记录', command=self.clear_log).pack(side='right', padx=6)
        logframe = tk.Frame(parent, bg=app.BG)
        logframe.pack(fill='both', expand=True, padx=6, pady=(0, 6))
        self.log = tk.Text(logframe, state='disabled', bg='#14263f', fg='#d6e3f5',
                           font=('Consolas', 10), relief='flat', wrap='word', height=8)
        scrollbar = ttk.Scrollbar(logframe, command=self.log.yview)
        scrollbar.pack(side='right', fill='y')
        self.log.configure(yscrollcommand=scrollbar.set)
        self.log.pack(side='left', fill='both', expand=True)
        for name, color in (('tx', '#78d5ad'), ('rx', '#a6cdfb'), ('rx_hex', '#a6cdfb'), ('error', '#ff9f9f')):
            self.log.tag_configure(name, foreground=color)
        self.refresh()

    def _target(self, worker):
        if not isinstance(worker, NetworkWorker) or worker.mode != 'udp':
            return None
        if self.target_var.get() == 'peer':
            if worker.peer is None:
                raise ValueError('尚无已锁定发送端，请先接收数据或指定 UDP 目标')
            return worker.peer
        _, host, port = validate_endpoint('tcp_client', self.target_host_var.get(), self.target_port_var.get())
        return host, port

    def send(self):
        worker = self.app.worker
        try:
            if not self.app._live_ready(self.app.sess) or not worker.commands.ready:
                raise ValueError('当前连接尚不能发送；串口/TCP需连接设备，回放不支持发送')
            target = self._target(worker)
            text = self.input.get('1.0', 'end-1c')
            mode = 'hex' if self.mode_var.get() == 'HEX 字节' else 'text'
            payload = encode_command(text, mode, self.ending_var.get())
            request_id = worker.commands.submit(payload, target)
        except ValueError as error:
            self.status_var.set(str(error))
            return None
        item = {'text': text, 'mode': self.mode_var.get(), 'ending': self.ending_var.get()}
        self.history.insert(0, item)
        del self.history[30:]
        self.history_box['values'] = [v['mode'] + ' · ' + v['text'].replace('\n', ' ↵ ')[:90] for v in self.history]
        self.history_box.current(0)
        self.status_var.set('命令 #%d 已排队 · %d B；等待连接写入结果' % (request_id, len(payload)))
        return request_id

    def load_history(self, index=None):
        index = self.history_box.current() if index is None else index
        if 0 <= index < len(self.history):
            item = self.history[index]
            self.mode_var.set(item['mode'])
            self.ending_var.set(item['ending'])
            self.input.delete('1.0', 'end')
            self.input.insert('1.0', item['text'])
            self.refresh()

    def _append_events(self, events):
        if not events:
            return
        self.log.configure(state='normal')
        for event in events:
            stamp = datetime.fromtimestamp(event['time'], timezone(timedelta(hours=8))).strftime('%H:%M:%S.%f')[:-3]
            data, kind = event['data'], event['kind']
            preview = (' '.join('%02x' % value for value in data[:512]) if kind == 'rx_hex' else
                       data[:512].decode('ascii', errors='backslashreplace').replace('\r', '\\r').replace('\n', '\\n'))
            if len(data) > 512 or event.get('size', len(data)) > len(data):
                preview += ' …（共 %d B）' % event.get('size', len(data))
            detail = event.get('message', '')
            if event.get('source'):
                detail = '来自 %s:%d' % event['source']
            if kind in ('tx', 'error'):
                target = event.get('target')
                detail = '#%d %s%s · %s' % (event['id'], event['endpoint'],
                         (' → %s:%d' % target) if target else '', detail)
                self.status_var.set(detail)
            self.log.insert('end', '[%s] %s %s%s\n' % (stamp, kind.upper(), preview, '  · '+detail if detail else ''), kind)
        lines = int(self.log.index('end-1c').split('.')[0])
        if lines > 1000:
            self.log.delete('1.0', '%d.0' % (lines-1000+1))
        self.log.see('end')
        self.log.configure(state='disabled')

    def refresh(self):
        worker = self.app.worker
        if worker is not self._worker:
            if self._worker is not None:
                self._append_events(self._worker.commands.poll())
            self._worker = worker
        if worker is not None:
            worker.commands.rx_mode = ('hex' if self.rx_var.get().startswith('HEX') else
                                       'text' if self.rx_var.get().startswith('文本') else 'off')
            self._append_events(worker.commands.poll())
        ready = bool(self.app._live_ready(self.app.sess) and worker.commands.ready)
        udp = (isinstance(worker, NetworkWorker) and worker.mode == 'udp') or (worker is None and self.app.source_var.get() == 'UDP 接收')
        if udp:
            if not self.udp_row.winfo_manager():
                self.udp_row.pack(fill='x', before=self.input_frame)
        else:
            self.udp_row.pack_forget()
        if ready and isinstance(worker, NetworkWorker) and worker.mode == 'udp' and self.target_var.get() == 'peer' and worker.peer is None:
            ready = False
        for entry in (self.target_host, self.target_port):
            entry.configure(state='normal' if self.target_var.get() == 'custom' else 'disabled')
        self.send_button.configure(state='normal' if ready else 'disabled')
        self.ending_box.configure(state='disabled' if self.mode_var.get() == 'HEX 字节' else 'readonly')
        endpoint = worker.commands.endpoint if worker is not None and worker.commands.ready else ''
        self.connection_var.set('当前连接：'+endpoint if endpoint else '当前无可发送连接 · TCP 服务端需等待设备接入')

    def clear_log(self):
        self.log.configure(state='normal')
        self.log.delete('1.0', 'end')
        self.log.configure(state='disabled')

    def save_log(self):
        content = self.log.get('1.0', 'end-1c')
        path = filedialog.asksaveasfilename(parent=self.parent.winfo_toplevel(), title='保存命令收发日志',
                                           defaultextension='.txt', filetypes=(('文本日志', '*.txt'),))
        if path:
            try:
                with open(path, 'w', encoding='utf-8') as output:
                    output.write(content)
            except OSError as error:
                self.status_var.set('日志保存失败：%s' % error)
            else:
                self.status_var.set('日志已保存：'+path)
