import tkinter as tk
from tkinter import ttk, messagebox
import serial
from serial.tools import list_ports


BAUDRATE = 9600
REPEAT_MS = 100  # Enviar comando cada 100 ms mientras el boton esta presionado

# Paleta rojo intenso + azul oscuro + blanco
BG_MAIN = "#070b1f"
BG_PANEL = "#0f1b3d"
BG_INPUT = "#132650"
FG_TEXT = "#ffffff"
ACCENT = "#d90429"
ACCENT_SOFT = "#ff4d6d"
LOG_BG = "#091433"


class MotorGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Control Motores DC - Infernal Dark")
        self.root.geometry("540x470")
        self.root.resizable(False, False)
        self.root.configure(bg=BG_MAIN)

        self.ser = None
        self.active_buttons = {}  # cmd -> after_id
        self.keyboard_pressed_cmds = set()
        self._keyboard_pause_after_stop = False

        self._configure_styles()
        self._build_ui()
        self._setup_keyboard_bindings()
        self._refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _configure_styles(self):
        style = ttk.Style()
        style.theme_use("clam")

        style.configure("Dark.TFrame", background=BG_MAIN)
        style.configure(
            "Dark.TLabelframe",
            background=BG_PANEL,
            bordercolor=ACCENT,
            borderwidth=1,
            relief="solid",
        )
        style.configure(
            "Dark.TLabelframe.Label",
            background=BG_PANEL,
            foreground=ACCENT_SOFT,
            font=("Segoe UI", 10, "bold"),
        )
        style.configure(
            "Dark.TLabel",
            background=BG_PANEL,
            foreground=FG_TEXT,
            font=("Segoe UI", 10),
        )
        style.configure(
            "Dark.TEntry",
            fieldbackground=BG_INPUT,
            foreground=FG_TEXT,
            bordercolor=ACCENT,
            insertcolor=FG_TEXT,
        )
        style.configure(
            "Dark.TCombobox",
            fieldbackground=BG_INPUT,
            foreground=FG_TEXT,
            bordercolor=ACCENT,
            arrowsize=14,
        )
        style.map(
            "Dark.TCombobox",
            fieldbackground=[("readonly", BG_INPUT)],
            foreground=[("readonly", FG_TEXT)],
        )
        style.configure(
            "Ghost.TButton",
            background=BG_INPUT,
            foreground=FG_TEXT,
            bordercolor=ACCENT,
            borderwidth=1,
            focuscolor=ACCENT,
            padding=6,
            font=("Segoe UI", 9, "bold"),
        )
        style.map(
            "Ghost.TButton",
            background=[("active", ACCENT), ("pressed", ACCENT)],
            foreground=[("active", "#1a0c18"), ("pressed", "#1a0c18")],
        )

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=12, style="Dark.TFrame")
        main.pack(fill="both", expand=True)

        title = tk.Label(
            main,
            text="Infernal Motor Control",
            bg=BG_MAIN,
            fg=ACCENT,
            font=("Segoe UI", 16, "bold"),
        )
        title.pack(pady=(0, 10))

        self._create_devil_badge(main)

        # Conexion serial
        conn_frame = ttk.LabelFrame(
            main, text="Conexion Serial", padding=10, style="Dark.TLabelframe"
        )
        conn_frame.pack(fill="x", pady=(0, 10))

        ttk.Label(conn_frame, text="Puerto COM:", style="Dark.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            conn_frame,
            textvariable=self.port_var,
            state="readonly",
            width=18,
            style="Dark.TCombobox",
        )
        self.port_combo.grid(row=0, column=1, padx=6, pady=4, sticky="w")

        ttk.Button(
            conn_frame, text="Actualizar", style="Ghost.TButton", command=self._refresh_ports
        ).grid(row=0, column=2, padx=4)
        self.btn_connect = ttk.Button(
            conn_frame, text="Conectar", style="Ghost.TButton", command=self._toggle_connection
        )
        self.btn_connect.grid(row=0, column=3, padx=4)

        self.status_var = tk.StringVar(value="Desconectado")
        ttk.Label(conn_frame, textvariable=self.status_var, style="Dark.TLabel").grid(
            row=1, column=0, columnspan=4, sticky="w", pady=(6, 0)
        )

        # Velocidad
        rpm_frame = ttk.LabelFrame(
            main, text="Control de Velocidad", padding=10, style="Dark.TLabelframe"
        )
        rpm_frame.pack(fill="x", pady=(0, 10))

        ttk.Label(rpm_frame, text="RPM (0 a 6):", style="Dark.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        self.rpm_var = tk.StringVar(value="3")
        self.rpm_entry = ttk.Entry(
            rpm_frame, textvariable=self.rpm_var, width=10, style="Dark.TEntry"
        )
        self.rpm_entry.grid(row=0, column=1, padx=6, sticky="w")
        ttk.Button(
            rpm_frame, text="Enviar RPM", style="Ghost.TButton", command=self._send_rpm
        ).grid(
            row=0, column=2, padx=6
        )

        # Motores
        motors_frame = ttk.LabelFrame(
            main,
            text="Giro — raton o teclado A/B/C/D (mantener)",
            padding=10,
            style="Dark.TLabelframe",
        )
        motors_frame.pack(fill="both", expand=True)

        tk.Label(
            motors_frame,
            text=(
                "Teclado: A = Motor1 horario, B = Motor1 antihorario, C = Motor2 horario, "
                "D = Motor2 antihorario. "
                "Al soltar cualquier tecla o boton del raton, ambos motores se detienen al instante; "
                "vuelva a pulsar cuando haya liberado todas las teclas direccionales si tenia mas de una."
            ),
            fg=ACCENT_SOFT,
            bg=BG_PANEL,
            font=("Segoe UI", 8),
            wraplength=480,
            justify="left",
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))

        # Motor 1
        m1_frame = ttk.LabelFrame(
            motors_frame, text="Motor 1", padding=8, style="Dark.TLabelframe"
        )
        m1_frame.grid(row=1, column=0, padx=8, pady=8, sticky="nsew")
        btn_m1_cw = tk.Button(
            m1_frame,
            text="Horario (A)",
            width=18,
            bg=BG_INPUT,
            fg=FG_TEXT,
            activebackground=ACCENT,
            activeforeground="#1a0c18",
            bd=1,
            relief="solid",
            highlightthickness=1,
            highlightbackground=ACCENT,
            font=("Segoe UI", 10, "bold"),
        )
        btn_m1_ccw = tk.Button(
            m1_frame,
            text="Antihorario (B)",
            width=18,
            bg=BG_INPUT,
            fg=FG_TEXT,
            activebackground=ACCENT,
            activeforeground="#1a0c18",
            bd=1,
            relief="solid",
            highlightthickness=1,
            highlightbackground=ACCENT,
            font=("Segoe UI", 10, "bold"),
        )
        btn_m1_cw.pack(pady=6)
        btn_m1_ccw.pack(pady=6)

        # Motor 2
        m2_frame = ttk.LabelFrame(
            motors_frame, text="Motor 2", padding=8, style="Dark.TLabelframe"
        )
        m2_frame.grid(row=1, column=1, padx=8, pady=8, sticky="nsew")
        btn_m2_cw = tk.Button(
            m2_frame,
            text="Horario (C)",
            width=18,
            bg=BG_INPUT,
            fg=FG_TEXT,
            activebackground=ACCENT,
            activeforeground="#1a0c18",
            bd=1,
            relief="solid",
            highlightthickness=1,
            highlightbackground=ACCENT,
            font=("Segoe UI", 10, "bold"),
        )
        btn_m2_ccw = tk.Button(
            m2_frame,
            text="Antihorario (D)",
            width=18,
            bg=BG_INPUT,
            fg=FG_TEXT,
            activebackground=ACCENT,
            activeforeground="#1a0c18",
            bd=1,
            relief="solid",
            highlightthickness=1,
            highlightbackground=ACCENT,
            font=("Segoe UI", 10, "bold"),
        )
        btn_m2_cw.pack(pady=6)
        btn_m2_ccw.pack(pady=6)

        motors_frame.columnconfigure(0, weight=1)
        motors_frame.columnconfigure(1, weight=1)

        # Asignar eventos de presionar/soltar
        self._bind_hold_button(btn_m1_cw, "A")
        self._bind_hold_button(btn_m1_ccw, "B")
        self._bind_hold_button(btn_m2_cw, "C")
        self._bind_hold_button(btn_m2_ccw, "D")

        # Consola de estado
        log_frame = ttk.LabelFrame(main, text="Mensajes", padding=10, style="Dark.TLabelframe")
        log_frame.pack(fill="both", expand=True)
        self.log = tk.Text(
            log_frame,
            height=7,
            state="disabled",
            bg=LOG_BG,
            fg=ACCENT_SOFT,
            insertbackground=FG_TEXT,
            relief="solid",
            bd=1,
            highlightthickness=1,
            highlightbackground=ACCENT,
            font=("Consolas", 10),
        )
        self.log.pack(fill="both", expand=True)

    def _refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        self._log(f"Puertos detectados: {', '.join(ports) if ports else 'ninguno'}")

    def _create_devil_badge(self, parent):
        # Dibujo simple de diablito en esquina superior derecha
        badge = tk.Canvas(
            parent,
            width=88,
            height=88,
            bg=BG_MAIN,
            highlightthickness=0,
            bd=0,
        )
        badge.place(relx=1.0, y=0, x=-8, anchor="ne")

        # Cuernos
        badge.create_polygon(24, 30, 30, 8, 40, 28, fill=ACCENT, outline=ACCENT_SOFT)
        badge.create_polygon(48, 28, 58, 8, 64, 30, fill=ACCENT, outline=ACCENT_SOFT)

        # Cara
        badge.create_oval(20, 24, 68, 72, fill=ACCENT, outline=ACCENT_SOFT, width=2)

        # Ojos
        badge.create_oval(30, 42, 38, 50, fill=FG_TEXT, outline="")
        badge.create_oval(50, 42, 58, 50, fill=FG_TEXT, outline="")

        # Sonrisa
        badge.create_arc(
            30, 48, 58, 66, start=200, extent=140, style="arc", outline=FG_TEXT, width=2
        )

        # Colmillos
        badge.create_polygon(39, 60, 42, 66, 45, 60, fill=FG_TEXT, outline="")
        badge.create_polygon(47, 60, 50, 66, 53, 60, fill=FG_TEXT, outline="")

    def _toggle_connection(self):
        if self.ser and self.ser.is_open:
            self._disconnect_serial()
        else:
            self._connect_serial()

    def _connect_serial(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("Puerto COM", "Selecciona un puerto COM.")
            return

        try:
            self.ser = serial.Serial(port, BAUDRATE, timeout=0.1)
            self.status_var.set(f"Conectado a {port} @ {BAUDRATE}")
            self.btn_connect.config(text="Desconectar")
            self._log(f"Conectado a {port}")
        except Exception as e:
            self.ser = None
            messagebox.showerror("Error de conexion", f"No se pudo conectar:\n{e}")

    def _disconnect_serial(self):
        self._immediate_stop_directional(log_reason=False)
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        finally:
            self.status_var.set("Desconectado")
            self.btn_connect.config(text="Conectar")
            self._log("Puerto serial desconectado.")

    def _send_line(self, text: str):
        if not (self.ser and self.ser.is_open):
            return
        try:
            self.ser.write((text + "\n").encode("utf-8"))
        except Exception as e:
            self._log(f"Error serial: {e}")
            self._disconnect_serial()

    def _send_rpm(self):
        try:
            rpm = float(self.rpm_var.get().strip())
        except ValueError:
            messagebox.showwarning("RPM invalida", "Ingresa un numero entre 0 y 6.")
            return

        if rpm < 0:
            rpm = 0
        if rpm > 6:
            rpm = 6

        self.rpm_var.set(str(rpm))
        self._send_line(str(rpm))
        self._log(f"RPM enviada: {rpm}")

    def _setup_keyboard_bindings(self):
        """A/B/C/D desde teclado fisico; no aplicar cuando se escribe RPM."""
        self.root.bind_all("<KeyPress>", self._kbd_press, add="+")
        self.root.bind_all("<KeyRelease>", self._kbd_release, add="+")

    def _motor_keys_allowed(self):
        w = self.root.focus_get()
        if isinstance(w, (tk.Entry, ttk.Entry)) and getattr(self, "rpm_entry", None) is w:
            return False
        return True

    @staticmethod
    def _motor_key_command(event):
        ks = getattr(event, "keysym", "") or ""
        if len(ks) == 1 and ks.lower() in "abcd":
            return ks.upper()
        ch = getattr(event, "char", "") or ""
        if len(ch) == 1 and ch.lower() in "abcd":
            return ch.upper()
        return None

    def _kbd_press(self, event):
        if not self._motor_keys_allowed():
            return
        cmd = self._motor_key_command(event)
        if cmd is None:
            return
        self.keyboard_pressed_cmds.add(cmd)
        if self._keyboard_pause_after_stop:
            return
        self._start_hold(cmd)

    def _kbd_release(self, event):
        if not self._motor_keys_allowed():
            return
        cmd = self._motor_key_command(event)
        if cmd is None:
            return
        self.keyboard_pressed_cmds.discard(cmd)
        self._immediate_stop_directional()
        self._keyboard_pause_after_stop = True
        if len(self.keyboard_pressed_cmds) == 0:
            self._keyboard_pause_after_stop = False

    def _bind_hold_button(self, button, cmd: str):
        button.bind("<ButtonPress-1>", lambda _: self._start_hold(cmd))
        button.bind("<ButtonRelease-1>", lambda _: self._immediate_stop_directional())
        button.bind("<Leave>", lambda _: self._immediate_stop_directional())

    def _start_hold(self, cmd: str):
        # Evita duplicar timers del mismo comando
        if cmd in self.active_buttons:
            return
        self._log(f"Presionado: {cmd}")
        self._repeat_send(cmd)

    def _repeat_send(self, cmd: str):
        self._send_line(cmd)
        after_id = self.root.after(REPEAT_MS, lambda: self._repeat_send(cmd))
        self.active_buttons[cmd] = after_id

    def _immediate_stop_directional(self, log_reason=True):
        had = len(self.active_buttons) > 0
        for cmd in list(self.active_buttons.keys()):
            after_id = self.active_buttons.pop(cmd, None)
            if after_id is not None:
                self.root.after_cancel(after_id)
        self._send_line("S")
        if had and log_reason:
            self._log("Parada: ambos motores (soltar).")

    def _log(self, msg: str):
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _on_close(self):
        self._disconnect_serial()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = MotorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
