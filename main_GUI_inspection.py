# -*- coding: utf-8 -*-
"""
================================================================================
  PANNELLO DI CONTROLLO  -  Ispezione & Marcatura Casco
================================================================================
GUI di interazione per il workflow robotico (Techman + ZED).

>>> Questo file NON modifica nessun modulo esistente. <<<
Si limita a IMPORTARE e ORCHESTRARE le funzioni gia' presenti nel progetto:
    - robot_control.RobotController
    - camera_scripts_v2.init_zed
    - defects_id_wrapper.duplicate_filter
    - inspection_and_marking.point_and_shoot / refine_defect_position /
      mark_defect / move_to_hub

COME SI AVVIA
-------------
Posiziona questo file nella cartella radice del progetto (la stessa di
inspection_and_marking.py) e lancia:

    python gui_ispezione.py

Dipendenze: solo la libreria standard (tkinter). Nessuna installazione extra
rispetto a quelle gia' richieste dal progetto.

NOTE
----
- Gli import "pesanti" (robot, SDK ZED) vengono caricati in modo pigro alla
  prima azione, cosi' la finestra si apre sempre, anche su un PC senza hardware.
- Le operazioni robotiche girano su un thread separato: l'interfaccia non si
  blocca mai e il pulsante di ARRESTO resta sempre reattivo.
- I parametri modificati nella GUI agiscono solo sulla sessione in corso
  (in memoria). I file su disco restano intatti.
"""

import sys
import queue
import threading
import traceback

import tkinter as tk
from tkinter import ttk, font as tkfont


# =============================================================================
#  PALETTE / TEMA
# =============================================================================
COL = {
    "bg":        "#0f172a",   # slate-900 (sfondo finestra)
    "panel":     "#1e293b",   # slate-800 (card)
    "panel2":    "#273449",   # leggero contrasto
    "border":    "#334155",   # slate-700
    "text":      "#e2e8f0",   # slate-200
    "muted":     "#94a3b8",   # slate-400
    "accent":    "#2dd4bf",   # teal-400
    "accent_d":  "#14b8a6",   # teal-500
    "amber":     "#f59e0b",
    "green":     "#22c55e",
    "red":       "#ef4444",
    "red_d":     "#dc2626",
    "blue":      "#60a5fa",
}


# =============================================================================
#  REDIRECT DELLO STDOUT VERSO LA CONSOLE DELLA GUI
# =============================================================================
class _StreamToQueue:
    """Cattura le print() dei moduli e le inoltra alla coda della GUI."""
    def __init__(self, q, original):
        self.q = q
        self.original = original
        self._buf = ""

    def write(self, s):
        if self.original:
            try:
                self.original.write(s)
            except Exception:
                pass
        self._buf += s
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            self.q.put(("log", line))

    def flush(self):
        if self._buf:
            self.q.put(("log", self._buf))
            self._buf = ""
        if self.original:
            try:
                self.original.flush()
            except Exception:
                pass


# =============================================================================
#  APPLICAZIONE
# =============================================================================
class ControlPanel(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Pannello di Controllo  •  Ispezione & Marcatura Casco")
        self.geometry("1180x760")
        self.minsize(1040, 660)
        self.configure(bg=COL["bg"])

        # Stato runtime
        self.q = queue.Queue()
        self.busy = False
        self.controller = None          # RobotController
        self.zed = None
        self.runtime = None
        self.image_zed = None
        self.point_cloud = None
        self.defects = []               # set di lavoro corrente
        self._im = None                 # modulo inspection_and_marking (lazy)

        self._build_fonts()
        self._build_style()
        self._build_ui()

        self.after(80, self._poll_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.log("Pannello pronto. Collega robot e camera per iniziare.", "info")

    # ---------------------------------------------------------------- fonts
    def _build_fonts(self):
        base = "Segoe UI" if sys.platform.startswith("win") else "Helvetica"
        self.f_title = tkfont.Font(family=base, size=16, weight="bold")
        self.f_h     = tkfont.Font(family=base, size=11, weight="bold")
        self.f_body  = tkfont.Font(family=base, size=10)
        self.f_small = tkfont.Font(family=base, size=9)
        self.f_mono  = tkfont.Font(family="Consolas" if sys.platform.startswith("win") else "Monospace", size=9)

    # ---------------------------------------------------------------- style
    def _build_style(self):
        st = ttk.Style(self)
        try:
            st.theme_use("clam")
        except Exception:
            pass

        st.configure("TFrame", background=COL["bg"])
        st.configure("Card.TFrame", background=COL["panel"])
        st.configure("TLabel", background=COL["panel"], foreground=COL["text"], font=self.f_body)
        st.configure("Muted.TLabel", background=COL["panel"], foreground=COL["muted"], font=self.f_small)
        st.configure("Head.TLabel", background=COL["panel"], foreground=COL["accent"], font=self.f_h)
        st.configure("BgLabel.TLabel", background=COL["bg"], foreground=COL["text"])

        # Entry
        st.configure("TEntry",
                     fieldbackground=COL["panel2"], foreground=COL["text"],
                     bordercolor=COL["border"], insertcolor=COL["text"])
        # Checkbutton
        st.configure("TCheckbutton", background=COL["panel"], foreground=COL["text"], font=self.f_body)
        st.map("TCheckbutton", background=[("active", COL["panel"])])

        # Pulsanti
        def button(name, bg, fg=COL["bg"], active=None):
            active = active or bg
            st.configure(name, background=bg, foreground=fg, font=self.f_h,
                         borderwidth=0, focusthickness=0, padding=(12, 8))
            st.map(name,
                   background=[("active", active), ("disabled", COL["border"])],
                   foreground=[("disabled", COL["muted"])])

        button("Accent.TButton", COL["accent"], COL["bg"], COL["accent_d"])
        button("Ghost.TButton", COL["panel2"], COL["text"], COL["border"])
        button("Warn.TButton", COL["amber"], COL["bg"], "#d98a06")
        button("Stop.TButton", COL["red"], "#ffffff", COL["red_d"])
        button("Step.TButton", COL["blue"], COL["bg"], "#3b82f6")

        # Treeview
        st.configure("Treeview",
                     background=COL["panel2"], fieldbackground=COL["panel2"],
                     foreground=COL["text"], rowheight=24, borderwidth=0, font=self.f_small)
        st.configure("Treeview.Heading",
                     background=COL["border"], foreground=COL["text"],
                     font=self.f_small, relief="flat")
        st.map("Treeview", background=[("selected", COL["accent_d"])],
               foreground=[("selected", COL["bg"])])

    # ---------------------------------------------------------------- helpers UI
    def _card(self, parent, title):
        outer = tk.Frame(parent, bg=COL["border"])
        inner = tk.Frame(outer, bg=COL["panel"])
        inner.pack(fill="both", expand=True, padx=1, pady=1)
        if title:
            ttk.Label(inner, text=title, style="Head.TLabel").pack(anchor="w", padx=14, pady=(12, 6))
        return outer, inner

    def _row(self, parent):
        r = tk.Frame(parent, bg=COL["panel"])
        r.pack(fill="x", padx=14, pady=3)
        return r

    def _param(self, parent, label, default, width=8):
        r = self._row(parent)
        ttk.Label(r, text=label, style="TLabel").pack(side="left")
        var = tk.StringVar(value=str(default))
        e = ttk.Entry(r, textvariable=var, width=width, justify="right")
        e.pack(side="right")
        return var

    # ---------------------------------------------------------------- build UI
    def _build_ui(self):
        # ---- Header
        header = tk.Frame(self, bg=COL["bg"])
        header.pack(fill="x", padx=16, pady=(14, 8))

        tk.Label(header, text="🛠  Ispezione & Marcatura Casco",
                 bg=COL["bg"], fg=COL["text"], font=self.f_title).pack(side="left")

        self.estop_btn = ttk.Button(header, text="■  ARRESTO  EMERGENZA",
                                     style="Stop.TButton", command=self.do_estop)
        self.estop_btn.pack(side="right")

        # badge di stato
        badges = tk.Frame(header, bg=COL["bg"])
        badges.pack(side="right", padx=16)
        self.badge_robot = self._make_badge(badges, "Robot")
        self.badge_cam   = self._make_badge(badges, "Camera")

        # ---- Corpo a 3 colonne
        body = tk.Frame(self, bg=COL["bg"])
        body.pack(fill="both", expand=True, padx=16, pady=(0, 8))
        body.columnconfigure(0, weight=0, minsize=300)
        body.columnconfigure(1, weight=1, uniform="main")
        body.columnconfigure(2, weight=1, uniform="main")
        body.rowconfigure(0, weight=1)

        self._build_left(body)
        self._build_center(body)
        self._build_right(body)

        # ---- Status bar
        self.status = tk.Label(self, text="Pronto", anchor="w",
                               bg=COL["panel"], fg=COL["muted"], font=self.f_small,
                               padx=12, pady=4)
        self.status.pack(fill="x", side="bottom")

    def _make_badge(self, parent, name):
        f = tk.Frame(parent, bg=COL["bg"])
        f.pack(side="left", padx=6)
        dot = tk.Canvas(f, width=12, height=12, bg=COL["bg"], highlightthickness=0)
        oid = dot.create_oval(2, 2, 11, 11, fill=COL["red"], outline="")
        dot.pack(side="left")
        lbl = tk.Label(f, text=name, bg=COL["bg"], fg=COL["muted"], font=self.f_small)
        lbl.pack(side="left", padx=(5, 0))
        return {"canvas": dot, "oval": oid, "label": lbl}

    def _set_badge(self, badge, ok):
        badge["canvas"].itemconfig(badge["oval"], fill=COL["green"] if ok else COL["red"])
        badge["label"].config(fg=COL["text"] if ok else COL["muted"])

    # ---- COLONNA SINISTRA ---------------------------------------------------
    def _build_left(self, parent):
        col = tk.Frame(parent, bg=COL["bg"])
        col.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        # Connessione
        out, c = self._card(col, "Connessione")
        out.pack(fill="x", pady=(0, 10))
        r = self._row(c)
        ttk.Label(r, text="IP Robot").pack(side="left")
        self.ip_var = tk.StringVar(value="192.168.1.3")
        ttk.Entry(r, textvariable=self.ip_var, width=15, justify="right").pack(side="right")
        b = tk.Frame(c, bg=COL["panel"]); b.pack(fill="x", padx=14, pady=(8, 4))
        ttk.Button(b, text="Collega", style="Accent.TButton",
                   command=self.do_connect).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ttk.Button(b, text="Disconnetti", style="Ghost.TButton",
                   command=self.do_disconnect).pack(side="left", expand=True, fill="x", padx=(4, 0))
        ttk.Button(c, text="Posizione di default (home)", style="Ghost.TButton",
                   command=self.do_default).pack(fill="x", padx=14, pady=(2, 12))

        # Camera
        out, c = self._card(col, "Telecamera ZED")
        out.pack(fill="x", pady=(0, 10))
        b = tk.Frame(c, bg=COL["panel"]); b.pack(fill="x", padx=14, pady=(4, 12))
        ttk.Button(b, text="Inizializza", style="Accent.TButton",
                   command=self.do_init_camera).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ttk.Button(b, text="Chiudi", style="Ghost.TButton",
                   command=self.do_close_camera).pack(side="left", expand=True, fill="x", padx=(4, 0))

        # Parametri
        out, c = self._card(col, "Parametri (sessione)")
        out.pack(fill="both", expand=True)
        self.p_radius      = self._param(c, "Raggio ispezione [mm]", 300)
        self.p_close       = self._param(c, "Raggio ravvicinato [mm]", 300)
        self.p_insp_speed  = self._param(c, "Vel. ispezione [mm/s]", 400)
        self.p_mark_speed   = self._param(c, "Vel. marcatura [mm/s]", 200)
        self.p_nshots      = self._param(c, "Foto di raffinamento", 10)
        self.p_dupdist     = self._param(c, "Distanza duplicati [mm]", 25)
        self.p_minappr     = self._param(c, "Raggio approccio min [mm]", 250)
        rr = self._row(c)
        self.p_generic = tk.BooleanVar(value=False)
        ttk.Checkbutton(rr, text="Detection generica", variable=self.p_generic).pack(side="left")
        ttk.Label(c, text="I valori agiscono solo sulla sessione corrente.",
                  style="Muted.TLabel").pack(anchor="w", padx=14, pady=(6, 12))

    # ---- COLONNA CENTRALE ---------------------------------------------------
    def _build_center(self, parent):
        col = tk.Frame(parent, bg=COL["bg"])
        col.grid(row=0, column=1, sticky="nsew", padx=(0, 10))

        # Workflow
        out, c = self._card(col, "Workflow")
        out.pack(fill="x")
        grid = tk.Frame(c, bg=COL["panel"]); grid.pack(fill="x", padx=14, pady=(2, 6))
        grid.columnconfigure((0, 1), weight=1)

        steps = [
            ("1 · Ispezione", self.do_inspection),
            ("2 · Duplicati", self.do_dedup),
            ("3 · Raffina",   self.do_refine),
            ("4 · Marca",     self.do_mark),
        ]
        for i, (txt, cmd) in enumerate(steps):
            ttk.Button(grid, text=txt, style="Step.TButton", command=cmd)\
                .grid(row=i // 2, column=i % 2, sticky="ew", padx=4, pady=4)

        b = tk.Frame(c, bg=COL["panel"]); b.pack(fill="x", padx=14, pady=(2, 12))
        ttk.Button(b, text="▶  Esegui tutto", style="Accent.TButton",
                   command=self.do_run_all).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ttk.Button(b, text="↺  Hub", style="Ghost.TButton",
                   command=self.do_goto_hub).pack(side="left", expand=True, fill="x", padx=(4, 0))

        # Movimento manuale
        out2, c2 = self._card(col, "Movimento manuale (sferico)")
        out2.pack(fill="x", pady=(10, 10))

        r = self._row(c2)
        ttk.Label(r, text="Alpha [°]").pack(side="left")
        self.jog_alpha = tk.StringVar(value="0")
        ttk.Entry(r, textvariable=self.jog_alpha, width=7, justify="right").pack(side="left", padx=(6, 16))
        ttk.Label(r, text="Beta [°]").pack(side="left")
        self.jog_beta = tk.StringVar(value="90")
        ttk.Entry(r, textvariable=self.jog_beta, width=7, justify="right").pack(side="left", padx=(6, 0))
        ttk.Button(c2, text="Vai alla posizione", style="Ghost.TButton",
                   command=self.do_goto_sph).pack(fill="x", padx=14, pady=(8, 12))

        # Tabella difetti
        out3, c3 = self._card(col, "Difetti rilevati")
        out3.pack(fill="both", expand=True)
        cols = ("n", "r", "alpha", "beta", "area", "stato")
        labels = ("#", "r", "α", "β", "area", "stato")
        self.tree = ttk.Treeview(c3, columns=cols, show="headings", height=8)
        widths = {"n": 34, "r": 56, "alpha": 56, "beta": 56, "area": 60, "stato": 110}
        for cid, lab in zip(cols, labels):
            self.tree.heading(cid, text=lab)
            self.tree.column(cid, width=widths[cid], anchor="center", stretch=True)
        self.tree.pack(fill="both", expand=True, padx=12, pady=(2, 6))
        ttk.Label(c3, text="Aggiornata dopo ispezione e raffinamento.",
                  style="Muted.TLabel").pack(anchor="w", padx=14, pady=(0, 10))

    # ---- COLONNA DESTRA -----------------------------------------------------
    def _build_right(self, parent):
        col = tk.Frame(parent, bg=COL["bg"])
        col.grid(row=0, column=2, sticky="nsew")
        out, c = self._card(col, "Console")
        out.pack(fill="both", expand=True)

        wrap = tk.Frame(c, bg=COL["panel"])
        wrap.pack(fill="both", expand=True, padx=12, pady=(2, 8))
        self.console = tk.Text(wrap, bg="#0b1220", fg=COL["text"], font=self.f_mono,
                               width=20, wrap="word", relief="flat", borderwidth=0,
                               insertbackground=COL["text"], state="disabled")
        sb = ttk.Scrollbar(wrap, command=self.console.yview)
        self.console.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.console.pack(side="left", fill="both", expand=True)

        for tag, color in (("info", COL["muted"]), ("ok", COL["green"]),
                           ("warn", COL["amber"]), ("err", COL["red"]),
                           ("robot", COL["accent"])):
            self.console.tag_config(tag, foreground=color)

        ttk.Button(c, text="Pulisci console", style="Ghost.TButton",
                   command=self._clear_console).pack(fill="x", padx=12, pady=(0, 12))

    # =========================================================================
    #  LOG / CODA
    # =========================================================================
    def log(self, msg, level="info"):
        self.q.put(("log", msg, level))

    def _clear_console(self):
        self.console.config(state="normal")
        self.console.delete("1.0", "end")
        self.console.config(state="disabled")

    def _append_console(self, msg, level="info"):
        # auto-classifica le print dei moduli
        if level == "info":
            low = msg.lower()
            if "[skip]" in low or "attenzione" in low or "impossibile" in low:
                level = "warn"
            elif "[refine]" in low or "completat" in low or "connesso" in low:
                level = "ok"
            elif "errore" in low or "error" in low or "traceback" in low:
                level = "err"
        self.console.config(state="normal")
        self.console.insert("end", msg + "\n", level)
        self.console.see("end")
        self.console.config(state="disabled")

    def _poll_queue(self):
        try:
            while True:
                item = self.q.get_nowait()
                kind = item[0]
                if kind == "log":
                    msg = item[1]
                    level = item[2] if len(item) > 2 else "info"
                    self._append_console(msg, level)
                elif kind == "status":
                    self.status.config(text=item[1])
                elif kind == "badge":
                    self._set_badge(self.badge_robot if item[1] == "robot" else self.badge_cam, item[2])
                elif kind == "busy":
                    self._set_busy(item[1])
                elif kind == "defects":
                    self._refresh_table()
        except queue.Empty:
            pass
        self.after(80, self._poll_queue)

    def _set_busy(self, busy):
        self.busy = busy
        self.status.config(text="Operazione in corso…" if busy else "Pronto")

    # =========================================================================
    #  TABELLA DIFETTI
    # =========================================================================
    def _refresh_table(self):
        self.tree.delete(*self.tree.get_children())
        for i, d in enumerate(self.defects, 1):
            sph = getattr(d, "sph_coord", None)
            area = getattr(d, "area", "")
            r = a = b = "—"
            if sph is not None:
                try:
                    r, a, b = (f"{sph[0]:.0f}", f"{sph[1]:.0f}", f"{sph[2]:.0f}")
                except Exception:
                    pass
            stato = getattr(d, "_gui_state", "rilevato")
            try:
                area = f"{float(area):.0f}"
            except Exception:
                area = str(area)
            self.tree.insert("", "end", values=(i, r, a, b, area, stato))

    # =========================================================================
    #  IMPORT PIGRO + APPLICAZIONE PARAMETRI
    # =========================================================================
    def _ensure_modules(self):
        """Importa inspection_and_marking (con tutto il suo albero) solo quando serve."""
        if self._im is None:
            import inspection_and_marking as im
            self._im = im
        return self._im

    def _f(self, var, cast=float, default=0):
        try:
            return cast(var.get())
        except Exception:
            return default

    def _apply_params(self, im):
        """Applica i parametri della GUI al modulo IN MEMORIA (i file non vengono toccati)."""
        im.INSPECTION_RADIUS       = self._f(self.p_radius, float, 300)
        im.CLOSE_INSPECTION_RADIUS = self._f(self.p_close, float, 300)
        im.INSPECTION_SPEED        = self._f(self.p_insp_speed, int, 400)
        im.MARKING_SPEED           = self._f(self.p_mark_speed, int, 200)
        im.N_CLOSE_SHOTS           = self._f(self.p_nshots, int, 10)
        im.DUPLICATE_DISTANCE      = self._f(self.p_dupdist, float, 25)
        im.MIN_APPROACH_RADIUS     = self._f(self.p_minappr, float, 250)
        im.GENERIC_DETECTION       = bool(self.p_generic.get())

    # =========================================================================
    #  ESECUZIONE TASK SU THREAD
    # =========================================================================
    def _run_task(self, name, fn, require_robot=False, require_cam=False):
        if self.busy:
            self.log("Attendere: un'operazione è già in corso.", "warn")
            return
        if require_robot and self.controller is None:
            self.log("Robot non collegato.", "warn")
            return
        if require_cam and self.zed is None:
            self.log("Telecamera non inizializzata.", "warn")
            return

        def worker():
            self.q.put(("busy", True))
            old_out, old_err = sys.stdout, sys.stderr
            sys.stdout = _StreamToQueue(self.q, old_out)
            sys.stderr = _StreamToQueue(self.q, old_err)
            try:
                self.q.put(("log", f"── {name} ──", "robot"))
                fn()
                self.q.put(("log", f"✓ {name} completato.", "ok"))
            except Exception as e:
                self.q.put(("log", f"✗ Errore in «{name}»: {e}", "err"))
                self.q.put(("log", traceback.format_exc(), "err"))
            finally:
                sys.stdout, sys.stderr = old_out, old_err
                self.q.put(("busy", False))
                self.q.put(("defects",))

        threading.Thread(target=worker, daemon=True).start()

    # =========================================================================
    #  AZIONI
    # =========================================================================
    def do_connect(self):
        def job():
            from robot_control import RobotController
            im = self._ensure_modules()
            from Variables import LOOK_DOWN_POSITION_J_INIZIO
            ip = self.ip_var.get().strip()
            print(f"Connessione al robot {ip} …")
            ctrl = RobotController(ip_address=ip, default_position_j=LOOK_DOWN_POSITION_J_INIZIO)
            try:
                ctrl.disconnect()
            except Exception:
                pass
            ctrl.connect()
            self.controller = ctrl
            self.q.put(("badge", "robot", True))
            print("Robot connesso.")
        self._run_task("Connessione robot", job)

    def do_disconnect(self):
        def job():
            if self.controller:
                self.controller.disconnect()
                self.controller = None
            self.q.put(("badge", "robot", False))
            print("Robot disconnesso.")
        self._run_task("Disconnessione robot", job)

    def do_default(self):
        def job():
            self.controller.default_positioning()
            print("Robot in posizione di default.")
        self._run_task("Posizione di default", job, require_robot=True)

    def do_init_camera(self):
        def job():
            from camera_scripts_v2 import init_zed
            print("Inizializzazione ZED …")
            self.zed, self.runtime, self.image_zed, self.point_cloud = init_zed()
            self.q.put(("badge", "cam", True))
            print("Telecamera ZED pronta.")
        self._run_task("Init camera", job)

    def do_close_camera(self):
        def job():
            if self.zed is not None:
                try:
                    self.zed.close()
                except Exception:
                    pass
            self.zed = self.runtime = self.image_zed = self.point_cloud = None
            self.q.put(("badge", "cam", False))
            print("Telecamera chiusa.")
        self._run_task("Chiusura camera", job)

    def do_inspection(self):
        def job():
            im = self._ensure_modules()
            self._apply_params(im)
            radius = im.INSPECTION_RADIUS
            collected = []
            for test_sph in im.INSPECTION_POSITIONS:
                print(f"→ Posizione (alpha={test_sph[1]}°, beta={test_sph[2]}°)")
                defect_list, _dbg, _mask, _bgr = im.point_and_shoot(
                    self.controller, self.zed, self.runtime,
                    self.image_zed, self.point_cloud,
                    test_sph=test_sph,
                    helmet_center=im.HELMET_CENTER_GLOBAL,
                    insp_radius=radius,
                )
                for d in defect_list:
                    d._gui_state = "rilevato"
                collected.extend(defect_list)
                print(f"   rilevati {len(defect_list)} difetti.")
            self.defects = collected
            print(f"Totale (pre-filtro): {len(collected)} difetti.")
        self._run_task("Ispezione globale", job, require_robot=True, require_cam=True)

    def do_dedup(self):
        def job():
            im = self._ensure_modules()
            self._apply_params(im)
            from defects_id_wrapper import duplicate_filter
            before = len(self.defects)
            self.defects = duplicate_filter(self.defects, distance_threshold=im.DUPLICATE_DISTANCE)
            print(f"Duplicati rimossi: {before} → {len(self.defects)} difetti unici.")
        self._run_task("Filtro duplicati", job)

    def do_refine(self):
        def job():
            im = self._ensure_modules()
            self._apply_params(im)
            if not self.defects:
                print("Nessun difetto da raffinare.")
                return
            kept = []
            for i, d in enumerate(self.defects, 1):
                print(f"Raffinamento difetto {i}/{len(self.defects)} …")
                ok = im.refine_defect_position(
                    self.controller, self.zed, self.runtime,
                    self.image_zed, self.point_cloud, d,
                    helmet_center=im.HELMET_CENTER_GLOBAL,
                    close_radius=im.CLOSE_INSPECTION_RADIUS,
                    n_shots=int(im.N_CLOSE_SHOTS),
                    generic_detection=im.GENERIC_DETECTION,
                )
                d._gui_state = "raffinato" if ok else "scartato"
                if ok:
                    kept.append(d)
                self.q.put(("defects",))
            print(f"Difetti validi dopo raffinamento: {len(kept)}/{len(self.defects)}.")
        self._run_task("Raffinamento", job, require_robot=True, require_cam=True)

    def do_mark(self):
        def job():
            im = self._ensure_modules()
            self._apply_params(im)
            to_mark = [d for d in self.defects if getattr(d, "_gui_state", "") != "scartato"]
            if not to_mark:
                print("Nessun difetto da marcare.")
                return
            for i, d in enumerate(to_mark, 1):
                print(f"Marcatura difetto {i}/{len(to_mark)} …")
                ok = im.mark_defect(
                    self.controller, d,
                    helmet_center=im.HELMET_CENTER_GLOBAL,
                    min_approach_radius=im.MIN_APPROACH_RADIUS,
                    marking_speed=int(im.MARKING_SPEED),
                )
                d._gui_state = "marcato" if ok else "errore mark"
                self.q.put(("defects",))
            im.move_to_hub(self.controller)
            print("Marcatura terminata. Robot all'hub.")
        self._run_task("Marcatura", job, require_robot=True)

    def do_run_all(self):
        def job():
            im = self._ensure_modules()
            self._apply_params(im)
            from defects_id_wrapper import duplicate_filter

            # 1) ispezione
            collected = []
            for test_sph in im.INSPECTION_POSITIONS:
                print(f"→ Posizione (alpha={test_sph[1]}°, beta={test_sph[2]}°)")
                dl, *_ = im.point_and_shoot(
                    self.controller, self.zed, self.runtime,
                    self.image_zed, self.point_cloud,
                    test_sph=test_sph, helmet_center=im.HELMET_CENTER_GLOBAL,
                    insp_radius=im.INSPECTION_RADIUS,
                )
                for d in dl:
                    d._gui_state = "rilevato"
                collected.extend(dl)
            self.defects = collected
            self.q.put(("defects",))
            print(f"Totale: {len(collected)} difetti.")

            # 2) dedup
            self.defects = duplicate_filter(self.defects, distance_threshold=im.DUPLICATE_DISTANCE)
            self.q.put(("defects",))
            print(f"Difetti unici: {len(self.defects)}.")

            # 3) refine
            kept = []
            for d in self.defects:
                ok = im.refine_defect_position(
                    self.controller, self.zed, self.runtime,
                    self.image_zed, self.point_cloud, d,
                    helmet_center=im.HELMET_CENTER_GLOBAL,
                    close_radius=im.CLOSE_INSPECTION_RADIUS,
                    n_shots=int(im.N_CLOSE_SHOTS),
                    generic_detection=im.GENERIC_DETECTION,
                )
                d._gui_state = "raffinato" if ok else "scartato"
                if ok:
                    kept.append(d)
                self.q.put(("defects",))

            # 4) mark
            for d in kept:
                ok = im.mark_defect(
                    self.controller, d, helmet_center=im.HELMET_CENTER_GLOBAL,
                    min_approach_radius=im.MIN_APPROACH_RADIUS,
                    marking_speed=int(im.MARKING_SPEED),
                )
                d._gui_state = "marcato" if ok else "errore mark"
                self.q.put(("defects",))
            im.move_to_hub(self.controller)
            print("Workflow completo terminato.")
        self._run_task("Workflow completo", job, require_robot=True, require_cam=True)

    def do_goto_hub(self):
        def job():
            im = self._ensure_modules()
            self._apply_params(im)
            im.move_to_hub(self.controller)
        self._run_task("Ritorno a hub", job, require_robot=True)

    def do_goto_sph(self):
        def job():
            im = self._ensure_modules()
            self._apply_params(im)
            from spherical_movement import move_circle_spherical
            a = self._f(self.jog_alpha, float, 0)
            b = self._f(self.jog_beta, float, 90)
            print(f"Movimento sferico verso alpha={a}°, beta={b}° …")
            ok = move_circle_spherical(
                controller=self.controller,
                end_sph_coord=[im.INSPECTION_RADIUS, a, b],
                radius=im.INSPECTION_RADIUS,
                tool_pose_ee=im.CAMERA_POSE_EE,
                helmet_center=im.HELMET_CENTER_GLOBAL,
                speed=int(im.INSPECTION_SPEED),
            )
            print("Posizione raggiunta." if ok else "[SKIP] Posizione non raggiungibile in sicurezza.")
        self._run_task("Movimento manuale", job, require_robot=True)

    def do_estop(self):
        """Arresto immediato: eseguito subito, anche durante un task."""
        try:
            if self.controller is not None:
                self.controller.emergency_stop()
                self.log("⛔ ARRESTO DI EMERGENZA inviato al robot.", "err")
            else:
                self.log("Robot non collegato: nessun arresto da inviare.", "warn")
        except Exception as e:
            self.log(f"Errore durante l'arresto: {e}", "err")

    # ---------------------------------------------------------------- chiusura
    def _on_close(self):
        try:
            if self.zed is not None:
                self.zed.close()
        except Exception:
            pass
        try:
            if self.controller is not None:
                self.controller.disconnect()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    ControlPanel().mainloop()
