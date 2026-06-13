import kinematics_v2 as kin
from robot_control import RobotController

from Variables import HELMET_CENTER_GLOBAL

import cv2
import numpy as np
import time
from matplotlib import pyplot as plt

def variable_helmet_radius(alpha, beta, r_apex=380.0, r_side=300.0, r_back=300.0, r_front=450.0, r_min=300.0):
    """
    Raggio di lavoro variabile in funzione degli angoli sferici (alpha, beta).

    MOTIVAZIONE GEOMETRICA
    Il casco non è una sfera centrata su HELMET_CENTER_GLOBAL. Per raggiungere i
    difetti bassi (sotto il centro, dove alpha sforerebbe i +/-90) si abbassa il
    centro del casco; così facendo però l'apice reale del casco si avvicina al
    centro e, a raggio costante, la camera all'apice rischia la collisione.
    Si compensa allontanando la camera in alto (apice) e tenendola più vicina
    ai lati e sul retro: una superficie tipo ellissoide/paraboloide invece di una sfera.

    MODELLO (forma quadratica liscia)
        r(a, b) = r_apex
                  - (r_apex - r_side) * sin^2(alpha)   # riduzione verso i lati
                  - (r_apex - r_back) * cos^2(beta)    # riduzione verso retro
                  + (r_front - r_apex) * sin^2(beta-90) # aumento verso il fronte

    PUNTI DI ANCORAGGIO (con i default richiesti):
        - alpha=0,    beta=90  -> r_apex = 400   (apice del casco)
        - alpha=+-90, beta=90  -> r_side = 300   (lati)
        - alpha=0,    beta=0   -> r_back = 300   (retro)
        - alpha=0,    beta=180 -> r_front = 500  (fronte)

    NOTE
    - sin^2(alpha): 0 ad alpha=0, 1 ad alpha=+-90 -> sposta dal valore apice a quello laterale.
    - cos^2(beta):  0 a beta=90, 1 a beta=0   -> sposta dal valore apice a quello retro.
    - sin^2(beta-90): 0 a beta=90, 1 a beta=180 -> sposta dal valore apice a quello frontale.
    - Le due riduzioni si sommano: negli "angoli" (alpha alto E beta lontano da 90)
      il raggio può scendere parecchio, perciò viene applicato un clamp a r_min.

    :param alpha: angolo azimutale [gradi].
    :param beta: angolo di elevazione [gradi] (0 retro, 90 apice, 180 fronte).
    :return: raggio in mm.
    """
    a = np.radians(alpha)
    b = np.radians(beta)

    if beta <= 90.0:
        r_beta = r_apex - (r_apex - r_back) * np.cos(b) ** 2
        r = r_beta - (r_beta - r_side) * np.sin(a) ** 2
    else:
        r_alpha = r_apex - (r_apex - r_side) * np.sin(a) ** 2
        r_front_beta = r_apex + (r_front - r_apex) * np.sin(np.radians(beta - 90.0)) ** 2
        w = np.clip((beta - 90.0) / 90.0, 0.0, 1.0)
        r = (1.0 - w) * r_alpha + w * r_front_beta

    return max(r, r_min)


def make_radius_fn(radius):
    """
    Normalizza il parametro 'radius' in una funzione radius(alpha, beta).

    Accetta sia uno scalare (raggio costante: comportamento legacy) sia una
    funzione gia' dipendente dagli angoli (es. variable_helmet_radius), cosi'
    move_circle_spherical resta compatibile con tutte le chiamate esistenti.
    """
    if callable(radius):
        return radius
    return lambda alpha, beta: float(radius)


def angles_unsafe(alpha, beta):
    """
    Verifica se gli angoli sferici (alpha, beta) si trovano al di fuori dei limiti 
    di sicurezza o all'interno di zone di collisione (dietro, davanti, o laterale eccessivo).
    
    Convenzione corrente:
    - alpha: intervallo [-90, 90] gradi.
    - beta: intervallo [0, 180] gradi (0 = retro, 90 = sommità, 180 = fronte).
    
    :return: True se la configurazione è pericolosa/non ammessa, False altrimenti.
    """
    # 1. Verifica dei limiti geometrici del dominio della convenzione
    if not (0.0 <= beta <= 180.0):
        return True

    # 2. Controllo limite laterale: alpha deve essere compreso entro +/- 89 gradi
    if alpha > 89 or alpha < -89.0:
        return True

    # 3. Controllo zona posteriore (retro): più larga che alta
    # A alpha = 0 la soglia è 20, a alpha = +/-90 la soglia sale a 45
    beta_soglia_retro = 20 + 25.0 * (alpha / 90.0)**2
    if beta < beta_soglia_retro:
        return True

    # 4. Controllo zona anteriore (fronte): più stretta che alta
    # A alpha = 0 la soglia massima ammessa è 110; il termine in alpha è moltiplicato per zero,
    # quindi il valore rimane costante per tutti gli angoli laterali.
    beta_soglia_fronte = 110.0 + 0 * (alpha / 90.0)**2
    if beta > beta_soglia_fronte:
        return True

    return False

def is_trajectory_unsafe(start_alpha, start_beta, end_alpha, end_beta, steps=100):
    """
    Verifica la sicurezza dell'intera traiettoria sferica tramite discretizzazione.
    Restituisce False se tutti i punti intermedi sono sicuri, True altrimenti.
    """
    alphas = np.linspace(start_alpha, end_alpha, num=steps)
    betas = np.linspace(start_beta, end_beta, num=steps)
    
    for a, b in zip(alphas, betas):
        if angles_unsafe(a, b):
            return True
            
    return False

def move_circle_spherical(controller, end_sph_coord, radius, tool_pose_ee, helmet_center=HELMET_CENTER_GLOBAL, speed=300):
    """
    Esegue un movimento circolare da una posizione corrente a una posizione finale
    definita da angoli sferici (alpha, beta) attorno al casco.
    La cinematica è sicura dai gimbal lock poiché i poli (beta=0, beta=180) 
    sono esclusi dalle limitazioni di sicurezza.

    'radius' può essere uno scalare (raggio costante) oppure una funzione
    radius(alpha, beta) -> mm (es. variable_helmet_radius): in quest'ultimo caso
    il raggio viene ricalcolato per ogni punto della traiettoria.
    """

    # Normalizza il raggio in una funzione degli angoli (scalare -> costante)
    radius_fn = make_radius_fn(radius)

    def actually_move(start_a, start_b, end_a, end_b, force_ptp=False):
        """
        Esegue fisicamente il movimento. Utilizza move_circle calcolando il midpoint,
        oppure ottimizza con PTP per spostamenti molto piccoli o forzati.
        Il raggio viene valutato puntualmente tramite radius_fn(alpha, beta).
        """
        d_alpha = end_a - start_a
        d_beta = end_b - start_b

        r_end_val = radius_fn(end_a, end_b)
        p_end, r_end = kin.to_helmet_coordinates([r_end_val, end_a, end_b], helmet_center)
        ee_end = kin.compute_ee_pose_for_tool_target(p_end, r_end, tool_pose_ee=tool_pose_ee)

        if force_ptp or (d_alpha**2 + d_beta**2 < 5**2):
            if force_ptp:
                print(f"  [FORCE PTP] Esecuzione forzata verso (alpha={end_a:.1f}°, beta={end_b:.1f}°).")
            else:
                print(f"  [SHORT PATH] Distanza angolare < 5°. Esecuzione ottimizzata PTP verso (alpha={end_a:.1f}°, beta={end_b:.1f}°).")
            controller.move_ptp(ee_end, speed=speed)
        else:
            mid_a = np.mean([start_a, end_a])
            mid_b = np.mean([start_b, end_b])
            r_mid_val = radius_fn(mid_a, mid_b)
            p_mid, r_mid = kin.to_helmet_coordinates([r_mid_val, mid_a, mid_b], helmet_center)
            ee_mid = kin.compute_ee_pose_for_tool_target(p_mid, r_mid, tool_pose_ee=tool_pose_ee)
            
            print(f"  [CIRCLE] Movimento sferico: ({start_a:.1f}°, {start_b:.1f}°) -> ({end_a:.1f}°, {end_b:.1f}°)")
            controller.move_circle(ee_mid, ee_end, speed=speed)

    # --- 1. Calcolo coordinate attuali ---
    ee_pose = controller.robot.tcp_coord
    H_ee_to_glob = kin.create_homogeneous_matrix(ee_pose)
    tool_position_ee = tool_pose_ee[:3]
    
    tool_position_global = kin.homogeneous_trasform(H_ee_to_glob, tool_position_ee)
    start_angles = kin.to_helmet_angles(tool_position_global, helmet_center)

    start_radius = start_angles[0]
    start_alpha, start_beta = start_angles[1], start_angles[2]

    # Raggio target nella posizione angolare attuale (coerente con radius variabile)
    target_start_radius = radius_fn(start_alpha, start_beta)

    if abs(start_radius - target_start_radius) > 20 and False:
        if input(f"  [WARNING] Raggio attuale {start_radius:.1f} mm differisce significativamente dal raggio target {target_start_radius:.1f} mm. \n Premere Invio per continuare comunque, o nope per annullare...").lower() == "nope":
            print("  Movimento annullato dall'utente.")
            return False
    
    end_alpha, end_beta = end_sph_coord[1], end_sph_coord[2]

    # --- 2. Controllo Sicurezza Destinazione ---
    if angles_unsafe(end_alpha, end_beta):
        print(f"  [SKIP] Destinazione (alpha={end_alpha:.1f}°, beta={end_beta:.1f}°) fuori limiti sicurezza.")
        return False

    # --- 3. Controllo Sicurezza Traiettoria ---
    # Se il segmento taglia una zona pericolosa, deviamo passando per l'apice (0, 90) che è sempre sicuro.
    if is_trajectory_unsafe(start_alpha, start_beta, end_alpha, end_beta):
        print(f"  [SAFETY] Traiettoria non sicura. Deviazione tramite l'apice del casco.")
        apex_alpha, apex_beta = 0.0, 90.0
        actually_move(start_alpha, start_beta, apex_alpha, apex_beta)
        actually_move(apex_alpha, apex_beta, end_alpha, end_beta)
        return True

    # --- 4. Suddivisione Archi Ampi (Prevenzione Errori Controller) ---
    # Dato che alpha è limitato a +/- 90, l'arco massimo teorico è 180°.
    # Un solo split a metà garantisce che il robot debba gestire archi <= 90°.
    if abs(end_alpha - start_alpha) > 90 or abs(end_beta - start_beta) > 90:
        print("  [SPLIT] Traiettoria sferica ampia. Suddivisione in due segmenti.")
        mid_a = np.mean([start_alpha, end_alpha])
        mid_b = np.mean([start_beta, end_beta])
        actually_move(start_alpha, start_beta, mid_a, mid_b)
        actually_move(mid_a, mid_b, end_alpha, end_beta)
        return True

    # --- 5. Esecuzione Traiettoria Diretta ---
    actually_move(start_alpha, start_beta, end_alpha, end_beta)
    return True


# ==== PLOTS ====

def plot_variable_helmet_radius(alpha_min=-90.0, alpha_max=90.0, beta_min=0.0, beta_max=180.0, alpha_steps=181, beta_steps=181):
    """Disegna la superficie del raggio variabile del casco in funzione di alpha e beta."""
    alphas = np.linspace(alpha_min, alpha_max, alpha_steps)
    betas = np.linspace(beta_min, beta_max, beta_steps)
    A, B = np.meshgrid(alphas, betas)
    R = np.vectorize(variable_helmet_radius)(A, B)

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(A, B, R, cmap='viridis', edgecolor='none', antialiased=True)

    ax.set_title('Raggio variabile del casco in funzione di alpha e beta')
    ax.set_xlabel('alpha [gradi]')
    ax.set_ylabel('beta [gradi]')
    ax.set_zlabel('raggio [mm]')
    fig.colorbar(surf, shrink=0.5, aspect=10, label='raggio [mm]')

    plt.tight_layout()
    plt.show()
    return fig, ax, surf


def _spherical_line_points(alpha_values, beta_values, helmet_center=HELMET_CENTER_GLOBAL):
    points = []
    unsafe_mask = []
    for a, b in zip(np.atleast_1d(alpha_values).ravel(), np.atleast_1d(beta_values).ravel()):
        r = variable_helmet_radius(a, b)
        p, _ = kin.to_helmet_coordinates([r, a, b], helmet_center)
        points.append(p)
        unsafe_mask.append(angles_unsafe(a, b))

    points = np.vstack(points)
    return points, np.array(unsafe_mask, dtype=bool)


def _plot_mask_segments(ax, points, mask, color, linewidth=1.5, label=None):
    if mask.dtype != bool:
        mask = mask.astype(bool)
    if not np.any(mask):
        return

    # Split into contiguous runs of True values
    indices = np.nonzero(mask)[0]
    breaks = np.where(np.diff(indices) != 1)[0]
    runs = np.split(indices, breaks + 1)

    first = True
    for run in runs:
        run_points = points[run]
        if run_points.shape[0] < 2:
            continue
        if label is not None and first:
            ax.plot(run_points[:, 0], run_points[:, 1], run_points[:, 2], c=color, linewidth=linewidth, label=label)
            first = False
        else:
            ax.plot(run_points[:, 0], run_points[:, 1], run_points[:, 2], c=color, linewidth=linewidth)


def _set_axes_equal_3d(ax):
    """Imposta proporzioni 1:1:1 per un asse 3D."""
    try:
        ax.set_box_aspect((1, 1, 1))
        return
    except AttributeError:
        pass

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    x_range = x_limits[1] - x_limits[0]
    y_range = y_limits[1] - y_limits[0]
    z_range = z_limits[1] - z_limits[0]
    max_range = max(x_range, y_range, z_range) / 2.0
    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    z_middle = np.mean(z_limits)
    ax.set_xlim3d(x_middle - max_range, x_middle + max_range)
    ax.set_ylim3d(y_middle - max_range, y_middle + max_range)
    ax.set_zlim3d(z_middle - max_range, z_middle + max_range)


def _plot_safety_lines(ax, alphas, betas, line_resolution, beta_max):
    plotted_safe = False
    plotted_unsafe = False

    for beta in betas:
        alpha_line = np.linspace(-90.0, 90.0, line_resolution)
        beta_line = np.full_like(alpha_line, beta)
        points, unsafe_mask = _spherical_line_points(alpha_line, beta_line)

        if np.any(~unsafe_mask):
            _plot_mask_segments(ax, points, ~unsafe_mask, 'blue', linewidth=1.8, label='safe' if not plotted_safe else None)
            plotted_safe = True
        if np.any(unsafe_mask):
            _plot_mask_segments(ax, points, unsafe_mask, 'red', linewidth=1.8, label='unsafe' if not plotted_unsafe else None)
            plotted_unsafe = True

    for alpha in alphas:
        beta_line = np.linspace(0.0, beta_max, line_resolution)
        alpha_line = np.full_like(beta_line, alpha)
        points, unsafe_mask = _spherical_line_points(alpha_line, beta_line)

        _plot_mask_segments(ax, points, ~unsafe_mask, 'blue', linewidth=1.8)
        _plot_mask_segments(ax, points, unsafe_mask, 'red', linewidth=1.8)

    ax.scatter(*HELMET_CENTER_GLOBAL, c='black', s=60, marker='x', label='helmet center')
    ax.set_xlabel('x [mm]')
    ax.set_ylabel('y [mm]')
    ax.set_zlabel('z [mm]')
    ax.grid(True, linestyle=':', alpha=0.4)
    ax.legend(loc='upper right')
    _set_axes_equal_3d(ax)


def plot_spherical_safety_grid(alpha_steps=13, beta_steps=9, line_resolution=101, beta_max=120.0, save_path=None, dpi=200, views=None):
    """Disegna una rete di traiettorie sferiche in 3D e colora safe/unsafe.

    La funzione mostra la stessa struttura da varie viste e può salvare
    una immagine per presentazioni.
    """
    if views is None:
        views = [
            ('Perspective', 30, -60, None),
            ('Top', 90, -90, 'z'),
            ('Front', 0, 90, 'y'),
            ('Side', 0, 0, 'x'),
        ]
    elif isinstance(views, tuple) and len(views) == 3:
        views = [(views[0], views[1], views[2], None)]
    elif isinstance(views, tuple) and len(views) == 4:
        views = [views]

    alphas = np.linspace(-90.0, 90.0, alpha_steps)
    betas = np.linspace(0.0, beta_max, beta_steps)

    n_views = len(views)
    ncols = min(2, n_views)
    nrows = int(np.ceil(n_views / ncols))

    subplot_kw = {'projection': '3d'}
    try:
        subplot_kw['proj_type'] = 'ortho'
    except Exception:
        pass

    fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(6 * ncols, 5 * nrows), subplot_kw=subplot_kw)
    if n_views == 1:
        axes = [axes]
    else:
        axes = np.array(axes).reshape(-1)

    for ax, (title, elev, azim, hidden_axis) in zip(axes, views):
        ax.set_title(title)
        _plot_safety_lines(ax, alphas, betas, line_resolution, beta_max)
        ax.view_init(elev=elev, azim=azim)
        if hasattr(ax, 'w_xaxis') and hidden_axis is not None:
            if hidden_axis == 'x':
                ax.w_xaxis.line.set_lw(0)
                ax.w_xaxis.set_ticklabels([])
            elif hidden_axis == 'y':
                ax.w_yaxis.line.set_lw(0)
                ax.w_yaxis.set_ticklabels([])
            elif hidden_axis == 'z':
                ax.w_zaxis.line.set_lw(0)
                ax.w_zaxis.set_ticklabels([])
        _set_axes_equal_3d(ax)

    for ax in axes[n_views:]:
        fig.delaxes(ax)

    plt.tight_layout()
    if save_path is not None:
        fig.savefig(save_path, dpi=dpi, bbox_inches='tight')
    plt.show()
    return fig, axes


if __name__ == "__main__":
    plot_variable_helmet_radius()
    plot_spherical_safety_grid(save_path='spherical_safety_grid.png')