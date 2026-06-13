import numpy as np
from matplotlib import pyplot as plt

import kinematics_v2 as kin
from spherical_movement import variable_helmet_radius, angles_unsafe
from Variables import HELMET_CENTER_GLOBAL

__all__ = ["plot_variable_helmet_radius", "plot_spherical_safety_grid"]


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


def _plot_safety_lines(ax, alphas, betas, line_resolution, beta_max, hidden_axis=None):
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
    ax.set_xlabel('x [mm]' if hidden_axis != 'x' else '')
    ax.set_ylabel('y [mm]' if hidden_axis != 'y' else '')
    ax.set_zlabel('z [mm]' if hidden_axis != 'z' else '')
    ax.grid(True, linestyle=':', alpha=0.4)
    ax.legend(loc='upper right')
    _set_axes_equal_3d(ax)


def plot_variable_helmet_radius(alpha_min=-90.0, alpha_max=90.0, beta_min=0.0, beta_max=180.0, alpha_steps=181, beta_steps=181):
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


def plot_spherical_safety_grid(alpha_steps=13, beta_steps=9, line_resolution=101, beta_max=120.0, save_path=None, dpi=200, views=None):
    if views is None:
        views = [
            ('Perspective', 30, -60, None),
            ('Top', 90, -90, 'z'),
            ('Back', 0, 90, 'y'),
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
        _plot_safety_lines(ax, alphas, betas, line_resolution, beta_max, hidden_axis=hidden_axis)
        ax.view_init(elev=elev, azim=azim)
        if hidden_axis == 'x':
            ax.set_xticks([])
            ax.set_xticklabels([])
            ax.set_xlabel('')
            ax.tick_params(axis='x', which='both', length=0)
        elif hidden_axis == 'y':
            ax.set_yticks([])
            ax.set_yticklabels([])
            ax.set_ylabel('')
            ax.tick_params(axis='y', which='both', length=0)
        elif hidden_axis == 'z':
            ax.set_zticks([])
            ax.set_zticklabels([])
            ax.set_zlabel('')
            ax.tick_params(axis='z', which='both', length=0)
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
