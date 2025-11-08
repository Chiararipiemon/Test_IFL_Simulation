import imfusion as imf
import imfusion.ultrasound as us
import numpy as np

# ===== PARAMETRI CONVEX =====
USE_CONVEX_GEOMETRY = True      # metti False se vuoi tornare "generico"
PROBE_WIDTH_MM      = 38.0
DEPTH_MM            = 120.0
OPENING_ANGLE_DEG   = 60.0
LONG_RADIUS_MM      = 100.0     # tuning
OFFSET_ALONG_BEAM   = 30.0      # distanza tra curva e centro frame (mm)
SWEEP_TIME_SEC      = 3.0
# =============================


def create_empty_image():
    """Crea una slice 128x128 con spacing metrico fittizio."""
    img_desc = imf.ImageDescriptor(imf.PixelType.UBYTE, 128, 128, 1, 1)
    img_desc.spacing = np.array([0.5, 0.5, 1.0])  # mm/pixel, cambia se serve
    img_desc.is_metric = True
    return imf.SharedImage(imf.MemImage(img_desc))


def get_tracking():
    """
    Restituisce la TrackingSequence da usare:
    - prima cerca 'Tracking Smooth from CSV'
    - altrimenti usa la prima TrackingSequence trovata.
    """
    preferred = None
    fallback = None
    for el in imf.app.data_model:
        if isinstance(el, imf.TrackingSequence):
            name = getattr(el, "name", "")
            if "Tracking Smooth from CSV" in name:
                preferred = el
            if fallback is None:
                fallback = el
    return preferred if preferred is not None else fallback


def configure_convex_geometry(sweep):
    """
    Prova a configurare la FrameGeometry dello sweep come convex.
    Funziona se l'oggetto frame_geometry espone i campi relativi;
    altrimenti stampa un avviso (puoi rifinire da GUI).
    """
    try:
        fgm = None
        for c in sweep.components:
            if isinstance(c, us.FrameGeometryMetadata):
                fgm = c
                break
        if fgm is None:
            print("Convex: nessun FrameGeometryMetadata trovato nello sweep.")
            return

        fg = fgm.frame_geometry
        if fg is None:
            print("Convex: frame_geometry è None.")
            return

        # Imposta parametri se esistono
        if hasattr(fg, "depth"):
            fg.depth = DEPTH_MM
        if hasattr(fg, "opening_angle"):
            fg.opening_angle = OPENING_ANGLE_DEG
        if hasattr(fg, "long_radius"):
            fg.long_radius = LONG_RADIUS_MM
        if hasattr(fg, "short_radius") and hasattr(fg, "opening_angle"):
            fg.short_radius = PROBE_WIDTH_MM / (
                2.0 * np.sin(np.deg2rad(OPENING_ANGLE_DEG))
            )

        # Alcune versioni usano width per linear; lo teniamo coerente
        if hasattr(fg, "width"):
            fg.width = PROBE_WIDTH_MM

        if hasattr(fg, "top_down"):
            fg.top_down = True

        print("Convex: frame geometry configurata (per quanto supportato dall'API).")

    except Exception as e:
        print("Convex: impossibile configurare da script:", e)
        print("       Usa 'Frame Geometry Properties' da GUI sullo sweep per impostare Convex.")


def build_sweep_ortho():
    """
    Crea una UltrasoundSweep 'Synthetic Sweep ORTHO' con:
    - frame ortogonali alla traiettoria (normale = tangente del tracking)
    - fascio (asse y immagine) verso il basso in modo consistente
    - offset lungo il fascio
    - (opzionale) frame geometry settata per immagini convex.
    """
    tracking = get_tracking()
    if tracking is None:
        print("Error: no TrackingSequence in the data model. "
              "Crea prima 'Tracking Smooth from CSV'.")
        return

    nframes = tracking.size
    print("Using tracking:", getattr(tracking, "name", "unnamed"), "size =", nframes)

    if nframes < 3:
        print("Error: tracking too short")
        return

    sweep = us.UltrasoundSweep()
    sweep.name = "Synthetic Sweep ORTHO"

    tracking_copy = imf.TrackingSequence()

    timestep = SWEEP_TIME_SEC / float(nframes - 1)

    # verso "giù" globale preferito (inverti segno se vuoi l'altro lato)
    down_global = np.array([0.0, -1.0, 0.0], dtype=float)

    for i in range(1, nframes - 1):
        m_prev = tracking.raw_matrix(i - 1)
        m      = tracking.raw_matrix(i)
        m_next = tracking.raw_matrix(i + 1)
        t = i * timestep

        p_prev = m_prev[:3, 3]
        p      = m[:3, 3]
        p_next = m_next[:3, 3]

        # Tangente alla traiettoria (diventa normale del piano immagine)
        tangent = p_next - p_prev
        nrm_t = np.linalg.norm(tangent)
        if nrm_t < 1e-6:
            continue
        tangent /= nrm_t

        # Asse profondità: proiezione di down_global sul piano ortogonale alla tangente
        depth_axis = down_global - np.dot(down_global, tangent) * tangent
        nrm_d = np.linalg.norm(depth_axis)
        if nrm_d < 1e-6:
            # fallback se quasi paralleli
            alt = np.array([0.0, 0.0, -1.0], dtype=float)
            depth_axis = alt - np.dot(alt, tangent) * tangent
            nrm_d = np.linalg.norm(depth_axis)
            if nrm_d < 1e-6:
                continue
        depth_axis /= nrm_d

        # Asse laterale = depth × normal per chiudere terna destrorsa
        lateral_axis = np.cross(depth_axis, tangent)
        nrm_l = np.linalg.norm(lateral_axis)
        if nrm_l < 1e-6:
            continue
        lateral_axis /= nrm_l

        # Matrice di posa della slice:
        # x = laterale (lungo sonda), y = profondità (verso il basso), z = tangente (normale al piano)
        out = np.eye(4, dtype=float)
        out[:3, 0] = lateral_axis
        out[:3, 1] = depth_axis
        out[:3, 2] = tangent

        # Posizione: sulla traiettoria + offset lungo il fascio
        out[:3, 3] = p + depth_axis * OFFSET_ALONG_BEAM

        tracking_copy.add(out, t, 1.0)
        sweep.add(create_empty_image())
        sweep.set_timestamp(t, sweep.size - 1)

    if tracking_copy.size == 0:
        print("Error: no valid frames created for sweep")
        return

    sweep.add_tracking(tracking_copy)
    sweep.properties.set_param("topDown", True)

    # Prova a settare convex
    if USE_CONVEX_GEOMETRY:
        configure_convex_geometry(sweep)

    imf.app.data_model.add(sweep)
    print("Synthetic Sweep ORTHO added to data model with",
          tracking_copy.size, "frames.")
