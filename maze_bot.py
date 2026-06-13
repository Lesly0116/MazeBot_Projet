# maze_bot.py - Navigation labyrinthe avec anti-répétition de trajectoire
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from collections import deque
import time
import math


print("Connexion à CoppeliaSim...")
client = RemoteAPIClient()
sim = client.require("sim")
print("Connecté !")

print("Récupération des composants...")

robot_handle = sim.getObject("/PioneerP3DX")
left_motor = sim.getObject("/leftMotor")
right_motor = sim.getObject("/rightMotor")

sensor_avant = sim.getObject("/PioneerP3DX/sensor_avant")
sensor_gauche = sim.getObject("/PioneerP3DX/sensor_gauche")
sensor_droite = sim.getObject("/PioneerP3DX/sensor_droite")

arrivee = sim.getObject("/Arrivee")

print("Robot, moteurs, capteurs et arrivée trouvés.")

if sim.getSimulationState() == sim.simulation_stopped:
    print("Lancement de la simulation...")
    sim.startSimulation()
    time.sleep(1)


# =========================
# PARAMÈTRES
# =========================

seuil_arret = 0.40

vitesse_base = 0.30
distance_mur = 0.20
seuil_avant = 0.30
seuil_urgence = 0.10
seuil_lateral = 0.18
k = 1.3

cell_size = 0.35
visited_zones = {}
bad_zones = set()
edge_marks = {}

# Historique réel du parcours
zone_history = deque(maxlen=80)
last_sample_time = time.time()
sample_interval = 0.50

best_distance_arrivee = 999.9
last_progress_time = time.time()
max_time_without_progress = 7.0

prediction_distance = 0.45
FRONT_OFFSET = 0.0

arrived = False
prev_pos = None
start_time = time.time()
last_print_time = time.time()

# Le robot commence en wall-following gauche
mode = "wall_left"

# Empêche de redéclencher l'anti-boucle trop vite
anti_repeat_cooldown_until = 0

# Détection de blocage physique
last_stuck_pos = None
last_stuck_check_time = time.time()
stuck_counter = 0

stuck_check_interval = 0.60
stuck_distance_limit = 0.04
stuck_counter_limit = 4


# Supprimer l'ancienne trajectoire si elle existe
try:
    # Chercher et supprimer l'ancien objet de dessin
    drawing_handle = sim.addDrawingObject(
        sim.drawing_lines,
        3,
        0,
        -1,
        99999,
        [1, 0, 0],
        [0, 0, 0]
    )
    sim.removeDrawingObject(drawing_handle)
    print("Ancienne trajectoire supprimée")
except Exception:
    pass

# Créer une nouvelle trajectoire
drawing_handle = sim.addDrawingObject(
    sim.drawing_lines,
    3,
    0,
    -1,
    99999,
    [1, 0, 0],
    [0, 0, 0]
)

print("Trajectoire rouge activée")


# =========================
# TABLEAU DE RÉSOLUTION DANS COPPELIASIM
# =========================

# À l'arrivée, on arrête complètement la simulation,
# mais on conserve le tableau final visible dans CoppeliaSim.
arreter_simulation_a_arrivee = True
conserver_tableau_final = True

# Si True, la trajectoire rouge est effacée à la fin du programme.
# Ainsi, au prochain lancement, une nouvelle trajectoire propre sera tracée.
effacer_trace_rouge_a_la_fin = True

# Fréquence de mise à jour du tableau.
table_update_interval = 0.25
last_table_update_time = time.time()

simUI = None
ui_tableau = None

try:
    simUI = client.require("simUI")
    
    # Définir le XML du tableau
    xml = """
    <ui title="Résultat de navigation" closeable="false" layout="form">
        <label text="État :" />
        <label id="1" text="EN COURS" />

        <label text="Temps :" />
        <label id="2" text="0.00 s" />

        <label text="Distance arrivée :" />
        <label id="3" text="0.00 m" />

        <label text="Mode :" />
        <label id="4" text="WALL_LEFT" />

        <label text="Zone :" />
        <label id="5" text="-" />
    </ui>
    """

    # Créer le tableau (s'il existe déjà, il sera écrasé/remplacé)
    ui_tableau = simUI.create(xml)

    # Positionner le tableau à droite de la scène (x=700 pour être bien à droite)
    try:
        # Essayer différentes méthodes de positionnement
        simUI.setPosition(ui_tableau, 700, 80)
        print("Tableau positionné à droite de la scène (x=700, y=80)")
    except Exception as e:
        print(f"Impossible de positionner le tableau avec setPosition(x,y): {e}")
        try:
            # Alternative: utiliser une liste
            simUI.setPosition(ui_tableau, [700, 80])
            print("Tableau positionné à droite (format liste)")
        except Exception as e2:
            print(f"Impossible de positionner le tableau: {e2}")
            print("Le tableau reste à sa position par défaut")
    
    print("Tableau UI activé dans CoppeliaSim")

except Exception as e:
    simUI = None
    ui_tableau = None
    print(f"Impossible d'activer le tableau UI dans CoppeliaSim: {e}")


def update_tableau_scene(statut, temps, distance_arrivee, zone=None):
    """
    Met à jour le tableau affiché dans l'interface CoppeliaSim.
    Écrase simplement les anciennes données par les nouvelles.
    """
    if simUI is None or ui_tableau is None:
        return

    if zone is None:
        zone_txt = "-"
    else:
        zone_txt = f"{zone[0]}, {zone[1]}"

    try:
        # Mettre à jour chaque label avec les nouvelles valeurs
        simUI.setLabelText(ui_tableau, 1, str(statut))
        simUI.setLabelText(ui_tableau, 2, f"{temps:.2f} s")
        simUI.setLabelText(ui_tableau, 3, f"{distance_arrivee:.2f} m")
        simUI.setLabelText(ui_tableau, 4, str(mode).upper())
        simUI.setLabelText(ui_tableau, 5, zone_txt)
    except Exception as e:
        print("Erreur lors de la mise à jour du tableau UI :", e)


# =========================
# FONCTIONS DE BASE
# =========================

def read_sensor(sensor):
    try:
        result, distance, detected_point, detected_object, normal = sim.readProximitySensor(sensor)
        if result == 1:
            return True, distance
    except Exception as e:
        print("Erreur capteur :", e)

    return False, 999.9


def stop_robot():
    sim.setJointTargetVelocity(left_motor, 0)
    sim.setJointTargetVelocity(right_motor, 0)


def move_forward(duration=0.35, speed=0.30):
    sim.setJointTargetVelocity(left_motor, speed)
    sim.setJointTargetVelocity(right_motor, speed)
    time.sleep(duration)
    stop_robot()


def move_backward(duration=0.25, speed=-0.45):
    sim.setJointTargetVelocity(left_motor, speed)
    sim.setJointTargetVelocity(right_motor, speed)
    time.sleep(duration)
    stop_robot()


def rotate_right(duration=0.80):
    sim.setJointTargetVelocity(left_motor, 0.75)
    sim.setJointTargetVelocity(right_motor, -0.75)
    time.sleep(duration)
    stop_robot()


def rotate_left(duration=0.80):
    sim.setJointTargetVelocity(left_motor, -0.75)
    sim.setJointTargetVelocity(right_motor, 0.75)
    time.sleep(duration)
    stop_robot()


def rotate_around(duration=1.55):
    sim.setJointTargetVelocity(left_motor, 0.75)
    sim.setJointTargetVelocity(right_motor, -0.75)
    time.sleep(duration)
    stop_robot()


def distance_2d(pos1, pos2):
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    return math.sqrt(dx * dx + dy * dy)


def get_zone_key(position):
    x = round(position[0] / cell_size)
    y = round(position[1] / cell_size)
    return (x, y)


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


def is_open(result, distance, threshold):
    return (not result) or distance > threshold


# =========================
# ORIENTATION ET MÉMOIRE
# =========================

def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def robot_yaw():
    orientation = sim.getObjectOrientation(robot_handle, -1)
    return orientation[2] + FRONT_OFFSET


initial_yaw = robot_yaw()


def heading_index():
    relative = wrap_angle(robot_yaw() - initial_yaw)
    return int(round(relative / (math.pi / 2))) % 4


def direction_angle(direction_index):
    return initial_yaw + direction_index * (math.pi / 2)


def predict_zone_from_direction(robot_pos, direction_index):
    angle = direction_angle(direction_index)

    predicted_pos = [
        robot_pos[0] + math.cos(angle) * prediction_distance,
        robot_pos[1] + math.sin(angle) * prediction_distance,
        robot_pos[2]
    ]

    return get_zone_key(predicted_pos), predicted_pos


def mark_edge(zone, direction_index):
    key = (zone, direction_index)
    edge_marks[key] = edge_marks.get(key, 0) + 1


def relative_to_absolute_direction(relative_direction):
    current_heading = heading_index()

    if relative_direction == "front":
        return current_heading

    if relative_direction == "left":
        return (current_heading + 1) % 4

    if relative_direction == "right":
        return (current_heading - 1) % 4

    if relative_direction == "back":
        return (current_heading + 2) % 4

    return current_heading


def remember_zone(zone):
    visited_zones[zone] = visited_zones.get(zone, 0) + 1
    zone_history.append(zone)


def recent_repetition_detected(current_zone):
    """
    Détecte une trajectoire répétée.
    Cas détectés :
    1. même zone visitée trop souvent ;
    2. petit groupe de zones répété ;
    3. séquence récente identique à la précédente.
    """
    history = list(zone_history)

    if visited_zones.get(current_zone, 0) >= 5:
        return True

    if len(history) >= 20:
        last_20 = history[-20:]
        if len(set(last_20)) <= 5:
            return True

    if len(history) >= 16:
        last_8 = history[-8:]
        previous_8 = history[-16:-8]
        if last_8 == previous_8:
            return True

    return False


def mark_recent_path_as_bad():
    """
    Marque les dernières zones comme mauvaises.
    Cela empêche le robot de considérer ce parcours comme acceptable.
    """
    recent = list(zone_history)[-20:]

    for zone in recent:
        bad_zones.add(zone)

    print(f"Trajet répétitif marqué mauvais : {len(recent)} zones")


def choose_best_direction(robot_pos, arrivee_pos, front_open, left_open, right_open, force_escape=False):
    current_zone = get_zone_key(robot_pos)

    candidates = []

    if front_open:
        candidates.append("front")

    if left_open:
        candidates.append("left")

    if right_open:
        candidates.append("right")

    if force_escape:
        candidates.append("back")

    if not candidates:
        candidates.append("back")

    best_direction = None
    best_score = 999999

    print("\nChoix par mémoire :")

    for relative_direction in candidates:
        absolute_direction = relative_to_absolute_direction(relative_direction)
        predicted_zone, predicted_pos = predict_zone_from_direction(robot_pos, absolute_direction)

        edge_count = edge_marks.get((current_zone, absolute_direction), 0)
        zone_visits = visited_zones.get(predicted_zone, 0)

        bad_penalty = 120 if predicted_zone in bad_zones else 0
        goal_distance = distance_2d(predicted_pos, arrivee_pos)

        turn_penalty = 0.0
        if relative_direction in ["left", "right"]:
            turn_penalty = 0.5
        elif relative_direction == "back":
            turn_penalty = 2.0

        score = (
            edge_count * 80.0 +
            zone_visits * 12.0 +
            bad_penalty +
            goal_distance +
            turn_penalty
        )

        print(
            f"  {relative_direction.upper()} | "
            f"dir={absolute_direction} | "
            f"passages={edge_count} | "
            f"zone={predicted_zone} | "
            f"visites={zone_visits} | "
            f"mauvaise={predicted_zone in bad_zones} | "
            f"score={score:.2f}"
        )

        if score < best_score:
            best_score = score
            best_direction = relative_direction

    print(f"Direction choisie : {best_direction.upper()}")
    return best_direction


def execute_direction(direction, robot_pos):
    current_zone = get_zone_key(robot_pos)
    absolute_direction = relative_to_absolute_direction(direction)

    mark_edge(current_zone, absolute_direction)

    if direction == "front":
        move_forward(0.45, vitesse_base)

    elif direction == "left":
        rotate_left(0.80)
        move_forward(0.45, vitesse_base)

    elif direction == "right":
        rotate_right(0.80)
        move_forward(0.45, vitesse_base)

    elif direction == "back":
        move_backward(0.35, -0.50)
        rotate_around(1.60)
        move_forward(0.40, vitesse_base)


# =========================
# BLOCAGE ET ANTI-RÉPÉTITION
# =========================

def robot_is_stuck(robot_pos):
    global last_stuck_pos
    global last_stuck_check_time
    global stuck_counter

    now = time.time()

    if last_stuck_pos is None:
        last_stuck_pos = robot_pos[:]
        last_stuck_check_time = now
        return False

    if now - last_stuck_check_time < stuck_check_interval:
        return False

    moved_distance = distance_2d(robot_pos, last_stuck_pos)

    if moved_distance < stuck_distance_limit:
        stuck_counter += 1
        print(
            f"Blocage possible : déplacement faible "
            f"{moved_distance:.3f} m ({stuck_counter}/{stuck_counter_limit})"
        )
    else:
        stuck_counter = 0

    last_stuck_pos = robot_pos[:]
    last_stuck_check_time = now

    return stuck_counter >= stuck_counter_limit


def force_escape_repetition(robot_pos, arrivee_pos, front_open, left_open, right_open):
    """
    Force le robot à abandonner un trajet répété.
    Il marque le chemin récent comme mauvais, change de mode,
    recule, puis choisit une autre direction.
    """
    global mode
    global anti_repeat_cooldown_until
    global last_progress_time
    global stuck_counter

    print("\nTRAJECTOIRE RÉPÉTÉE DÉTECTÉE → abandon du trajet courant")

    current_zone = get_zone_key(robot_pos)
    bad_zones.add(current_zone)
    mark_recent_path_as_bad()

    # Changer de stratégie de mur
    if mode == "wall_left":
        mode = "wall_right"
        print("Changement de stratégie : WALL RIGHT")
    else:
        mode = "wall_left"
        print("Changement de stratégie : WALL LEFT")

    stop_robot()
    time.sleep(0.20)

    # Back plus franc pour sortir du couloir répétitif
    move_backward(0.60, -0.55)

    # Priorité : choisir un côté ouvert au lieu de reprendre le même front
    if mode == "wall_right" and right_open:
        direction = "right"
    elif mode == "wall_left" and left_open:
        direction = "left"
    else:
        direction = choose_best_direction(
            robot_pos,
            arrivee_pos,
            front_open,
            left_open,
            right_open,
            force_escape=True
        )

    execute_direction(direction, robot_pos)

    stuck_counter = 0
    last_progress_time = time.time()

    # Pendant quelques secondes, on évite de redéclencher immédiatement
    anti_repeat_cooldown_until = time.time() + 4.0


def escape_stuck(robot_pos, arrivee_pos, front_open, left_open, right_open):
    global stuck_counter
    global last_progress_time

    current_zone = get_zone_key(robot_pos)
    bad_zones.add(current_zone)

    print("\nRobot coincé → back + nouvelle direction")
    print(f"Zone bloquée marquée mauvaise : {current_zone}")

    stop_robot()
    time.sleep(0.20)

    move_backward(0.60, -0.55)

    direction = choose_best_direction(
        robot_pos,
        arrivee_pos,
        front_open,
        left_open,
        right_open,
        force_escape=True
    )

    execute_direction(direction, robot_pos)

    stuck_counter = 0
    last_progress_time = time.time()


last_zone = get_zone_key(sim.getObjectPosition(robot_handle, -1))


# =========================
# BOUCLE PRINCIPALE
# =========================

try:
    while not arrived:
        robot_pos = sim.getObjectPosition(robot_handle, -1)
        arrivee_pos = sim.getObjectPosition(arrivee, -1)

        if prev_pos is not None:
            sim.addDrawingObjectItem(drawing_handle, [
                prev_pos[0], prev_pos[1], prev_pos[2],
                robot_pos[0], robot_pos[1], robot_pos[2]
            ])

        prev_pos = robot_pos

        distance_arrivee = distance_2d(robot_pos, arrivee_pos)
        current_zone = get_zone_key(robot_pos)

        # =========================
        # MISE À JOUR DU TABLEAU DANS LA SCÈNE
        # =========================
        temps_ecoule = time.time() - start_time

        if time.time() - last_table_update_time > table_update_interval:
            update_tableau_scene("EN COURS", temps_ecoule, distance_arrivee, current_zone)
            last_table_update_time = time.time()

        # =========================
        # ARRÊT À L'ARRIVÉE
        # =========================
        if distance_arrivee < seuil_arret:
            stop_robot()
            temps_total = time.time() - start_time

            print("\n" + "=" * 45)
            print("ROBOT ARRIVÉ À DESTINATION !")
            print(f"Distance finale : {distance_arrivee:.2f} m")
            print(f"Temps de résolution : {temps_total:.2f} secondes")
            print("=" * 45)

            # Affichage du résultat final dans le tableau de la scène
            update_tableau_scene("ARRIVE", temps_total, distance_arrivee, current_zone)

            arrived = True

            # Petite pause pour laisser le temps au tableau final de se mettre à jour.
            time.sleep(0.50)

            if arreter_simulation_a_arrivee:
                # Arrêt complet de la simulation à l'arrivée.
                sim.stopSimulation()
            else:
                # Option alternative si tu veux figer la scène sans l'arrêter.
                sim.pauseSimulation()

            break

        # current_zone a déjà été calculée plus haut pour le tableau de la scène

        # =========================
        # MÉMORISATION DES ZONES
        # =========================
        if time.time() - last_sample_time > sample_interval:
            remember_zone(current_zone)
            last_sample_time = time.time()

        zone_visits = visited_zones.get(current_zone, 0)

        if current_zone != last_zone:
            current_heading = heading_index()
            mark_edge(last_zone, current_heading)
            last_zone = current_zone

        if distance_arrivee < best_distance_arrivee - 0.08:
            best_distance_arrivee = distance_arrivee
            last_progress_time = time.time()

        no_progress = time.time() - last_progress_time > max_time_without_progress
        repeated_path = recent_repetition_detected(current_zone)

        # Lecture des capteurs
        result_avant, dist_avant = read_sensor(sensor_avant)
        result_gauche, dist_gauche = read_sensor(sensor_gauche)
        result_droite, dist_droite = read_sensor(sensor_droite)

        front_open = is_open(result_avant, dist_avant, seuil_avant)
        left_open = is_open(result_gauche, dist_gauche, seuil_lateral)
        right_open = is_open(result_droite, dist_droite, seuil_lateral)

        if time.time() - last_print_time > 0.30:
            print(
                f"Zone: {current_zone} visitée {zone_visits} fois | "
                f"Mode: {mode} | "
                f"Arrivée: {distance_arrivee:.2f} m | "
                f"Avant: {result_avant} {dist_avant:.2f} | "
                f"Gauche: {result_gauche} {dist_gauche:.2f} | "
                f"Droite: {result_droite} {dist_droite:.2f}"
            )
            last_print_time = time.time()

        # =========================
        # BLOCAGE PHYSIQUE
        # =========================
        if robot_is_stuck(robot_pos):
            escape_stuck(robot_pos, arrivee_pos, front_open, left_open, right_open)
            continue

        # =========================
        # TRAJECTOIRE RÉPÉTÉE
        # =========================
        if time.time() > anti_repeat_cooldown_until:
            if repeated_path or no_progress:
                force_escape_repetition(
                    robot_pos,
                    arrivee_pos,
                    front_open,
                    left_open,
                    right_open
                )
                continue

        # =========================
        # IMPASSE
        # =========================
        if not front_open and not left_open and not right_open:
            print("\nImpasse détectée → demi-tour")
            bad_zones.add(current_zone)

            direction = choose_best_direction(
                robot_pos,
                arrivee_pos,
                front_open,
                left_open,
                right_open,
                force_escape=True
            )

            execute_direction(direction, robot_pos)
            last_progress_time = time.time()
            continue

        # =========================
        # INTERSECTION
        # =========================
        open_count = 0

        if front_open:
            open_count += 1
        if left_open:
            open_count += 1
        if right_open:
            open_count += 1

        if open_count >= 2:
            print("\nIntersection détectée → choix par mémoire")

            direction = choose_best_direction(
                robot_pos,
                arrivee_pos,
                front_open,
                left_open,
                right_open,
                force_escape=False
            )

            execute_direction(direction, robot_pos)
            continue

        # =========================
        # WALL-FOLLOWING NORMAL
        # =========================
        vitesse_gauche = vitesse_base
        vitesse_droite = vitesse_base

        if mode == "wall_left":

            if result_avant and dist_avant < seuil_avant:
                move_backward(0.20)
                vitesse_gauche = 0.75
                vitesse_droite = -0.60

            elif result_gauche and dist_gauche < seuil_urgence:
                vitesse_gauche = 0.70
                vitesse_droite = 0.20

            elif result_gauche:
                erreur = distance_mur - dist_gauche
                correction = k * erreur

                vitesse_gauche = vitesse_base + correction
                vitesse_droite = vitesse_base - correction

            else:
                vitesse_gauche = 0.20
                vitesse_droite = 0.65

        else:

            if result_avant and dist_avant < seuil_avant:
                move_backward(0.20)
                vitesse_gauche = -0.60
                vitesse_droite = 0.75

            elif result_droite and dist_droite < seuil_urgence:
                vitesse_gauche = 0.20
                vitesse_droite = 0.70

            elif result_droite:
                erreur = distance_mur - dist_droite
                correction = k * erreur

                vitesse_gauche = vitesse_base - correction
                vitesse_droite = vitesse_base + correction

            else:
                vitesse_gauche = 0.65
                vitesse_droite = 0.20

        vitesse_gauche = clamp(vitesse_gauche, -0.8, 1.2)
        vitesse_droite = clamp(vitesse_droite, -0.8, 1.2)

        sim.setJointTargetVelocity(left_motor, vitesse_gauche)
        sim.setJointTargetVelocity(right_motor, vitesse_droite)

        time.sleep(0.05)


except KeyboardInterrupt:
    print("\nSimulation interrompue manuellement")


finally:
    try:
        stop_robot()
    except Exception:
        pass

    # La trajectoire rouge est supprimée à la fin pour éviter que les anciennes
    # lignes restent visibles quand on relance une nouvelle simulation.
    if effacer_trace_rouge_a_la_fin:
        try:
            sim.addDrawingObjectItem(drawing_handle, None)
            sim.removeDrawingObject(drawing_handle)
            print("Trajectoire rouge effacée")
        except Exception:
            pass

    # On conserve le tableau final après l'arrivée
    if arrived and conserver_tableau_final:
        print("Tableau conservé après l'arrivée")
    else:
        # Supprimer le tableau seulement si on veut vraiment le nettoyer
        # Par défaut, on le garde pour la prochaine simulation
        pass

    print("Fin de la simulation")