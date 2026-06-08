import tm_libraries.techman as tm
import time
import numpy as np

IP_ADDRESS = "127.0.0.1"

"""
all movements accept speed in mm/s now, with a rough /10 division for ptp
"""
SPEED = 300  # Velocità di movimento in mm/s
PTP_SCALE = 0.1  # Fattore di scala per la velocità PTP (10% della velocità lineare)

class RobotController:
    """Classe per l'astrazione e il controllo bloccante del manipolatore Techman."""
    
    def __init__(self, ip_address=IP_ADDRESS, default_position_j = None):
        # Crea l'oggetto robot e apre la connessione Modbus TCP
        self.robot = tm.TM_Robot(ip_address)
        self.default_tolerance = 0.5
        self.default_timeout = 300.0
        self.secure_wait = 0.1
        self.default_position_j = default_position_j

    def connect(self):
        # Connette il listen node sul robot tramite la porta 5890
        self.robot.connect_listen_node()

    def disconnect(self):
        # Chiude Modbus e le connessioni TCP attive
        self.robot.close_connection()

    def emergency_stop(self):
        # Ferma il robot e pulisce il buffer di esecuzione impostando mode=0
        self.robot.stop(mode=0)

    def default_positioning(self):
        if self.default_position_j is not None:
            self.move_joints(self.default_position_j)

    def _wait_until_pose(self, target_pose, use_joints=False):
        """Metodo privato per la sincronizzazione bloccante della traiettoria.

        Se use_joints=True, confronta lo stato corrente dei giunti invece delle coordinate TCP.
        """
        start = time.time()
        target_arr = np.array(target_pose)

        while True:
            current_arr = np.array(self.robot.joints if use_joints else self.robot.tcp_coord)
            diff = current_arr - target_arr

            if not use_joints:
                # Normalizza la differenza degli angoli (rx, ry, rz) per evitare l'oscillazione intorno ai +-180 gradi
                diff[3:] = (diff[3:] + 180) % 360 - 180

            errors = np.abs(diff)

            if np.max(errors) <= self.default_tolerance:
                time.sleep(self.secure_wait)  # Attende un breve periodo per garantire la stabilità
                return True

            if time.time() - start > self.default_timeout:
                raise TimeoutError(f"Target non raggiunto. Errore: {np.round(errors, 2)}")
            
            time.sleep(0.1) #to not overload the robot with status requests

    def move_ptp(self, pose, speed=SPEED, data_format="CPP"):
        """Esegue un movimento PTP (Punto-Punto) e attende il completamento"""
        speed = speed * PTP_SCALE
        speed = int(speed)  # Assicurati che la velocità sia un intero
        self.robot.ptp(pose, speed, data_format=data_format)
        self._wait_until_pose(pose, use_joints=False)

    def move_joints(self, joints, speed=SPEED):
        speed = speed * PTP_SCALE
        speed = int(speed)  # Assicurati che la velocità sia un intero
        self.robot.ptp(joints, speed, data_format="JPP")
        self._wait_until_pose(joints, use_joints=True)

    def move_line(self, pose, speed=SPEED, data_format="CAP"):
        """Esegue un movimento lineare (Line) e attende il completamento"""
        self.robot.line(pose, speed, data_format=data_format)
        self._wait_until_pose(pose, use_joints=False)

    def move_circle(self, mid_point, end_point, speed=SPEED): #here speed is in mm/s
        """
        Genera un movimento circolare tra la posizione corrente, 
        mid_point e end_point, attendendo il completamento
        """
        self.robot.circle(mid_point, end_point, speed)
        self._wait_until_pose(end_point)

def main():
    print("Esempio di utilizzo del RobotController per eseguire un movimento PTP.")
    controller = RobotController()
    controller.disconnect()
    controller.connect()    

    path = input("Press enter to open visiera")
    time.sleep(5)

    rotation_pose = [333, 550, 452, 90, 90, -90]
    controller.move_ptp(rotation_pose)
    

    print("Movimento PTP completato con successo.")
    controller.disconnect()




# =====================================================================
# BLOCCO DI VALIDAZIONE UNITARIA (UNIT TEST)
# =====================================================================

if __name__ == "__main__":
    controller = RobotController()
    controller.disconnect()
    controller.connect()
    controller.default_positioning()
