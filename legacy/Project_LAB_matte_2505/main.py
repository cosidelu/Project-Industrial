import enum
from robot_control import RobotController

class RobotState(enum.Enum):
    INIT = 0
    EXECUTE_FIRST_PTP = 1
    ERROR = 2
    EXIT = 3

def main():
    # Inizializzazione dell'interfaccia di controllo
    # L'indirizzo IP di default è settato a 127.0.0.1 nel modulo
    controller = RobotController()  
    current_state = RobotState.INIT
    
    while current_state != RobotState.EXIT:
        
        match current_state:
            case RobotState.INIT:
                print("Stato INIT: Connessione al controller robotico...")
                try:
                    controller.connect()
                    current_state = RobotState.EXECUTE_FIRST_PTP
                except Exception as e:
                    print(f"Errore di connessione: {e}")
                    current_state = RobotState.ERROR
                    
            case RobotState.EXECUTE_FIRST_PTP:
                print("Stato EXECUTE_FIRST_PTP: Esecuzione movimento PTP...")
                try:
                    # Definizione di una posa target [x, y, z, rx, ry, rz]
                    target_pose = [370.0, 300.0, 60.0, 150.0, 0.0, 90.0]
                    
                    # Il metodo move_ptp è bloccante e include l'attesa del completamento
                    controller.move_ptp(target_pose, speed=10)
                    print("Movimento completato con successo.")
                    
                    # Transizione al termine delle operazioni implementate
                    current_state = RobotState.EXIT 
                except TimeoutError as te:
                    print(f"Errore Cinematico: {te}")
                    current_state = RobotState.ERROR
                    
            case RobotState.ERROR:
                print("Stato ERROR: Arresto di emergenza...")
                controller.emergency_stop()
                current_state = RobotState.EXIT
                
            case _:
                print("Stato Indefinito. Transizione a EXIT.")
                current_state = RobotState.EXIT

    print("Chiusura della connessione.")
    controller.disconnect()

if __name__ == "__main__":
    main()