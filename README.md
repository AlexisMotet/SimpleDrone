# TODO

- ~~Implémenter un controller à partir du clavier~~
- Implémenter un IMU réaliste: biais à ajouter https://github.com/Aceinna/gnss-ins-sim, comparer à des vraies données
- Tester sur un banked turn
- ~~Implémenter un EKF pour filtrer les sorties de l'IMU~~ (AHRS lib)
- Implémenter son propre EKF ?


- ~~Créer une structure de données DroneParams qui puisse renvoyer la matrice "mixer"~~
- Implémenter un modèle de moteur
- Tester le flight controller





# Boucle de simulation

```python
while True:

    for drone in drones:

        rc_inputs = drone.controller.read_inputs(drone.state, drone.params) # renvoie les commandes du radio controller

        rpm, realtime_dt = drone.flight_controler.update(rc_inputs) # renvoie les vitesses de rotation des moteurs

        drone.state = integrate(drone.state, drone.params, rpm, realtime_dt) # calcule la nouvelle position du drone

    render(drones)
```
# Controller

Un clavier, une manette Xbox, un véritable RC, ou un algorithme.
Prend en entrée l'état dynamique du drone, le modèle du drone et ses paramètres, renvoie quatre commandes:
- Une poussée (throttle, entre 0 et 1)
- Une commande de roll
- Une commande de pitch
- Une commande de yaw rate

# Flight Controller

Logiciel embarqué qui convertit la commande de l'utilisateur en vitesse de rotation des moteurs.
Peut être simulé en Python.

Prend en entrée les quatre commandes, l'état actuel du drone, etc.
Renvoie des vitesses de rotation par moteur et le temps qu'il lui a fallu pour calculer ces vitesses (fréquence d'update du FC)

# Integration

Combine l'état précédent du drone et les vitesses de rotation des moteurs pour obtenir le nouvel état après dt.

# Rendering

Affiche les drones avec Panda3D.

https://www.panda3d.org/

## inputs

Des structures "Drone" qui contiennent:
- un temps
- position X, Y, Z
- vitesse de rotation des moteurs
- position des moteurs par rapport au centre de gravité du drone
- fichier 3d du drone ?

## output

- Affichage des drones en temps réel
- Les hélices tournent à la bonne vitesse
- Affichage dans un monde 3d ~réaliste

## notes

- render et le reste du code doivent être indépendants
