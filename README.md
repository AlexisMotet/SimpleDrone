# Boucle de simulation

```python
while True:

    for drone in drones:

        rc_inputs = drone.controller.read_inputs() # renvoie les commandes du radio controller

        rpm, realtime_dt = drone.flight_controler.update(rc_inputs) # renvoie les vitesses de rotation des moteurs

        drone.state = integrate(drone.state, drone.params, rpm, realtime_dt) # calcule la nouvelle position du drone

    render(drones)
```

# render

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





