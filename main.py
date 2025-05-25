def run_simulation(drones, outputs):
    import time
    drone_state = integrator.state
    last = time.perf_counter()
    while True:
        # one thread per drone ?
        rc_inputs = controller.read_inputs()
        rpm, target_dt = flight_controler.update(rc_inputs)
        now = time.perf_counter()
        dt = now - last
        last = now
        drone_state = integrator.integrate(drone_state, drone_params, rpm, dt)
        for o in outputs:
            o.output(drone_state) # barrier for output
        time.sleep(max(0, target_dt - dt))
        


if __name__ == "__main__":##
    drone0 = Drone("drone0.yaml", StraightLineController(), FlightController("flight_controller.yaml"))
    drone1 = Drone("drone1.yaml", XboxController(), BetaflightController())


    run_simulation((drone0, drone1), (Pandas3dOutput(),))


