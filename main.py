from src.simulation import Simulation


def main():
    simulation = Simulation(
        "assets/scenes/playground.xml"
    )

    simulation.run()


if __name__ == "__main__":
    main()
