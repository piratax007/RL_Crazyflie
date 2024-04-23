from CrazyflieAPI import drone


def main():
    actions = drone.control.applicable_pwm()
    drone.setParam("motorPowerSet.enable", 1)
    drone.setParam("motorPowerSet.m1", actions[0])
    drone.setParam("motorPowerSet.m2", actions[1])
    drone.setParam("motorPowerSet.m3", actions[2])
    drone.setParam("motorPowerSet.m4", actions[3])
    drone.setParam("motorPowerSet.enable", 0)


if __name__ == '__main__':
    main()
