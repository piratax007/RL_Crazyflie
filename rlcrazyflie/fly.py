from CrazyflieAPI import crazyflie


def main():
    crazyflie.drone.setParam("motorPowerSet.enable", 1)

    actions = crazyflie.control.applicable_pwm()

    crazyflie.drone.setParam("motorPowerSet.m1", actions[0])
    crazyflie.drone.setParam("motorPowerSet.m2", actions[1])
    crazyflie.drone.setParam("motorPowerSet.m3", actions[2])
    crazyflie.drone.setParam("motorPowerSet.m4", actions[3])

    crazyflie.drone.land(targetHeight=0.04, duration=2.5)

    crazyflie.drone.setParam("motorPowerSet.enable", 0)


if __name__ == '__main__':
    main()
