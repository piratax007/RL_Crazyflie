from rlcrazyflie.CrazyflieAPI import crazyflie
import rclpy


def main():
    crazyflie.drone.setParam("motorPowerSet.enable", 1)

    while rclpy.ok():
        actions = crazyflie.control.applicable_pwm()
        print(f"""
        Observarions: {crazyflie.control.observation_space}
        PWM Control: {actions}
        """)

    # crazyflie.drone.setParam("motorPowerSet.m1", actions[0])
    # crazyflie.drone.setParam("motorPowerSet.m2", actions[1])
    # crazyflie.drone.setParam("motorPowerSet.m3", actions[2])
    # crazyflie.drone.setParam("motorPowerSet.m4", actions[3])
    #
    # crazyflie.drone.land(targetHeight=0.04, duration=2.5)

    crazyflie.drone.setParam("motorPowerSet.enable", 0)


if __name__ == '__main__':
    main()
