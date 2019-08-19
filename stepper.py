import sys
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

step_states_4 = [
        [1,0,0,1],
        [1,1,0,0],
        [0,1,1,0],
        [0,0,1,1]]

step_states_8 = [
        [1,0,0,0],
        [1,1,0,0],
        [0,1,0,0],
        [0,1,1,0],
        [0,0,1,0],
        [0,0,1,1],
        [0,0,0,1],
        [1,0,0,1]]

class StepperMotor:

    def __init__(self, gpio_pins, step_states):
        self.gpio_pins = gpio_pins
        self.step_states = step_states
        self.step_numstates = len(step_states)
        self.step_state = 0
        step_timerid = 4
        status_timerid = 2
        self.registered_pins = False

    def __configure_gpio(self):
        if self.registered_pins is False:
            print("Registering Stepper Motor")
            for pin in self.gpio_pins:
                GPIO.setup(pin,GPIO.OUT)
            self.registered_pins = True

    def __stop(self):
        print("Stopping Motor")
        GPIO.output(self.gpio_pins[0], True)
        for i in range(1, len(self.gpio_pins)):
            GPIO.output(self.gpio_pins[i], False)

    def release(self):
        print("Releasing Motor")
        for pin in self.gpio_pins:
            GPIO.output(pin, False)

    def take_step(self, distance, step_direction=1, speed=100):
        self.__configure_gpio()
        cur_step = 0
        while distance > 0:
            self.step_state += step_direction
            cur_step += step_direction
            if self.step_state >= self.step_numstates:
                self.step_state = 0
            elif self.step_state < 0:
                self.step_state = self.step_numstates - 1

            # print("Issuing step state {} of {}".format(self.step_state, self.step_numstates))

            for i in range(4):
                pin = self.gpio_pins[i]
                state = self.step_states[self.step_state][i]
                # print("Setting {} of {} gpio pin {} to {}".format(i, 4, pin, state))
                GPIO.output(pin, state)

            distance -= 1
            if distance > 0:
                time.sleep(speed/float(1000))

        self.__stop()

    def close(self):
        if self.registered_pins:
            print("Unregistering Stepper Motor")
            for pin in self.gpio_pins:
                GPIO.cleanup(pin)
            self.registered_pins = False



gpio_pins = [4,17,27,22]
stepper_motor = StepperMotor(gpio_pins, step_states_8)

stepper_motor.take_step(15000, -1, 0.35)
time.sleep(2)
stepper_motor.release()
time.sleep(2)
stepper_motor.close()