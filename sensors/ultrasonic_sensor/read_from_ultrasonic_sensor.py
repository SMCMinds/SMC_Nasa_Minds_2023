import sys
import time
from grove.gpio import GPIO

usleep = lambda x: time.sleep(x / 1000000.0)

_TIMEOUT1 = 1000            # What units are we dealing with here? ms?
_TIMEOUT2 = 10000

class GroveUltrasonicRanger(object):
    def __init__(self, pin):
        self.dio =GPIO(pin)         # set pin number on initialization

    def _get_distance(self):
        self.dio.dir(GPIO.OUT)      # ?
        self.dio.write(0)           # ?
        usleep(2)                   # setting a sleep time, lower bound?
        self.dio.write(1)           # ?
        usleep(10)                  # setting a sleep time, upper bound?
        self.dio.write(0)           # writing setting to "dio"?

        self.dio.dir(GPIO.IN)       # ?

        t0 = time.time()            
        count = 0
        while count < _TIMEOUT1:    
            if self.dio.read():
                break
            count += 1
        if count >= _TIMEOUT1:
            return None

        t1 = time.time()
        count = 0
        while count < _TIMEOUT2:
            if not self.dio.read():
                break
            count += 1
        if count >= _TIMEOUT2:
            return None

        t2 = time.time()

        dt = int((t1 - t0) * 1000000)
        if dt > 530:
            return None

        # distance = ((t2 - t1) * 1000000 / 29 / 2)    # cm
        distance = ((t2 - t1) * 1000000 / 29 / 2) * 0.393701   # in
        return distance

    def get_distance(self):
        while True:
            dist = self._get_distance()
            if dist:
                return dist

Grove = GroveUltrasonicRanger

def main():
    if len(sys.argv) < 2:
        print('Usage: {} pin_number'.format(sys.argv[0]))
        sys.exit(1)

    sonar = GroveUltrasonicRanger(int(sys.argv[1]))

    print('Detecting distance...')
    while True:
        #print('{} cm'.format(sonar.get_distance()))    #cm
        print('{} in'.format(sonar.get_distance()))     #in
        time.sleep(1)

if __name__ == '__main__':
    main()