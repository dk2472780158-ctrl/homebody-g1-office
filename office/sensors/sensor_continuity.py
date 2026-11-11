"""Pure timing guards: never change source timestamps or robot state."""
import math


class ContinuityError(ValueError):
    pass


class BacklogGuard:
    """Bound growing transport delay without assuming synchronized clocks."""
    def __init__(self,limit=.75):
        self.limit=limit;self.best_offset={};self.lag={}
    def observe(self,topic,stamp,received):
        offset=received-stamp
        self.best_offset[topic]=min(offset,self.best_offset.get(topic,offset))
        self.lag[topic]=max(0.,offset-self.best_offset[topic])
        if self.lag[topic]>self.limit:
            raise ContinuityError(f'{topic} sensor backlog grew {self.lag[topic]:.3f}s')
        return self.lag[topic]


class SensorContinuity:
    def __init__(self):
        self.last = {}
        self.first_received = None
        self.limits = {'cloud': 1.0, 'imu': .5}
        self.dropped_reordered_imu = 0
        self.consecutive_reordered_imu = 0
        self.initialized = False

    def observe(self, topic, stamp, received):
        if not math.isfinite(stamp) or not math.isfinite(received):
            raise ContinuityError('nonfinite sensor timestamp')
        if topic in self.last:
            previous_stamp, previous_received = self.last[topic]
            delta = stamp - previous_stamp
            # Discard a bounded, isolated late IMU packet instead of feeding a
            # timestamp regression into FAST-LIO or restarting the whole map.
            if topic=='imu' and -.03<=delta<=0 and received-previous_received<.1:
                self.consecutive_reordered_imu+=1
                if self.consecutive_reordered_imu<=2:
                    self.dropped_reordered_imu+=1
                    return False
            if delta <= 0 or delta > self.limits[topic]:
                raise ContinuityError(f'{topic} source discontinuity: {delta:.6f}s')
            if received - previous_received > 2:
                raise ContinuityError(f'{topic} receive interruption')
        self.last[topic] = (stamp, received)
        if topic=='imu':self.consecutive_reordered_imu=0
        if self.first_received is None:
            self.first_received = received
        return True

    def check_idle(self, now):
        if self.first_received is None:
            return
        if now - self.first_received > 5 and len(self.last) != 2:
            raise ContinuityError('one sensor stream is missing')
        for topic, (_, received) in self.last.items():
            if now - received > 2:
                raise ContinuityError(f'{topic} receive timeout')

    def ready(self, now):
        return (len(self.last) == 2 and now-self.first_received >= 1
                and all(now-received < .5 for _, received in self.last.values())
                and abs(self.last['cloud'][0]-self.last['imu'][0]) < .5)

    def can_forward(self, now):
        # Synchronize startup once. A sparse cloud must not repeatedly suppress
        # valid IMU samples; continuity, backlog and idle guards remain active.
        self.initialized = self.initialized or self.ready(now)
        return self.initialized
